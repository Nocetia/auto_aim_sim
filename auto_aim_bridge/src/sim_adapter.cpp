#include "sim_adapter.hpp"

#include <algorithm>
#include <array>
#include <cmath>
#include <fstream>
#include <limits>
#include <optional>
#include <set>
#include <stdexcept>
#include <utility>
#include <vector>

#include <opencv2/imgproc.hpp>
#include <yaml-cpp/yaml.h>

#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"
#include "tasks/auto_aim/yolo.hpp"
#include "uv_control_adapter.hpp"

namespace auto_aim_bridge
{
namespace
{

double finite_number(const Json & value, const std::string & field)
{
  if (!value.is_number()) {
    throw std::invalid_argument(field + " must be a number");
  }

  const double number = value.get<double>();

  if (!std::isfinite(number)) {
    throw std::invalid_argument(field + " must be finite");
  }

  return number;
}

std::vector<double> numeric_array(
  const Json & value,
  std::size_t expected_size,
  const std::string & field)
{
  if (!value.is_array() || value.size() != expected_size) {
    throw std::invalid_argument(
      field + " must contain " + std::to_string(expected_size) + " numbers");
  }

  std::vector<double> numbers;
  numbers.reserve(expected_size);

  for (const auto & element : value) {
    numbers.push_back(finite_number(element, field));
  }

  return numbers;
}

int robot_number(const std::string & robot)
{
  if (robot == "hero") {
    return 1;
  }

  if (robot == "sentry") {
    return 0;
  }

  if (robot == "infantry_3") {
    return 3;
  }

  if (robot == "infantry_4") {
    return 4;
  }

  if (robot == "infantry_5") {
    return 5;
  }

  throw std::invalid_argument("Unsupported appearance.robot: " + robot);
}

void initialize_lightbar(
  auto_aim::Lightbar & lightbar,
  const cv::Point2f & top,
  const cv::Point2f & bottom,
  auto_aim::Color color,
  std::size_t index)
{
  lightbar.id = index;
  lightbar.color = color;
  lightbar.top = top;
  lightbar.bottom = bottom;
  lightbar.center = (top + bottom) * 0.5F;
  lightbar.top2bottom = bottom - top;
  lightbar.points = {top, bottom};
  lightbar.length = cv::norm(bottom - top);
  lightbar.width = 0.0;
  lightbar.ratio = 0.0;
  lightbar.angle = std::atan2(bottom.y - top.y, bottom.x - top.x);
  lightbar.angle_error = std::abs(lightbar.angle - CV_PI / 2.0);
  lightbar.rotated_rect = cv::RotatedRect();
}

auto_aim::Armor make_armor(
  const Json & candidate,
  int color_index,
  int number,
  int image_width,
  int image_height)
{
  const auto & corners = candidate.at("corners");

  if (!corners.is_array() || corners.size() != 4) {
    throw std::invalid_argument("camera.armors[].corners must have four TL/TR/BR/BL points");
  }

  std::vector<cv::Point2f> image_points;
  image_points.reserve(4);

  for (const auto & corner : corners) {
    const auto coordinates = numeric_array(corner, 2, "camera.armors[].corners[]");
    const double pixel_x = coordinates[0];
    const double pixel_y = coordinates[1];

    if (pixel_x < 0.0 || pixel_x >= image_width || pixel_y < 0.0 || pixel_y >= image_height) {
      throw std::invalid_argument("Armor corner is outside the camera image");
    }

    image_points.emplace_back(
      static_cast<float>(pixel_x),
      static_cast<float>(pixel_y));
  }

  if (!cv::isContourConvex(image_points) || cv::contourArea(image_points, true) <= 1e-6) {
    throw std::invalid_argument("Armor corners must form a nondegenerate TL/TR/BR/BL polygon");
  }

  auto_aim::Armor armor(
    color_index,
    number,
    1.0F,
    cv::boundingRect(image_points),
    image_points);

  // Physical IDs are retained only for same-snapshot diagnostics. Tracker
  // determines its own armor-slot association from the solved measurements.
  armor.source_id = candidate.at("id").get<std::uint64_t>();
  armor.class_id = -1;
  armor.priority = auto_aim::ArmorPriority::fifth;
  armor.duplicated = false;
  armor.center_norm = cv::Point2f();
  armor.traditional_points.clear();

  initialize_lightbar(armor.left, image_points[0], image_points[3], armor.color, 0);
  initialize_lightbar(armor.right, image_points[1], image_points[2], armor.color, 1);

  armor.side_ratio =
    std::max(armor.left.length, armor.right.length) /
    std::min(armor.left.length, armor.right.length);

  // Constructors used by neural detectors leave pose fields uninitialized.
  // They become usable only after the real Solver has accepted the PnP result.
  const double unavailable = std::numeric_limits<double>::quiet_NaN();
  armor.xyz_in_gimbal.setConstant(unavailable);
  armor.xyz_in_world.setConstant(unavailable);
  armor.ypr_in_gimbal.setConstant(unavailable);
  armor.ypr_in_world.setConstant(unavailable);
  armor.ypd_in_world.setConstant(unavailable);
  armor.yaw_raw = unavailable;

  return armor;
}

Json vector_json(const Eigen::Vector3d & vector)
{
  return Json::array({vector.x(), vector.y(), vector.z()});
}

Json points_json(const std::vector<cv::Point2f> & points)
{
  Json result = Json::array();

  for (const auto & point : points) {
    result.push_back(Json::array({point.x, point.y}));
  }

  return result;
}

Json describe_armor(const auto_aim::Armor & armor, const Json & state)
{
  Json diagnostic = {
    {"source_id", armor.source_id},
    {"color", auto_aim::COLORS.at(armor.color)},
    {"name", auto_aim::ARMOR_NAMES.at(armor.name)},
    {"type", auto_aim::ARMOR_TYPES.at(armor.type)},
    {"confidence", armor.confidence},
    {"corners", points_json(armor.points)},
    {"traditional_corners", points_json(armor.traditional_points)},
    {"position_world", vector_json(armor.xyz_in_world)},
    {"yaw_world", armor.ypr_in_world[0]},
    {"yaw_raw", armor.yaw_raw}
  };

  // Truth is read only after all algorithm processing, exclusively for scoring.
  if (!state.contains("truth") || !state.at("truth").contains("armors")) {
    return diagnostic;
  }

  const auto origin = numeric_array(state.at("gimbal").at("origin"), 3, "gimbal.origin");
  const Eigen::Vector3d gimbal_origin(origin.data());

  for (const auto & truth_armor : state.at("truth").at("armors")) {
    if (truth_armor.at("id").get<std::uint64_t>() != armor.source_id) {
      continue;
    }

    const auto center = numeric_array(truth_armor.at("center"), 3, "truth.armors[].center");
    const Eigen::Vector3d expected_position = Eigen::Vector3d(center.data()) - gimbal_origin;
    const double expected_yaw = finite_number(
      truth_armor.at("normal_yaw"),
      "truth.armors[].normal_yaw") + CV_PI;

    diagnostic["position_error_m"] = (armor.xyz_in_world - expected_position).norm();
    diagnostic["yaw_error_rad"] = std::remainder(
      armor.ypr_in_world[0] - expected_yaw,
      2.0 * CV_PI);

    break;
  }

  return diagnostic;
}

Json describe_detection(const auto_aim::Armor & armor)
{
  return {
    {"source_id", armor.source_id},
    {"color", auto_aim::COLORS.at(armor.color)},
    {"name", auto_aim::ARMOR_NAMES.at(armor.name)},
    {"type", auto_aim::ARMOR_TYPES.at(armor.type)},
    {"confidence", armor.confidence},
    {"corners", points_json(armor.points)},
    {"traditional_corners", points_json(armor.traditional_points)}
  };
}

Json describe_tracker_measurement(
  const auto_aim::TrackerMeasurementObservation & measurement)
{
  Json diagnostic = {
    {"valid", measurement.valid},
    {"source_id", measurement.source_id},
    {"pnp_position_world", vector_json(measurement.pnp_position_world)},
    {"pnp_yaw_world", measurement.pnp_yaw_world},
    {"used_position_world", vector_json(measurement.used_position_world)},
    {"used_yaw_world", measurement.used_yaw_world}
  };

  if (measurement.innovation_ypda) {
    diagnostic["innovation_ypda"] = Json::array({
      (*measurement.innovation_ypda)[0],
      (*measurement.innovation_ypda)[1],
      (*measurement.innovation_ypda)[2],
      (*measurement.innovation_ypda)[3]
    });
  } else {
    diagnostic["innovation_ypda"] = nullptr;
  }

  return diagnostic;
}

bool finite_plan(const auto_aim::Plan & plan)
{
  const std::array<double, 6> command_values = {
    plan.yaw,
    plan.pitch,
    plan.yaw_vel,
    plan.pitch_vel,
    plan.yaw_acc,
    plan.pitch_acc
  };

  return std::all_of(command_values.begin(), command_values.end(), [](double number) {
    return std::isfinite(number) && std::abs(number) <= 1000.0;
  });
}

Json plan_diagnostic(const auto_aim::Plan & plan)
{
  return {
    {"control", plan.control},
    {"planner_fire_intent", plan.fire},
    {"target_yaw", plan.target_yaw},
    {"target_yaw_vel", plan.target_yaw_vel},
    {"target_pitch", plan.target_pitch},
    {"yaw", plan.yaw},
    {"pitch", plan.pitch},
    {"yaw_vel", plan.yaw_vel},
    {"pitch_vel", plan.pitch_vel},
    {"yaw_acc", plan.yaw_acc},
    {"pitch_acc", plan.pitch_acc},
    {"target_armor_index", plan.target_armor_index}
  };
}

std::filesystem::path resolve_model_path(
  const std::filesystem::path & config_directory,
  const YAML::Node & configured_value,
  const std::string & config_key)
{
  if (!configured_value || !configured_value.IsScalar()) {
    throw std::invalid_argument(
      "Rendered-frame input requires scalar " + config_key + " in the YAML config");
  }

  const std::filesystem::path configured_path = configured_value.as<std::string>();
  std::vector<std::filesystem::path> candidates;

  if (configured_path.is_absolute()) {
    candidates.push_back(configured_path);
  } else {
    candidates.push_back(config_directory / configured_path);
    candidates.push_back(std::filesystem::path(BRIDGE_AUTOAIM_ROOT) / configured_path);
  }

  for (const auto & candidate : candidates) {
    std::error_code filesystem_error;

    if (std::filesystem::is_regular_file(candidate, filesystem_error)) {
      return std::filesystem::canonical(candidate);
    }
  }

  throw std::invalid_argument(
    "Cannot resolve " + config_key + " model path: " + configured_path.string());
}

YAML::Node make_detector_config(
  const YAML::Node & base_config,
  const std::filesystem::path & source_config_path)
{
  YAML::Node detector_config = YAML::Clone(base_config);
  const std::string yolo_name = detector_config["yolo_name"].as<std::string>();

  if (yolo_name != "yolov5" && yolo_name != "yolov8" && yolo_name != "yolo11") {
    throw std::invalid_argument(
      "yolo_name must be one of yolov5, yolov8 or yolo11");
  }

  const auto config_directory = std::filesystem::absolute(source_config_path).parent_path();
  const std::array<std::string, 2> model_keys = {
    "classify_model",
    yolo_name + "_model_path"
  };

  for (const auto & model_key : model_keys) {
    detector_config[model_key] = resolve_model_path(
      config_directory,
      detector_config[model_key],
      model_key).string();
  }

  return detector_config;
}

void write_yaml(const std::filesystem::path & path, const YAML::Node & value)
{
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << value << '\n';
}

}  // namespace

const char * input_mode_name(InputMode input_mode)
{
  switch (input_mode) {
    case InputMode::exact_corners:
      return "exact-corners";

    case InputMode::raw_frame:
      return "raw-frame";
  }

  throw std::invalid_argument("Unknown bridge input mode");
}

const char * estimator_mode_name(EstimatorMode estimator_mode)
{
  switch (estimator_mode) {
    case EstimatorMode::pnp:
      return "pnp";

    case EstimatorMode::uv_ekf:
      return "uv-ekf";
  }

  throw std::invalid_argument("Unknown bridge estimator mode");
}

namespace
{

Observation adapt_packet(
  const Json & state,
  InputMode input_mode,
  cv::Mat bgr_image,
  SteadyClock::time_point received_at,
  bool require_raw_image)
{
  Observation observation;
  observation.received_at = received_at;
  observation.state = state;
  observation.input_mode = input_mode;
  observation.bgr_image = std::move(bgr_image);

  const auto & camera = state.at("camera");
  const auto & gimbal = state.at("gimbal");
  const auto & appearance = state.at("appearance");
  const auto & weapon = state.at("weapon");

  const double camera_width = finite_number(camera.at("width"), "camera.width");
  const double camera_height = finite_number(camera.at("height"), "camera.height");

  if (camera_width != 1440.0 || camera_height != 1080.0) {
    throw std::invalid_argument(
      "Use a 1440x1080 simulator camera: the current Tracker assumes center (720, 540)");
  }

  const auto intrinsics = numeric_array(camera.at("K"), 9, "camera.K");
  numeric_array(camera.at("distortion"), 5, "camera.distortion");
  const auto rotation = numeric_array(camera.at("R_camera2gimbal"), 9, "camera.R_camera2gimbal");
  numeric_array(camera.at("t_camera2gimbal"), 3, "camera.t_camera2gimbal");
  numeric_array(gimbal.at("origin"), 3, "gimbal.origin");

  if (intrinsics[0] <= 0.0 || intrinsics[4] <= 0.0 ||
      std::abs(intrinsics[6]) > 1e-9 || std::abs(intrinsics[7]) > 1e-9 ||
      std::abs(intrinsics[8] - 1.0) > 1e-9) {
    throw std::invalid_argument("camera.K must be a valid pinhole intrinsic matrix");
  }

  const Eigen::Matrix<double, 3, 3, Eigen::RowMajor> camera_to_gimbal(rotation.data());
  const double orthogonality_error = (
    camera_to_gimbal.transpose() * camera_to_gimbal - Eigen::Matrix3d::Identity()).norm();
  const double determinant_error = std::abs(camera_to_gimbal.determinant() - 1.0);

  if (orthogonality_error > 1e-6 || determinant_error > 1e-6) {
    throw std::invalid_argument("camera.R_camera2gimbal must be a proper rotation");
  }

  const auto quaternion = numeric_array(gimbal.at("quaternion_wxyz"), 4, "gimbal.quaternion_wxyz");
  observation.gimbal_to_world = Eigen::Quaterniond(
    quaternion[0],
    quaternion[1],
    quaternion[2],
    quaternion[3]);

  if (observation.gimbal_to_world.norm() < 1e-12) {
    throw std::invalid_argument("gimbal.quaternion_wxyz must not be zero");
  }

  observation.gimbal_to_world.normalize();
  observation.sim_time = finite_number(state.at("sim_time"), "sim_time");
  const double camera_time = finite_number(camera.at("timestamp"), "camera.timestamp");

  if (observation.sim_time < 0.0 || observation.sim_time > 1e9 ||
      std::abs(camera_time - observation.sim_time) > 1e-6 ||
      camera.at("tick") != state.at("tick")) {
    throw std::invalid_argument(
      "Camera and gimbal must belong to the same valid simulation snapshot");
  }

  observation.bullet_speed = finite_number(weapon.at("speed"), "weapon.speed");
  observation.firing_delay = finite_number(weapon.at("delay"), "weapon.delay");

  if (observation.bullet_speed < 10.0 || observation.bullet_speed > 25.0) {
    throw std::invalid_argument(
      "weapon.speed must be 10..25 m/s; upstream Planner silently substitutes "
      "22 outside this range");
  }

  if (observation.firing_delay < 0.0 || observation.firing_delay > 1.0) {
    throw std::invalid_argument("weapon.delay must be in [0, 1] seconds");
  }

  const std::string team = appearance.at("team").get<std::string>();

  if (team != "red" && team != "blue") {
    throw std::invalid_argument("appearance.team must be red or blue");
  }

  if (input_mode == InputMode::exact_corners) {
    const int number = robot_number(appearance.at("robot").get<std::string>());
    const int color_index = team == "blue" ? 0 : 1;
    const auto & candidates = camera.at("armors");

    if (!candidates.is_array() || candidates.size() > 4) {
      throw std::invalid_argument("camera.armors must be an array of at most four armors");
    }

    std::set<int> physical_ids;

    for (const auto & candidate : candidates) {
      const auto & identity = candidate.at("id");

      if (!identity.is_number_integer() || identity.get<double>() < 0.0 ||
          identity.get<double>() > 3.0) {
        throw std::invalid_argument("camera.armors[].id must be an integer in 0..3");
      }

      if (!physical_ids.insert(identity.get<int>()).second) {
        throw std::invalid_argument("camera.armors contains a duplicate physical ID");
      }

      observation.armors.push_back(make_armor(candidate, color_index, number, 1440, 1080));
    }
  } else if (require_raw_image) {
    if (observation.bgr_image.empty() || observation.bgr_image.type() != CV_8UC3) {
      throw std::invalid_argument("Rendered frame must be a nonempty CV_8UC3 BGR image");
    }

    if (observation.bgr_image.cols != static_cast<int>(camera_width) ||
        observation.bgr_image.rows != static_cast<int>(camera_height)) {
      throw std::invalid_argument("Rendered frame dimensions do not match frame.camera");
    }
  }

  observation.active = !state.at("paused").get<bool>() && state.at("ipc").at("enabled").get<bool>();

  observation.context = Json({
    {"input", input_mode_name(input_mode)},
    {"publisher", state.at("publisher_id")},
    {"session", state.at("session_id")},
    {"paused", state.at("paused")},
    {"enabled", state.at("ipc").at("enabled")},
    {"appearance", appearance},
    {"width", camera.at("width")},
    {"height", camera.at("height")},
    {"brightness_gain", camera.at("brightness_gain")},
    {"K", camera.at("K")},
    {"distortion", camera.at("distortion")},
    {"R", camera.at("R_camera2gimbal")},
    {"t", camera.at("t_camera2gimbal")},
    {"gimbal_origin", gimbal.at("origin")},
    {"weapon", weapon}
  }).dump();

  return observation;
}

}  // namespace

Observation adapt_state(const Json & state)
{
  if (state.at("protocol") != "rm_aim_sim/2" || state.at("type") != "state") {
    throw std::invalid_argument("Exact-corner input requires an rm_aim_sim/2 state packet");
  }

  return adapt_packet(
    state,
    InputMode::exact_corners,
    cv::Mat(),
    SteadyClock::now(),
    false);
}

Observation inspect_state(const Json & state, InputMode input_mode)
{
  if (state.at("protocol") != "rm_aim_sim/2" || state.at("type") != "state") {
    throw std::invalid_argument("State inspection requires an rm_aim_sim/2 state packet");
  }

  return adapt_packet(
    state,
    input_mode,
    cv::Mat(),
    SteadyClock::now(),
    false);
}

Observation adapt_frame(
  const Json & frame_metadata,
  cv::Mat bgr_image,
  SteadyClock::time_point received_at)
{
  if (frame_metadata.at("protocol") != "rm_aim_sim/2" ||
      frame_metadata.at("type") != "frame") {
    throw std::invalid_argument("Rendered input requires an rm_aim_sim/2 frame packet");
  }

  return adapt_packet(
    frame_metadata,
    InputMode::raw_frame,
    std::move(bgr_image),
    received_at,
    true);
}

struct AutoAimPipeline::Implementation
{
  YAML::Node base_config;
  std::filesystem::path output_directory;
  InputMode input_mode = InputMode::exact_corners;
  EstimatorMode estimator_mode = EstimatorMode::pnp;
  std::unique_ptr<auto_aim::YOLO> detector;
  std::uint64_t detector_frame_count = 0;
  std::unique_ptr<auto_aim::Solver> solver;
  std::unique_ptr<auto_aim::Tracker> tracker;
  std::unique_ptr<UvControlAdapter> uv_control_adapter;
  std::unique_ptr<auto_aim::Planner> planner;
  std::optional<std::uint64_t> generation;
  std::string context;
  SteadyClock::time_point clock_origin;
  double first_sim_time = 0.0;
  std::optional<double> previous_sim_time;
  bool was_confirmed_tracking = false;
  std::optional<auto_aim::Target> target_for_planning;
  double latest_measurement_age_seconds = std::numeric_limits<double>::infinity();
  double latest_bullet_speed = 0.0;
  std::optional<double> planner_firing_delay;
  PipelineResult latest_result;
  bool has_result = false;

  SteadyClock::time_point algorithm_time(double sim_time) const
  {
    const auto simulation_elapsed = std::chrono::duration<double>(
      sim_time - first_sim_time);

    return clock_origin +
      std::chrono::duration_cast<SteadyClock::duration>(simulation_elapsed);
  }

  void start_generation(const Observation & observation, std::uint64_t next_generation)
  {
    if (planner_firing_delay &&
        std::abs(*planner_firing_delay - observation.firing_delay) > 1e-9) {
      throw std::invalid_argument(
        "weapon.delay changed while Planner is active; restart the bridge to apply it");
    }

    const auto & camera = observation.state.at("camera");
    YAML::Node effective_config = YAML::Clone(base_config);

    effective_config["camera_matrix"] = camera.at("K").get<std::vector<double>>();
    effective_config["distort_coeffs"] = camera.at("distortion").get<std::vector<double>>();
    effective_config["R_camera2gimbal"] = camera.at("R_camera2gimbal").get<std::vector<double>>();
    effective_config["t_camera2gimbal"] = camera.at("t_camera2gimbal").get<std::vector<double>>();
    effective_config["enemy_color"] =
      observation.state.at("appearance").at("team").get<std::string>();

    effective_config["R_gimbal2imubody"] = std::vector<double>{
      1.0, 0.0, 0.0,
      0.0, 1.0, 0.0,
      0.0, 0.0, 1.0
    };

    effective_config["gimbal_yaw_zero"] = 0.0;
    effective_config["gimbal_pitch_sign"] = -1.0;
    effective_config["high_speed_delay_time"] = observation.firing_delay;
    effective_config["low_speed_delay_time"] = observation.firing_delay;

    const auto effective_path = output_directory /
      ("effective_config_" + std::to_string(next_generation) + ".yaml");

    write_yaml(effective_path, effective_config);

    tracker.reset();
    uv_control_adapter.reset();
    solver = std::make_unique<auto_aim::Solver>(effective_path.string());
    tracker = std::make_unique<auto_aim::Tracker>(effective_path.string(), *solver);

    if (estimator_mode == EstimatorMode::uv_ekf) {
      uv_control_adapter = std::make_unique<UvControlAdapter>(effective_path);

      tracker->set_debug_measurement_override([this](auto_aim::Armor & armor) {
        return uv_control_adapter->override_measurement(armor);
      });
    }

    clock_origin = SteadyClock::now();
    first_sim_time = observation.sim_time;
    previous_sim_time.reset();
    was_confirmed_tracking = false;
    target_for_planning.reset();
    latest_measurement_age_seconds = std::numeric_limits<double>::infinity();
    latest_bullet_speed = observation.bullet_speed;
    latest_result = {};
    has_result = false;

    if (!planner) {
      planner = std::make_unique<auto_aim::Planner>(effective_path.string());
      planner_firing_delay = observation.firing_delay;
    }

    // Calibration does not enter Planner. Its settings, including the verified
    // fixed firing delay, stay fixed for this process.
    // This public API clears target selection; TinyMPC retains its warm start.
    // Repeatedly constructing upstream Planner would leak its raw solver handles.
    planner->plan(std::nullopt, observation.bullet_speed, clock_origin);

    generation = next_generation;
    context = observation.context;
  }
};

AutoAimPipeline::AutoAimPipeline(
  const std::filesystem::path & config_path,
  const std::filesystem::path & output_directory,
  InputMode input_mode,
  EstimatorMode estimator_mode)
: implementation_(std::make_unique<Implementation>())
{
  implementation_->base_config = YAML::LoadFile(config_path.string());
  implementation_->output_directory = output_directory;
  implementation_->input_mode = input_mode;
  implementation_->estimator_mode = estimator_mode;

  if (!std::filesystem::is_directory(output_directory)) {
    throw std::invalid_argument("Pipeline output directory does not exist");
  }

  // Upstream MPC constructs Eigen vectors from raw pointers. Check the shapes
  // here so a mistyped user configuration cannot cause an out-of-bounds read.
  for (const std::string axis : {"yaw", "pitch"}) {
    const auto state_cost = implementation_->base_config["Q_" + axis].as<std::vector<double>>();
    const auto input_cost = implementation_->base_config["R_" + axis].as<std::vector<double>>();

    if (state_cost.size() != 2 || input_cost.size() != 1 ||
        !std::isfinite(state_cost[0]) || !std::isfinite(state_cost[1]) ||
        !std::isfinite(input_cost[0]) || state_cost[0] <= 0.0 ||
        state_cost[1] < 0.0 || input_cost[0] <= 0.0) {
      throw std::invalid_argument(
        "Q_" + axis + " must be [positive, nonnegative]; R must be [positive]");
    }
  }

  if (input_mode == InputMode::raw_frame) {
    const YAML::Node detector_config = make_detector_config(
      implementation_->base_config,
      config_path);

    const auto detector_config_path = output_directory / "detector_config.yaml";
    write_yaml(detector_config_path, detector_config);

    implementation_->detector = std::make_unique<auto_aim::YOLO>(
      detector_config_path.string(),
      false);
  }
}

AutoAimPipeline::~AutoAimPipeline() = default;

PipelineResult AutoAimPipeline::process(
  const Observation & observation,
  std::uint64_t generation)
{
  auto & pipeline = *implementation_;

  if (observation.input_mode != pipeline.input_mode) {
    throw std::invalid_argument("Observation input mode does not match this pipeline");
  }

  if (!pipeline.generation || *pipeline.generation != generation) {
    pipeline.start_generation(observation, generation);
  } else if (pipeline.context != observation.context) {
    throw std::invalid_argument("Changed observation context requires a new generation");
  }

  if (pipeline.previous_sim_time && observation.sim_time <= *pipeline.previous_sim_time) {
    throw std::invalid_argument("Simulation timestamps must increase within a generation");
  }

  pipeline.previous_sim_time = observation.sim_time;
  const auto algorithm_time = pipeline.algorithm_time(observation.sim_time);

  const auto detector_started_at = SteadyClock::now();
  std::list<auto_aim::Armor> input_armors = observation.armors;

  if (pipeline.input_mode == InputMode::raw_frame) {
    if (!pipeline.detector) {
      throw std::logic_error("Rendered-frame pipeline has no detector");
    }

    const auto maximum_frame_count = static_cast<std::uint64_t>(
      std::numeric_limits<int>::max());

    const int detector_frame_count = static_cast<int>(
      pipeline.detector_frame_count % maximum_frame_count);

    ++pipeline.detector_frame_count;
    input_armors = pipeline.detector->detect(
      observation.bgr_image,
      detector_frame_count);

    std::uint64_t detection_id = 0;

    for (auto & armor : input_armors) {
      // This is only a frame-local detector identity. It is never matched to
      // frame.camera.armors or treated as a persistent physical face ID.
      armor.source_id = detection_id++;
      armor.priority = auto_aim::ArmorPriority::fifth;
    }
  }

  const double detector_milliseconds = pipeline.input_mode == InputMode::raw_frame
    ? std::chrono::duration<double, std::milli>(
        SteadyClock::now() - detector_started_at).count()
    : 0.0;

  Json detection_diagnostics = Json::array();

  for (const auto & armor : input_armors) {
    detection_diagnostics.push_back(describe_detection(armor));
  }

  const std::size_t input_armor_count = input_armors.size();
  std::list<auto_aim::Armor> solved_armors = std::move(input_armors);
  pipeline.solver->set_R_gimbal2world(observation.gimbal_to_world);

  if (pipeline.estimator_mode == EstimatorMode::uv_ekf) {
    if (!pipeline.uv_control_adapter) {
      throw std::logic_error("UV-EKF estimator has no control adapter");
    }

    pipeline.uv_control_adapter->prepare_frame(
      solved_armors,
      *pipeline.solver,
      algorithm_time,
      observation.gimbal_to_world.toRotationMatrix());
  }

  // Tracker performs the production IPPE/refinement path before association.
  // UV mode may replace the selected plate measurement with a pixel-EKF
  // posterior, but simulator 3D truth is never an algorithm input.
  const auto targets = pipeline.tracker->track(solved_armors, algorithm_time);
  const auto & tracking = pipeline.tracker->observation();
  const std::string tracker_state = pipeline.tracker->state();

  if (tracker_state == "tracking") {
    pipeline.was_confirmed_tracking = true;
  } else if (tracker_state == "lost") {
    pipeline.was_confirmed_tracking = false;
  }

  const bool confirmed =
    tracker_state == "tracking" ||
    (tracker_state == "temp_lost" && pipeline.was_confirmed_tracking);

  const bool measurement_recent = tracking.tracker_time_since_last_measurement_ms &&
    *tracking.tracker_time_since_last_measurement_ms <= 100.0;

  pipeline.latest_measurement_age_seconds = tracking.tracker_time_since_last_measurement_ms
    ? *tracking.tracker_time_since_last_measurement_ms / 1000.0
    : std::numeric_limits<double>::infinity();

  pipeline.target_for_planning.reset();

  if (observation.active && confirmed && measurement_recent && !targets.empty()) {
    pipeline.target_for_planning = targets.front();
  }

  PipelineResult result;
  result.generation = generation;
  result.source_sim_time = observation.sim_time;
  result.planning_sim_time = observation.sim_time;
  result.observation_received_at = observation.received_at;
  result.tracker_state = tracker_state;

  Json armor_diagnostics = Json::array();

  for (const auto & armor : solved_armors) {
    armor_diagnostics.push_back(describe_armor(armor, observation.state));
  }

  Json tracker_measurements = Json::array();

  for (const auto & measurement : tracking.measurements) {
    tracker_measurements.push_back(describe_tracker_measurement(measurement));
  }

  result.diagnostics = {
    {"generation", generation},
    {"source_sim_time", observation.sim_time},
    {"planning_sim_time", observation.sim_time},
    {"tick", observation.state.at("tick")},
    {"input", input_mode_name(pipeline.input_mode)},
    {"estimator", estimator_mode_name(pipeline.estimator_mode)},
    {"input_armors", input_armor_count},
    {"solved_armors", solved_armors.size()},
    {"detector_ms", detector_milliseconds},
    {"detections", detection_diagnostics},
    {"tracker_state", tracker_state},
    {"measurement_found", tracking.measurement_found},
    {"tracker_measurements", tracker_measurements},
    {"uv_ekf", pipeline.uv_control_adapter
      ? pipeline.uv_control_adapter->diagnostics()
      : Json(nullptr)},
    {"armors", armor_diagnostics},
    {"plan", plan_diagnostic(result.plan)}
  };

  pipeline.latest_bullet_speed = observation.bullet_speed;
  pipeline.latest_result = std::move(result);
  pipeline.has_result = true;

  return replan(observation.sim_time, generation);
}

PipelineResult AutoAimPipeline::replan(
  double planning_sim_time,
  std::uint64_t generation)
{
  auto & pipeline = *implementation_;

  if (!std::isfinite(planning_sim_time)) {
    throw std::invalid_argument("Planning simulation time must be finite");
  }

  if (!pipeline.generation || *pipeline.generation != generation || !pipeline.has_result) {
    throw std::invalid_argument("Cannot plan before processing this generation");
  }

  // A newer observation can overtake an already queued planning request.
  // Planning from that newer measurement is safe; moving time backwards is not.
  planning_sim_time = std::max(
    planning_sim_time,
    pipeline.latest_result.source_sim_time);

  const double target_age = planning_sim_time - pipeline.latest_result.source_sim_time;
  const double measurement_age = pipeline.latest_measurement_age_seconds + target_age;
  std::optional<auto_aim::Target> target;

  if (measurement_age <= 0.1 && pipeline.target_for_planning) {
    target = pipeline.target_for_planning;
  }

  PipelineResult result = pipeline.latest_result;
  result.planning_sim_time = planning_sim_time;
  result.plan = pipeline.planner->plan(
    target,
    pipeline.latest_bullet_speed,
    pipeline.algorithm_time(planning_sim_time));

  if (!finite_plan(result.plan)) {
    throw std::runtime_error("Planner produced a nonfinite or out-of-protocol command");
  }

  result.diagnostics["planning_sim_time"] = planning_sim_time;
  result.diagnostics["target_age_at_plan_ms"] = target_age * 1000.0;
  result.diagnostics["measurement_age_at_plan_ms"] = measurement_age * 1000.0;
  result.diagnostics["plan"] = plan_diagnostic(result.plan);

  pipeline.latest_result = result;
  return result;
}

}  // namespace auto_aim_bridge
