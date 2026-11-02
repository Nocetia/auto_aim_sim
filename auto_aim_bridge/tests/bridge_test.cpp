#include "sim_adapter.hpp"
#include "sim_client.hpp"

#include <Eigen/Geometry>
#include <algorithm>
#include <array>
#include <cmath>
#include <cstdlib>
#include <filesystem>
#include <fstream>
#include <iostream>
#include <limits>
#include <stdexcept>
#include <string>
#include <vector>

#include <sys/stat.h>
#include <unistd.h>

#include <opencv2/calib3d.hpp>
#include <yaml-cpp/yaml.h>

namespace
{

using auto_aim_bridge::AutoAimPipeline;
using auto_aim_bridge::Json;
using auto_aim_bridge::PipelineResult;

constexpr double kPi = 3.14159265358979323846;
constexpr double kFrameInterval = 1.0 / 60.0;
constexpr double kMeasurementFreshnessLimit = 0.1;
constexpr int kPhysicalArmorId = 2;

void require(bool condition, const std::string & explanation)
{
  if (!condition) {
    throw std::runtime_error(explanation);
  }
}

void require_near(
  double actual,
  double expected,
  double tolerance,
  const std::string & explanation)
{
  require(
    std::isfinite(actual) && std::abs(actual - expected) <= tolerance,
    explanation + ": actual=" + std::to_string(actual) +
      ", expected=" + std::to_string(expected));
}

template <typename Function>
void require_invalid_argument(Function operation, const std::string & explanation)
{
  try {
    operation();
  } catch (const std::invalid_argument &) {
    return;
  }

  throw std::runtime_error(explanation + ": expected std::invalid_argument");
}

template <typename Function>
void require_runtime_error(Function operation, const std::string & explanation)
{
  try {
    operation();
  } catch (const std::runtime_error &) {
    return;
  }

  throw std::runtime_error(explanation + ": expected std::runtime_error");
}

class TemporaryDirectory
{
public:
  TemporaryDirectory()
  {
    std::string path_template =
      (std::filesystem::temp_directory_path() / "rm_aim_bridge_test_XXXXXX").string();

    std::vector<char> writable_path(path_template.begin(), path_template.end());
    writable_path.push_back('\0');

    const char * created_path = ::mkdtemp(writable_path.data());
    if (created_path == nullptr) {
      throw std::runtime_error("Cannot create the bridge test output directory");
    }

    path_ = created_path;
  }

  ~TemporaryDirectory()
  {
    std::error_code ignored_error;
    std::filesystem::remove_all(path_, ignored_error);
  }

  TemporaryDirectory(const TemporaryDirectory &) = delete;
  TemporaryDirectory & operator=(const TemporaryDirectory &) = delete;

  std::filesystem::path child(const std::string & name) const
  {
    const std::filesystem::path result = path_ / name;
    std::filesystem::create_directory(result);
    return result;
  }

private:
  std::filesystem::path path_;
};

class TemporaryIpcDirectory
{
public:
  TemporaryIpcDirectory()
  {
    channel_ =
      "bridge_unit_" + std::to_string(::getpid()) + "_" +
      std::to_string(auto_aim_bridge::SteadyClock::now().time_since_epoch().count());

    path_ = std::filesystem::path("/dev/shm") / ("rm_aim_sim_" + channel_);

    if (!std::filesystem::create_directory(path_)) {
      throw std::runtime_error("Cannot create a unique bridge test IPC directory");
    }

    if (::chmod(path_.c_str(), 0700) != 0) {
      throw std::runtime_error("Cannot set test IPC directory permissions");
    }
  }

  ~TemporaryIpcDirectory()
  {
    std::error_code ignored_error;
    std::filesystem::remove_all(path_, ignored_error);
  }

  TemporaryIpcDirectory(const TemporaryIpcDirectory &) = delete;
  TemporaryIpcDirectory & operator=(const TemporaryIpcDirectory &) = delete;

  const std::string & channel() const
  {
    return channel_;
  }

  std::filesystem::path file(const std::string & name) const
  {
    return path_ / name;
  }

private:
  std::string channel_;
  std::filesystem::path path_;
};

void write_json_file(const std::filesystem::path & path, const Json & value)
{
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream << value.dump();
}

Json read_json_file(const std::filesystem::path & path)
{
  std::ifstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);

  Json result;
  stream >> result;
  return result;
}

void write_binary_file(
  const std::filesystem::path & path,
  const std::vector<std::uint8_t> & bytes)
{
  std::ofstream stream(path, std::ios::binary);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  stream.write(
    reinterpret_cast<const char *>(bytes.data()),
    static_cast<std::streamsize>(bytes.size()));
}

Json vector_json(const Eigen::Vector3d & vector)
{
  return Json::array({vector.x(), vector.y(), vector.z()});
}

Eigen::Vector3d vector_from_json(const Json & value)
{
  return Eigen::Vector3d(
    value.at(0).get<double>(),
    value.at(1).get<double>(),
    value.at(2).get<double>());
}

Json make_projected_state(
  const std::string & robot = "infantry_3",
  const std::string & team = "blue")
{
  const double focal_length = 1303.6753236814714;
  const cv::Matx33d camera_matrix(
    focal_length, 0.0, 720.0,
    0.0, focal_length, 540.0,
    0.0, 0.0, 1.0);

  Eigen::Matrix3d camera_to_gimbal;
  camera_to_gimbal <<
    0.0, 0.0, 1.0,
    -1.0, 0.0, 0.0,
    0.0, -1.0, 0.0;

  const Eigen::Vector3d camera_offset(0.08, 0.0, 0.04);
  const Eigen::Vector3d gimbal_origin(1.2, -0.5, 0.65);
  const double gimbal_yaw = 0.12;
  const double gimbal_pitch = -0.08;

  const Eigen::Quaterniond gimbal_orientation(
    Eigen::AngleAxisd(gimbal_yaw, Eigen::Vector3d::UnitZ()) *
    Eigen::AngleAxisd(gimbal_pitch, Eigen::Vector3d::UnitY()));

  const Eigen::Vector3d armor_position(4.0, 0.5, -0.35);
  const double armor_inward_yaw = 0.35;
  const Eigen::Matrix3d armor_to_world =
    (Eigen::AngleAxisd(armor_inward_yaw, Eigen::Vector3d::UnitZ()) *
     Eigen::AngleAxisd(15.0 * kPi / 180.0, Eigen::Vector3d::UnitY()))
      .toRotationMatrix();

  const Eigen::Matrix3d world_to_camera =
    camera_to_gimbal.transpose() * gimbal_orientation.toRotationMatrix().transpose();

  const Eigen::Matrix3d armor_to_camera = world_to_camera * armor_to_world;
  const Eigen::Vector3d position_in_camera = camera_to_gimbal.transpose() *
    (gimbal_orientation.toRotationMatrix().transpose() * armor_position - camera_offset);

  cv::Matx33d rotation_matrix;
  for (int row = 0; row < 3; ++row) {
    for (int column = 0; column < 3; ++column) {
      rotation_matrix(row, column) = armor_to_camera(row, column);
    }
  }

  cv::Vec3d rotation_vector;
  cv::Rodrigues(rotation_matrix, rotation_vector);

  const double armor_width = robot == "hero" ? 0.230 : 0.135;
  const double armor_height = 0.056;
  const std::vector<cv::Point3d> object_points{
    {0.0, armor_width / 2.0, armor_height / 2.0},
    {0.0, -armor_width / 2.0, armor_height / 2.0},
    {0.0, -armor_width / 2.0, -armor_height / 2.0},
    {0.0, armor_width / 2.0, -armor_height / 2.0}
  };

  std::vector<cv::Point2d> image_points;
  cv::projectPoints(
    object_points,
    rotation_vector,
    cv::Vec3d(position_in_camera.x(), position_in_camera.y(), position_in_camera.z()),
    camera_matrix,
    cv::Vec<double, 5>::all(0.0),
    image_points);

  Json corners = Json::array();
  for (const cv::Point2d & point : image_points) {
    require(
      point.x > 0.0 && point.x < 1440.0 && point.y > 0.0 && point.y < 1080.0,
      "Synthetic fixture corners must lie within the image");
    corners.push_back(Json::array({point.x, point.y}));
  }

  const Eigen::Vector3d outward_normal = -armor_to_world.col(0);
  const double wall_unix_seconds = std::chrono::duration<double>(
    std::chrono::system_clock::now().time_since_epoch()).count();

  return Json{
    {"protocol", "rm_aim_sim/2"},
    {"type", "state"},
    {"publisher_id", "offline-publisher"},
    {"session_id", "offline-session"},
    {"publication", 1},
    {"wall_unix_sec", wall_unix_seconds},
    {"tick", 2400},
    {"sim_time", 10.0},
    {"paused", false},
    {"gimbal", {
      {"yaw", gimbal_yaw},
      {"pitch", gimbal_pitch},
      {"yaw_vel", 0.0},
      {"pitch_vel", 0.0},
      {"origin", vector_json(gimbal_origin)},
      {"quaternion_wxyz", Json::array({
        gimbal_orientation.w(),
        gimbal_orientation.x(),
        gimbal_orientation.y(),
        gimbal_orientation.z()
      })}
    }},
    {"weapon", {
      {"speed", 23.0},
      {"rate", 10.0},
      {"delay", 0.015},
      {"barrel_length", 0.22},
      {"gravity", 9.7833},
      {"damage", 20}
    }},
    {"health", {
      {"current", 350},
      {"maximum", 350},
      {"alive", true},
      {"death_time", -1.0}
    }},
    {"appearance", {
      {"robot", robot},
      {"team", team},
      {"armor_model", ""},
      {"body_model", ""},
      {"armor_scale", 1.0},
      {"body_scale", 1.0}
    }},
    {"camera", {
      {"timestamp", 10.0},
      {"tick", 2400},
      {"width", 1440},
      {"height", 1080},
      {"brightness_gain", 0.4},
      {"K", Json::array({
        focal_length, 0.0, 720.0,
        0.0, focal_length, 540.0,
        0.0, 0.0, 1.0
      })},
      {"distortion", Json::array({0.0, 0.0, 0.0, 0.0, 0.0})},
      {"R_camera2gimbal", Json::array({
        0.0, 0.0, 1.0,
        -1.0, 0.0, 0.0,
        0.0, -1.0, 0.0
      })},
      {"t_camera2gimbal", vector_json(camera_offset)},
      {"origin", vector_json(
        gimbal_origin + gimbal_orientation.toRotationMatrix() * camera_offset)},
      {"yaw", gimbal_yaw},
      {"pitch", gimbal_pitch},
      {"armors", Json::array({{
        {"id", kPhysicalArmorId},
        {"corners", corners}
      }})}
    }},
    {"truth", {
      {"center", vector_json(gimbal_origin + armor_position)},
      {"angle", armor_inward_yaw - kPi},
      {"armors", Json::array({{
        {"id", kPhysicalArmorId},
        {"center", vector_json(gimbal_origin + armor_position)},
        {"normal", vector_json(outward_normal)},
        {"normal_yaw", armor_inward_yaw - kPi}
      }})}
    }},
    {"stats", Json::object()},
    {"events", Json::array()},
    {"ipc", {
      {"enabled", true},
      {"last_seq", -1},
      {"received", 0},
      {"rejected", 0},
      {"status", "offline"}
    }}
  };
}

void set_frame_time(Json & state, int frame_index, double start_time = 10.0)
{
  const double sim_time = start_time + frame_index * kFrameInterval;
  const std::uint64_t tick = static_cast<std::uint64_t>(std::llround(sim_time * 240.0));

  state["publication"] = frame_index + 1;
  state["sim_time"] = sim_time;
  state["tick"] = tick;
  state["camera"]["timestamp"] = sim_time;
  state["camera"]["tick"] = tick;
}

const Json & single_solved_armor(const PipelineResult & result)
{
  const Json & armors = result.diagnostics.at("armors");
  require(armors.is_array() && armors.size() == 1, "Expected exactly one solved armor");
  require(
    armors.at(0).at("source_id").get<int>() == kPhysicalArmorId,
    "PnP diagnostics must preserve the frame-local source identity");
  return armors.at(0);
}

void check_finite_plan(const auto_aim::Plan & plan)
{
  const std::array<double, 9> values{
    plan.target_yaw,
    plan.target_yaw_vel,
    plan.target_pitch,
    plan.yaw,
    plan.pitch,
    plan.yaw_vel,
    plan.pitch_vel,
    plan.yaw_acc,
    plan.pitch_acc
  };

  for (double value : values) {
    require(std::isfinite(value), "Planner outputs must be finite");
  }
}

void check_same_plan(const auto_aim::Plan & first, const auto_aim::Plan & second)
{
  require(first.control == second.control, "Truth must not change control intent");
  require(first.fire == second.fire, "Truth must not change planner fire intent");
  require(
    first.target_armor_index == second.target_armor_index,
    "Truth must not change planner armor selection");

  const std::array<double, 9> first_values{
    first.target_yaw, first.target_yaw_vel, first.target_pitch,
    first.yaw, first.pitch, first.yaw_vel, first.pitch_vel, first.yaw_acc, first.pitch_acc
  };
  const std::array<double, 9> second_values{
    second.target_yaw, second.target_yaw_vel, second.target_pitch,
    second.yaw, second.pitch, second.yaw_vel, second.pitch_vel, second.yaw_acc, second.pitch_acc
  };

  for (std::size_t index = 0; index < first_values.size(); ++index) {
    require_near(
      first_values[index], second_values[index], 1e-7,
      "Truth must not change planner output component " + std::to_string(index));
  }
}

void test_corner_and_label_mapping()
{
  const Json state = make_projected_state();
  const auto observation = auto_aim_bridge::adapt_state(state);

  require(observation.active, "A fresh enabled snapshot must be active");
  require(observation.armors.size() == 1, "Adapter must preserve visible armor count");

  const auto_aim::Armor & armor = observation.armors.front();
  require(armor.name == auto_aim::three, "infantry_3 must map to armor number three");
  require(armor.type == auto_aim::small, "infantry_3 must use the small PnP model");
  require(armor.color == auto_aim::blue, "Blue appearance must map to blue");
  require(armor.source_id == kPhysicalArmorId, "Face identity must remain diagnostic identity");
  require(armor.points.size() == 4, "Adapter must supply four image landmarks");

  const Json & source_corners = state.at("camera").at("armors").at(0).at("corners");
  for (std::size_t index = 0; index < armor.points.size(); ++index) {
    require_near(
      armor.points[index].x, source_corners.at(index).at(0).get<double>(), 1e-4,
      "Adapter must preserve TL/TR/BR/BL horizontal coordinates");
    require_near(
      armor.points[index].y, source_corners.at(index).at(1).get<double>(), 1e-4,
      "Adapter must preserve TL/TR/BR/BL vertical coordinates");
  }

  const auto hero = auto_aim_bridge::adapt_state(make_projected_state("hero", "red"));
  require(hero.armors.front().name == auto_aim::one, "Hero must map to armor number one");
  require(hero.armors.front().type == auto_aim::big, "Hero must use the large PnP model");
  require(hero.armors.front().color == auto_aim::red, "Red appearance must map to red");

  const auto sentry = auto_aim_bridge::adapt_state(make_projected_state("sentry"));
  require(sentry.armors.front().name == auto_aim::sentry, "Sentry must use its semantic label");
  require(sentry.armors.front().type == auto_aim::small, "Sentry must use the small PnP model");

  Json godot_numeric_state = state;
  godot_numeric_state["camera"]["width"] = 1440.0;
  godot_numeric_state["camera"]["height"] = 1080.0;
  require(
    auto_aim_bridge::adapt_state(godot_numeric_state).armors.size() == 1,
    "Godot floating-point serialization of integral camera dimensions must be accepted");

  Json inactive_state = state;
  inactive_state["paused"] = true;
  require(!auto_aim_bridge::adapt_state(inactive_state).active, "Paused state must be inactive");

  inactive_state = state;
  inactive_state["ipc"]["enabled"] = false;
  require(
    !auto_aim_bridge::adapt_state(inactive_state).active,
    "Disabled external control must be inactive");
}

void test_invalid_observations()
{
  const Json valid_state = make_projected_state();

  Json invalid_state = valid_state;
  invalid_state["camera"]["width"] = 1280;
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "Unsupported tracker image dimensions must be rejected");

  invalid_state = valid_state;
  invalid_state["weapon"]["delay"] = -0.1;
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "A negative muzzle release delay must be rejected");

  invalid_state = valid_state;
  invalid_state["gimbal"]["quaternion_wxyz"] = Json::array({0.0, 0.0, 0.0, 0.0});
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "A zero attitude quaternion must be rejected");

  invalid_state = valid_state;
  invalid_state["camera"]["armors"][0]["corners"].erase(3);
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "An armor with three corners must be rejected");

  invalid_state = valid_state;
  invalid_state["camera"]["K"] = Json::array({1.0, 2.0});
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "Malformed calibration array length must be rejected");

  invalid_state = valid_state;
  invalid_state["camera"]["armors"][0]["corners"][0][0] =
    std::numeric_limits<double>::infinity();
  require_invalid_argument(
    [&] { auto_aim_bridge::adapt_state(invalid_state); },
    "Nonfinite image coordinates must be rejected");
}

void test_raw_frame_input_isolation()
{
  Json frame_metadata = make_projected_state();
  frame_metadata["type"] = "frame";
  frame_metadata["request_id"] = "offline-frame-1";
  frame_metadata["frame_file"] = "frame-offline-frame-1.raw";
  frame_metadata["encoding"] = "rgb8";
  frame_metadata["image_bytes"] = 1440 * 1080 * 3;
  frame_metadata.erase("publication");
  frame_metadata.erase("wall_unix_sec");
  frame_metadata.erase("truth");
  frame_metadata.erase("events");

  // These fields are intentionally unusable as labels or keypoints. A raw
  // observation must reach the detector only through its BGR pixels.
  frame_metadata["appearance"]["robot"] = "must-not-label-detections";
  frame_metadata["camera"]["armors"] = "must-not-be-read";

  cv::Mat bgr_image(1080, 1440, CV_8UC3, cv::Scalar(17, 31, 47));
  const auto received_at = auto_aim_bridge::SteadyClock::now();
  const auto observation = auto_aim_bridge::adapt_frame(
    frame_metadata,
    bgr_image,
    received_at);

  require(
    observation.input_mode == auto_aim_bridge::InputMode::raw_frame,
    "Rendered observation must preserve its input mode");
  require(
    observation.armors.empty(),
    "Rendered input must not convert camera.armors into algorithm detections");
  require(
    observation.bgr_image.rows == 1080 && observation.bgr_image.cols == 1440 &&
      observation.bgr_image.type() == CV_8UC3,
    "Rendered input must preserve an owned 1440x1080 BGR image");
  require(
    observation.received_at == received_at,
    "Rendered input must preserve its monotonic receive timestamp");

  Json state_metadata = make_projected_state();
  state_metadata["appearance"]["robot"] = "must-not-label-detections";
  state_metadata["camera"]["armors"] = "must-not-be-read";

  const auto inspected = auto_aim_bridge::inspect_state(
    state_metadata,
    auto_aim_bridge::InputMode::raw_frame);

  require(
    inspected.armors.empty(),
    "Raw-mode state inspection must ignore ideal corner annotations");

  cv::Mat wrong_size(720, 1280, CV_8UC3);
  require_invalid_argument(
    [&] {
      auto_aim_bridge::adapt_frame(
        frame_metadata,
        wrong_size,
        received_at);
    },
    "A rendered image that disagrees with frame metadata must be rejected");
}

void test_raw_frame_mailbox()
{
  constexpr int image_width = 1440;
  constexpr int image_height = 1080;

  std::vector<std::uint8_t> rgb_bytes(
    static_cast<std::size_t>(image_width) * image_height * 3);

  for (std::size_t offset = 0; offset < rgb_bytes.size(); offset += 3) {
    rgb_bytes[offset] = 10;
    rgb_bytes[offset + 1] = 20;
    rgb_bytes[offset + 2] = 30;
  }

  std::vector<std::uint8_t> rgba_bytes(
    static_cast<std::size_t>(image_width) * image_height * 4);

  for (std::size_t offset = 0; offset < rgba_bytes.size(); offset += 4) {
    rgba_bytes[offset] = 4;
    rgba_bytes[offset + 1] = 5;
    rgba_bytes[offset + 2] = 6;
    rgba_bytes[offset + 3] = 255;
  }

  TemporaryIpcDirectory ipc_directory;
  Json state = make_projected_state();
  int publication = 0;

  auto refresh_state = [&] {
    ++publication;
    const double sim_time = 10.0 + publication * 0.01;
    const std::uint64_t tick = 2400 + publication;
    const double wall_unix_seconds = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    state["publication"] = publication;
    state["wall_unix_sec"] = wall_unix_seconds;
    state["sim_time"] = sim_time;
    state["tick"] = tick;
    state["camera"]["timestamp"] = sim_time;
    state["camera"]["tick"] = tick;

    write_json_file(ipc_directory.file("state.json"), state);
  };

  auto publish_frame = [&](
    const Json & request,
    const std::string & encoding,
    const std::vector<std::uint8_t> & bytes,
    bool truncate_raw_file) {
    const std::string request_identity = request.at("request_id").get<std::string>();
    const std::string frame_filename = "frame-" + request_identity + ".raw";
    Json frame = state;

    frame["type"] = "frame";
    frame["request_id"] = request_identity;
    frame["frame_file"] = frame_filename;
    frame["encoding"] = encoding;
    frame["image_bytes"] = bytes.size();

    if (truncate_raw_file) {
      std::vector<std::uint8_t> truncated(bytes.begin(), bytes.end() - 1);
      write_binary_file(ipc_directory.file(frame_filename), truncated);
    } else {
      write_binary_file(ipc_directory.file(frame_filename), bytes);
    }

    write_json_file(ipc_directory.file("frame.json"), frame);
  };

  refresh_state();

  auto_aim_bridge::LocalSimClient client(ipc_directory.channel());
  require(client.poll_state().has_value(), "Client must accept a fresh test state");

  client.request_frame();
  require(client.frame_request_pending(), "Client must remember its frame request");

  Json request = read_json_file(ipc_directory.file("request.json"));
  publish_frame(request, "rgb8", rgb_bytes, false);

  const auto rgb_frame = client.poll_frame();
  require(rgb_frame.has_value(), "Client must consume a matching RGB frame");
  require(!client.frame_request_pending(), "Completed RGB frame must acknowledge its request");
  require(
    rgb_frame->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(30, 20, 10),
    "RGB input must be copied and converted to BGR");

  refresh_state();
  require(client.poll_state().has_value(), "Client must refresh before the malformed frame");
  client.request_frame();
  request = read_json_file(ipc_directory.file("request.json"));
  publish_frame(request, "rgb8", rgb_bytes, true);

  require_runtime_error(
    [&] { client.poll_frame(); },
    "Client must reject a raw file whose size disagrees with frame metadata");
  require(
    !client.frame_request_pending(),
    "A malformed raw frame must cancel its pending request");

  refresh_state();
  require(client.poll_state().has_value(), "Client must refresh before the RGBA frame");
  client.request_frame();
  request = read_json_file(ipc_directory.file("request.json"));
  publish_frame(request, "rgba8", rgba_bytes, false);

  const auto rgba_frame = client.poll_frame();
  require(rgba_frame.has_value(), "Client must consume a matching RGBA frame");
  require(
    rgba_frame->bgr.at<cv::Vec3b>(0, 0) == cv::Vec3b(6, 5, 4),
    "RGBA input must drop alpha and convert to BGR");
}

void test_projected_pose(const TemporaryDirectory & temporary_directory)
{
  for (const std::string & robot : {std::string("infantry_3"), std::string("hero")}) {
    const auto output_directory = temporary_directory.child("pose_" + robot);
    AutoAimPipeline pipeline(
      BRIDGE_DEFAULT_CONFIG_PATH,
      output_directory);

    const Json state = make_projected_state(robot);
    const auto result = pipeline.process(auto_aim_bridge::adapt_state(state), 1);
    const Json & solved = single_solved_armor(result);
    const YAML::Node effective_config = YAML::LoadFile(
      (output_directory / "effective_config_1.yaml").string());

    require_near(
      effective_config["high_speed_delay_time"].as<double>(),
      state["weapon"]["delay"].get<double>(),
      1e-12,
      "High-speed prediction must include simulator muzzle release delay");
    require_near(
      effective_config["low_speed_delay_time"].as<double>(),
      state["weapon"]["delay"].get<double>(),
      1e-12,
      "Low-speed prediction must include simulator muzzle release delay");

    const Eigen::Vector3d expected_position(4.0, 0.5, -0.35);
    const Eigen::Vector3d actual_position = vector_from_json(solved.at("position_world"));

    require(
      (actual_position - expected_position).norm() < 1e-3,
      robot + ": PnP must recover position relative to the fixed gimbal origin within 1 mm");
    require_near(
      std::remainder(solved.at("yaw_world").get<double>() - 0.35, 2.0 * kPi),
      0.0, 0.01,
      robot + ": PnP must recover the inward-normal yaw");

    require_near(
      solved.at("position_error_m").get<double>(), 0.0, 1e-3,
      robot + ": same-snapshot ground-truth position comparison must subtract gimbal origin");
    require_near(
      solved.at("yaw_error_rad").get<double>(), 0.0, 0.01,
      robot + ": ground-truth outward normal must be converted to inward yaw");

    check_finite_plan(result.plan);
  }
}

void test_truth_independence(const TemporaryDirectory & temporary_directory)
{
  AutoAimPipeline with_truth(
    BRIDGE_DEFAULT_CONFIG_PATH,
    temporary_directory.child("with_truth"));

  AutoAimPipeline without_truth(
    BRIDGE_DEFAULT_CONFIG_PATH,
    temporary_directory.child("without_truth"));

  Json state = make_projected_state();
  PipelineResult result_with_truth;

  for (int frame_index = 0; frame_index < 12; ++frame_index) {
    set_frame_time(state, frame_index);
    Json state_without_truth = state;
    state_without_truth.erase("truth");

    const auto observation_with_truth = auto_aim_bridge::adapt_state(state);
    const auto observation_without_truth = auto_aim_bridge::adapt_state(state_without_truth);

    require(
      observation_with_truth.context == observation_without_truth.context,
      "Diagnostic truth must not affect the pipeline context");

    result_with_truth = with_truth.process(observation_with_truth, 1);
    const auto result_without_truth = without_truth.process(observation_without_truth, 1);

    require(
      result_with_truth.tracker_state == result_without_truth.tracker_state,
      "Removing ground truth must preserve tracker transitions");

    const Json & solved_with_truth = single_solved_armor(result_with_truth);
    const Json & solved_without_truth = single_solved_armor(result_without_truth);

    require(
      (vector_from_json(solved_with_truth.at("position_world")) -
       vector_from_json(solved_without_truth.at("position_world"))).norm() < 1e-9,
      "Removing ground truth must preserve PnP position");
    require_near(
      solved_with_truth.at("yaw_world").get<double>(),
      solved_without_truth.at("yaw_world").get<double>(), 1e-9,
      "Removing ground truth must preserve PnP yaw");

    require(
      !solved_without_truth.contains("position_error_m") ||
        solved_without_truth.at("position_error_m").is_null(),
      "Unavailable truth must not be recorded as zero position error");

    check_finite_plan(result_with_truth.plan);
    check_finite_plan(result_without_truth.plan);
    check_same_plan(result_with_truth.plan, result_without_truth.plan);
  }

  require(
    result_with_truth.tracker_state == "tracking",
    "Repeated observations must confirm tracking");
  require(result_with_truth.plan.control, "A confirmed visible target must produce a control plan");
  require(
    result_with_truth.plan.target_yaw > 0.0F,
    "A target to the left must request positive yaw");
  require(
    result_with_truth.plan.target_pitch > 0.0F,
    "This below-gimbal fixture must request positive, downward protocol pitch");
}

void test_confirmation_and_recent_measurement_gate(
  const TemporaryDirectory & temporary_directory)
{
  AutoAimPipeline pipeline(
    BRIDGE_DEFAULT_CONFIG_PATH,
    temporary_directory.child("confirmation_gate"));

  Json state = make_projected_state();
  set_frame_time(state, 0);

  PipelineResult result = pipeline.process(auto_aim_bridge::adapt_state(state), 1);

  require(result.tracker_state == "detecting", "The first observation must be unconfirmed");
  require(
    !result.plan.control && !result.plan.fire,
    "An unconfirmed first observation must not produce control or fire");

  state["camera"]["armors"] = Json::array();
  set_frame_time(state, 1);
  result = pipeline.process(auto_aim_bridge::adapt_state(state), 1);

  require(
    result.tracker_state == "temp_lost",
    "A missed frame while detecting must retain the unconfirmed target temporarily");
  require(
    !result.plan.control && !result.plan.fire,
    "Temporary loss of an unconfirmed target must not bypass the confirmation gate");

  state = make_projected_state();
  const YAML::Node config = YAML::LoadFile(BRIDGE_DEFAULT_CONFIG_PATH);
  const int minimum_detection_count = config["min_detect_count"].as<int>();

  require(minimum_detection_count > 1, "The confirmation-gate fixture needs multiple detections");

  for (int frame_index = 0; frame_index < minimum_detection_count; ++frame_index) {
    set_frame_time(state, frame_index);
    result = pipeline.process(auto_aim_bridge::adapt_state(state), 2);

    if (frame_index + 1 < minimum_detection_count) {
      require(
        result.tracker_state == "detecting",
        "Target must remain unconfirmed before the gate");
      require(
        !result.plan.control && !result.plan.fire,
        "Pre-confirmation observations must not produce control or fire");
    }
  }

  require(
    result.tracker_state == "tracking",
    "min_detect_count observations must confirm tracking");
  require(result.plan.control, "A confirmed visible target must produce control");

  state["camera"]["armors"] = Json::array();
  set_frame_time(state, minimum_detection_count);
  result = pipeline.process(auto_aim_bridge::adapt_state(state), 2);

  require(
    result.tracker_state == "temp_lost",
    "A confirmed target may enter temporary loss after one missed frame");
  require(
    result.plan.control,
    "A confirmed target may retain predictive control while its measurement is under 100 ms old");

  const int stale_frame_index = minimum_detection_count +
    static_cast<int>(std::ceil(kMeasurementFreshnessLimit / kFrameInterval));
  for (
    int frame_index = minimum_detection_count + 1;
    frame_index <= stale_frame_index;
    ++frame_index) {
    set_frame_time(state, frame_index);
    result = pipeline.process(auto_aim_bridge::adapt_state(state), 2);
  }

  require(
    result.tracker_state == "temp_lost",
    "The stale-measurement check must remain within the tracker loss-count tolerance");
  require(
    !result.plan.control && !result.plan.fire,
    "A temporary-loss prediction older than 100 ms must not produce control or fire");
}

std::size_t yaml_file_count(const std::filesystem::path & directory)
{
  std::size_t count = 0;
  for (const auto & entry : std::filesystem::recursive_directory_iterator(directory)) {
    if (entry.is_regular_file() && entry.path().extension() == ".yaml") {
      ++count;
    }
  }
  return count;
}

void test_timestamp_loss_and_restart(const TemporaryDirectory & temporary_directory)
{
  const auto output_directory = temporary_directory.child("lifecycle");
  AutoAimPipeline pipeline(BRIDGE_DEFAULT_CONFIG_PATH, output_directory);
  Json state = make_projected_state();

  PipelineResult result;
  for (int frame_index = 0; frame_index < 8; ++frame_index) {
    set_frame_time(state, frame_index);
    result = pipeline.process(auto_aim_bridge::adapt_state(state), 1);
  }

  require(result.tracker_state == "tracking", "Lifecycle fixture must establish tracking");
  const auto duplicate_observation = auto_aim_bridge::adapt_state(state);

  require_invalid_argument(
    [&] { pipeline.process(duplicate_observation, 1); },
    "Duplicate simulation timestamps must not update the tracker twice");

  state["camera"]["armors"] = Json::array();
  for (int frame_index = 8; frame_index < 28; ++frame_index) {
    set_frame_time(state, frame_index);
    result = pipeline.process(auto_aim_bridge::adapt_state(state), 1);
  }

  require(result.tracker_state == "lost", "Sustained absent corners must lose the target");
  require(
    !result.plan.control && !result.plan.fire,
    "A lost target must not produce control or fire");
  require(
    result.diagnostics.at("solved_armors").get<int>() == 0,
    "Empty input must have no PnP poses");

  const std::size_t original_config_count = yaml_file_count(output_directory);
  require(original_config_count >= 1, "The first generation must record its effective YAML");

  Json restarted_state = make_projected_state("hero", "red");
  restarted_state["session_id"] = "restarted-session";
  set_frame_time(restarted_state, 0, 1.0);

  result = pipeline.process(auto_aim_bridge::adapt_state(restarted_state), 2);

  require(result.generation == 2, "Pipeline results must identify the new generation");
  require(result.tracker_state == "detecting", "A restarted generation must begin a new track");
  require(
    yaml_file_count(output_directory) > original_config_count,
    "Each generation must preserve a separate effective calibration YAML");
  single_solved_armor(result);

  Json changed_delay_state = make_projected_state("hero", "red");
  changed_delay_state["session_id"] = "changed-delay-session";
  changed_delay_state["weapon"]["delay"] = 0.02;
  set_frame_time(changed_delay_state, 0, 2.0);

  require_invalid_argument(
    [&] {
      pipeline.process(auto_aim_bridge::adapt_state(changed_delay_state), 3);
    },
    "Changing muzzle release delay must require a bridge restart");
}

}  // namespace

int main()
{
  try {
    TemporaryDirectory temporary_directory;

    test_corner_and_label_mapping();
    std::cout << "PASS: exact projected corners, labels and activity mapping\n";

    test_invalid_observations();
    std::cout << "PASS: unsupported and malformed observations are rejected\n";

    test_raw_frame_input_isolation();
    std::cout << "PASS: raw frames ignore ideal armor corners and labels\n";

    test_raw_frame_mailbox();
    std::cout << "PASS: raw frame mailbox validates files and converts RGB/RGBA\n";

    test_projected_pose(temporary_directory);
    std::cout << "PASS: small and large armor PnP under nonzero gimbal attitude\n";

    test_truth_independence(temporary_directory);
    std::cout << "PASS: PnP, tracker and planner do not depend on ground truth\n";

    test_confirmation_and_recent_measurement_gate(temporary_directory);
    std::cout << "PASS: confirmation and recent-measurement gates control prediction\n";

    test_timestamp_loss_and_restart(temporary_directory);
    std::cout << "PASS: duplicate time, target loss and generation restart\n";

    return EXIT_SUCCESS;
  } catch (const std::exception & error) {
    std::cerr << "FAIL: " << error.what() << '\n';
    return EXIT_FAILURE;
  }
}
