#include "uv_control_adapter.hpp"

#include <yaml-cpp/yaml.h>

#include <algorithm>
#include <cmath>
#include <stdexcept>
#include <utility>
#include <vector>

#include "tools/math_tools.hpp"

namespace auto_aim_bridge
{
namespace
{

UvControlAdapter::AssociationConfig load_association_config(
  const YAML::Node & yaml)
{
  UvControlAdapter::AssociationConfig config;
  const YAML::Node node = yaml["uv_ekf_association"];

  if (node) {
    if (node["missing_timeout_s"]) {
      config.missing_timeout_seconds = node["missing_timeout_s"].as<double>();
    }

    if (node["max_rejected_frames"]) {
      config.maximum_rejected_frames = node["max_rejected_frames"].as<unsigned int>();
    }

    if (node["nis_gate"]) {
      config.nis_gate = node["nis_gate"].as<double>();
    }

    if (node["second_best_ratio"]) {
      config.second_best_ratio = node["second_best_ratio"].as<double>();
    }

    if (node["min_nis_margin"]) {
      config.minimum_nis_margin = node["min_nis_margin"].as<double>();
    }
  }

  if (!std::isfinite(config.missing_timeout_seconds) ||
      config.missing_timeout_seconds <= 0.0 ||
      config.maximum_rejected_frames == 0 ||
      !std::isfinite(config.nis_gate) || config.nis_gate <= 0.0 ||
      !std::isfinite(config.second_best_ratio) || config.second_best_ratio <= 1.0 ||
      !std::isfinite(config.minimum_nis_margin) || config.minimum_nis_margin < 0.0) {
    throw std::invalid_argument("Invalid uv_ekf_association configuration");
  }

  return config;
}

auto_aim::Color load_enemy_color(const YAML::Node & yaml)
{
  return yaml["enemy_color"].as<std::string>() == "red"
    ? auto_aim::Color::red
    : auto_aim::Color::blue;
}

nlohmann::json optional_number(const std::optional<double> & value)
{
  return value ? nlohmann::json(*value) : nlohmann::json(nullptr);
}

nlohmann::json optional_source_id(
  const std::optional<std::uint64_t> & value)
{
  return value ? nlohmann::json(*value) : nlohmann::json(nullptr);
}

template <typename Derived>
nlohmann::json eigen_vector_json(const Eigen::MatrixBase<Derived> & vector)
{
  nlohmann::json result = nlohmann::json::array();

  for (Eigen::Index index = 0; index < vector.size(); ++index) {
    result.push_back(vector[index]);
  }

  return result;
}

}  // namespace

UvControlAdapter::UvControlAdapter(const std::filesystem::path & config_path)
: filter_(config_path.string())
{
  const YAML::Node yaml = YAML::LoadFile(config_path.string());
  association_config_ = load_association_config(yaml);
  enemy_color_ = load_enemy_color(yaml);
}

void UvControlAdapter::prepare_frame(
  std::list<auto_aim::Armor> & armors,
  const auto_aim::Solver & solver,
  std::chrono::steady_clock::time_point timestamp,
  const Eigen::Matrix3d & gimbal_to_world)
{
  const auto processing_started = std::chrono::steady_clock::now();
  frame_ = {};
  current_gimbal_to_world_ = gimbal_to_world;

  const auto finish_frame = [&] {
    frame_.processing_milliseconds = std::chrono::duration<double, std::milli>(
      std::chrono::steady_clock::now() - processing_started).count();
  };

  if (previous_frame_timestamp_) {
    frame_.dt_seconds = std::chrono::duration<double>(
      timestamp - *previous_frame_timestamp_).count();

    if (timestamp <= *previous_frame_timestamp_) {
      frame_.status = "non_increasing_timestamp";
      armors.clear();
      finish_frame();
      return;
    }
  }

  previous_frame_timestamp_ = timestamp;

  if (filter_.initialized() && last_measurement_timestamp_) {
    const double time_since_measurement = std::chrono::duration<double>(
      timestamp - *last_measurement_timestamp_).count();

    if (time_since_measurement > association_config_.missing_timeout_seconds) {
      reset_filter("missing_timeout");
      armors.clear();
      finish_frame();
      return;
    }
  }

  if (!filter_.initialized()) {
    initialize_from_pnp(
      armors,
      solver,
      timestamp,
      gimbal_to_world);

    finish_frame();
    return;
  }

  if (!filter_.predict(timestamp)) {
    const std::string prediction_status = filter_.diagnostics().status;
    reset_filter(prediction_status);
    armors.clear();
    finish_frame();
    return;
  }

  frame_.prediction_succeeded = true;
  frame_.status = "predicted";

  select_and_update(armors, gimbal_to_world);

  if (frame_.update_accepted) {
    last_measurement_timestamp_ = timestamp;
    rejected_frame_count_ = 0;
  } else {
    ++rejected_frame_count_;

    if (rejected_frame_count_ >= association_config_.maximum_rejected_frames) {
      reset_filter("rejection_limit_" + frame_.status);
    }
  }

  if (frame_.measurement_ready) {
    retain_selected_candidate(armors);
  } else {
    armors.clear();
  }

  finish_frame();
}

bool UvControlAdapter::initialize_from_pnp(
  std::list<auto_aim::Armor> & armors,
  const auto_aim::Solver & solver,
  std::chrono::steady_clock::time_point timestamp,
  const Eigen::Matrix3d & gimbal_to_world)
{
  frame_.status = "awaiting_pnp_seed";
  std::list<auto_aim::Armor> seed_candidates = armors;

  seed_candidates.remove_if([this](const auto_aim::Armor & armor) {
    return armor.color != enemy_color_ ||
      armor.name == auto_aim::ArmorName::not_armor ||
      armor.points.size() != auto_aim::Armor::N_LANDMARKS;
  });

  seed_candidates.sort([](const auto_aim::Armor & left, const auto_aim::Armor & right) {
    const cv::Point2f image_center(720.0F, 540.0F);
    return cv::norm(left.center - image_center) < cv::norm(right.center - image_center);
  });

  seed_candidates.sort([](const auto_aim::Armor & left, const auto_aim::Armor & right) {
    return left.priority < right.priority;
  });

  for (const auto & candidate : seed_candidates) {
    auto solved_seed = candidate;

    if (!solver.solve(
          solved_seed,
          candidate.points,
          std::nullopt)) {
      continue;
    }

    if (!filter_.initialize(solved_seed, timestamp, gimbal_to_world)) {
      continue;
    }

    const auto evaluation = filter_.evaluate(candidate.points, gimbal_to_world);

    if (!evaluation ||
        evaluation->reprojection_rmse_px > filter_.config().max_reprojection_rmse_px) {
      filter_.reset("seed_pixel_mismatch");
      frame_.status = "seed_pixel_mismatch";
      continue;
    }

    ++segment_id_;
    rejected_frame_count_ = 0;
    last_measurement_timestamp_ = timestamp;
    tracked_color_ = candidate.color;
    tracked_name_ = candidate.name;
    fixed_pitch_radians_ = solved_seed.ypr_in_world[1];
    fixed_roll_radians_ = solved_seed.ypr_in_world[2];

    frame_.selected_source_id = candidate.source_id;
    frame_.post_update_rmse_pixels = evaluation->reprojection_rmse_px;
    frame_.initialization_frame = true;
    frame_.measurement_ready = true;
    frame_.status = "initialized_from_pnp";

    retain_selected_candidate(armors);
    return true;
  }

  armors.clear();
  return false;
}

void UvControlAdapter::select_and_update(
  std::list<auto_aim::Armor> & armors,
  const Eigen::Matrix3d & gimbal_to_world)
{
  struct Match
  {
    const auto_aim::Armor * armor = nullptr;
    auto_aim::SimpleUvEkf::Evaluation evaluation;
  };

  std::vector<Match> matches;

  for (const auto & armor : armors) {
    if (armor.color != tracked_color_ ||
        armor.name != tracked_name_ ||
        armor.type != filter_.armor_type()) {
      continue;
    }

    const auto evaluation = filter_.evaluate(armor.points, gimbal_to_world);

    if (!evaluation || !std::isfinite(evaluation->nis) ||
        evaluation->nis > association_config_.nis_gate ||
        evaluation->reprojection_rmse_px > filter_.config().max_reprojection_rmse_px) {
      continue;
    }

    matches.push_back(Match{&armor, *evaluation});
  }

  std::sort(matches.begin(), matches.end(), [](const Match & left, const Match & right) {
    return left.evaluation.nis < right.evaluation.nis;
  });

  frame_.status = "no_pixel_match";

  const bool ambiguous = matches.size() > 1 &&
    (matches[1].evaluation.nis <
       matches[0].evaluation.nis * association_config_.second_best_ratio ||
     matches[1].evaluation.nis - matches[0].evaluation.nis <
       association_config_.minimum_nis_margin);

  if (ambiguous) {
    frame_.status = "ambiguous_pixel_match";
    return;
  }

  if (matches.empty()) {
    return;
  }

  const Match & match = matches.front();
  frame_.selected_source_id = match.armor->source_id;
  frame_.pre_update_rmse_pixels = match.evaluation.reprojection_rmse_px;
  frame_.nis = match.evaluation.nis;
  frame_.update_attempted = true;
  frame_.update_accepted = filter_.update(match.armor->points, gimbal_to_world);
  frame_.status = filter_.diagnostics().status;

  if (const auto posterior = filter_.evaluate(match.armor->points, gimbal_to_world)) {
    frame_.post_update_rmse_pixels = posterior->reprojection_rmse_px;
  }

  frame_.measurement_ready = frame_.update_accepted;
}

void UvControlAdapter::retain_selected_candidate(
  std::list<auto_aim::Armor> & armors) const
{
  if (!frame_.selected_source_id) {
    armors.clear();
    return;
  }

  const std::uint64_t selected_source_id = *frame_.selected_source_id;

  armors.remove_if([selected_source_id](const auto_aim::Armor & armor) {
    return armor.source_id != selected_source_id;
  });
}

auto_aim::TrackerMeasurementOverrideResult UvControlAdapter::override_measurement(
  auto_aim::Armor & armor)
{
  auto_aim::TrackerMeasurementOverrideResult result;
  result.source_id = armor.source_id;

  if (!frame_.measurement_ready || !frame_.selected_source_id ||
      armor.source_id != *frame_.selected_source_id || !filter_.initialized()) {
    result.accepted = false;
    result.reason = "uv_measurement_not_ready";
    return result;
  }

  const auto state = filter_.state();
  armor.xyz_in_world = state.segment<3>(0);
  armor.ypd_in_world = tools::xyz2ypd(armor.xyz_in_world);
  armor.ypr_in_world = Eigen::Vector3d(
    state[6],
    fixed_pitch_radians_,
    fixed_roll_radians_);

  armor.xyz_in_gimbal = current_gimbal_to_world_.transpose() * armor.xyz_in_world;
  armor.ypr_in_gimbal = tools::eulers(
    current_gimbal_to_world_.transpose() *
      tools::rotation_matrix(armor.ypr_in_world),
    2,
    1,
    0);

  if (!armor.xyz_in_world.allFinite() ||
      !armor.ypd_in_world.allFinite() ||
      !armor.ypr_in_world.allFinite() ||
      !armor.xyz_in_gimbal.allFinite() ||
      !armor.ypr_in_gimbal.allFinite()) {
    result.accepted = false;
    result.reason = "uv_measurement_nonfinite";
    return result;
  }

  frame_.measurement_injected = true;
  result.accepted = true;
  result.reason = frame_.initialization_frame
    ? "uv_initialized_from_pnp"
    : "uv_updated_from_pixels";
  return result;
}

void UvControlAdapter::reset_filter(const std::string & status)
{
  filter_.reset(status);
  last_measurement_timestamp_.reset();
  tracked_name_ = auto_aim::ArmorName::not_armor;
  rejected_frame_count_ = 0;

  frame_.selected_source_id.reset();
  frame_.measurement_ready = false;
  frame_.reset = true;
  frame_.status = status;
}

nlohmann::json UvControlAdapter::diagnostics() const
{
  nlohmann::json result = {
    {"segment_id", segment_id_},
    {"initialized", filter_.initialized()},
    {"initialization_frame", frame_.initialization_frame},
    {"prediction_succeeded", frame_.prediction_succeeded},
    {"update_attempted", frame_.update_attempted},
    {"update_accepted", frame_.update_accepted},
    {"reset", frame_.reset},
    {"measurement_ready", frame_.measurement_ready},
    {"measurement_injected", frame_.measurement_injected},
    {"status", frame_.status},
    {"selected_source_id", optional_source_id(frame_.selected_source_id)},
    {"dt_s", optional_number(frame_.dt_seconds)},
    {"pre_update_rmse_px", optional_number(frame_.pre_update_rmse_pixels)},
    {"post_update_rmse_px", optional_number(frame_.post_update_rmse_pixels)},
    {"nis", optional_number(frame_.nis)},
    {"processing_ms", frame_.processing_milliseconds}
  };

  if (filter_.initialized()) {
    result["state"] = eigen_vector_json(filter_.state());
    result["covariance_diagonal"] = eigen_vector_json(
      filter_.covariance().diagonal());
  } else {
    result["state"] = nullptr;
    result["covariance_diagonal"] = nullptr;
  }

  return result;
}

}  // namespace auto_aim_bridge
