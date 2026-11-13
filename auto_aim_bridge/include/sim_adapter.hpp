#pragma once

#include <Eigen/Geometry>
#include <chrono>
#include <cstdint>
#include <filesystem>
#include <list>
#include <memory>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>
#include <opencv2/core/mat.hpp>

#include "tasks/auto_aim/armor.hpp"
#include "tasks/auto_aim/planner/planner.hpp"

namespace auto_aim_bridge
{

using SteadyClock = std::chrono::steady_clock;
using Json = nlohmann::json;

enum class InputMode
{
  exact_corners,
  raw_frame
};

const char * input_mode_name(InputMode input_mode);

enum class EstimatorMode
{
  pnp,
  uv_ekf
};

const char * estimator_mode_name(EstimatorMode estimator_mode);

struct GroundTruthAngularVelocitySample
{
  std::string session_id;
  std::int64_t target_id = -1;
  std::string robot_id;
  std::uint64_t tick = 0;
  double timestamp = 0.0;
  double yaw_rad_s = 0.0;
};

struct Observation
{
  Json state;
  std::string context;
  std::list<auto_aim::Armor> armors;
  cv::Mat bgr_image;
  InputMode input_mode = InputMode::exact_corners;
  Eigen::Quaterniond gimbal_to_world = Eigen::Quaterniond::Identity();
  SteadyClock::time_point received_at;
  double sim_time = 0.0;
  double bullet_speed = 0.0;
  double firing_delay = 0.0;
  std::optional<GroundTruthAngularVelocitySample> ground_truth_angular_velocity;
  std::string ground_truth_angular_velocity_status = "missing";
  bool active = false;
};

// Converts already-projected TL/TR/BR/BL pixels. No 3D truth is used here.
// Throws with a field-specific explanation for unsupported or malformed input.
Observation adapt_state(const Json & state);

// Validates state metadata and computes its lifecycle context without creating
// an algorithm observation. Raw-frame mode deliberately ignores camera.armors.
Observation inspect_state(const Json & state, InputMode input_mode);

// Validates one rendered frame and keeps its owned BGR image together with the
// exact metadata snapshot that produced it. camera.armors is not converted or
// used as an algorithm input in this mode.
Observation adapt_frame(
  const Json & frame_metadata,
  cv::Mat bgr_image,
  SteadyClock::time_point received_at);

struct PipelineResult
{
  std::uint64_t generation = 0;
  double source_sim_time = 0.0;
  double planning_sim_time = 0.0;
  SteadyClock::time_point observation_received_at;
  auto_aim::Plan plan;
  std::string tracker_state = "lost";
  bool ground_truth_angular_velocity_enabled = false;
  Json diagnostics;
};

// One worker owns the entire algorithm pipeline. The supplied directory must
// already exist; each generation records its own effective calibration YAML.
class AutoAimPipeline
{
public:
  AutoAimPipeline(
    const std::filesystem::path & config_path,
    const std::filesystem::path & output_directory,
    InputMode input_mode = InputMode::exact_corners,
    EstimatorMode estimator_mode = EstimatorMode::pnp);

  ~AutoAimPipeline();

  AutoAimPipeline(const AutoAimPipeline &) = delete;
  AutoAimPipeline & operator=(const AutoAimPipeline &) = delete;

  PipelineResult process(const Observation & observation, std::uint64_t generation);

  // Replans the newest tracked target at the latest simulator state time.
  // Call this from the same worker that owns process().
  PipelineResult replan(double planning_sim_time, std::uint64_t generation);

private:
  struct Implementation;
  std::unique_ptr<Implementation> implementation_;
};

}  // namespace auto_aim_bridge
