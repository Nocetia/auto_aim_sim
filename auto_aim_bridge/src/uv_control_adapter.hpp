#pragma once

#include <Eigen/Core>

#include <chrono>
#include <cstdint>
#include <filesystem>
#include <list>
#include <optional>
#include <string>

#include <nlohmann/json.hpp>

#include "tasks/auto_aim/simple_uv_ekf.hpp"
#include "tasks/auto_aim/solver.hpp"
#include "tasks/auto_aim/tracker.hpp"

namespace auto_aim_bridge
{

// Adapts the upstream single-plate pixel EKF to Tracker's existing
// armor-measurement boundary. The downstream Target EKF and Planner remain
// unchanged because they model the complete rotating vehicle, not one plate.
class UvControlAdapter
{
public:
  struct AssociationConfig
  {
    double missing_timeout_seconds = 0.15;
    unsigned int maximum_rejected_frames = 5;
    double nis_gate = 26.124481558;
    double second_best_ratio = 1.5;
    double minimum_nis_margin = 1.0;
  };

  explicit UvControlAdapter(const std::filesystem::path & config_path);

  // Selects at most one detector candidate for this frame. On an accepted UV
  // update the list is reduced to that candidate. Otherwise it is cleared so
  // Tracker performs prediction only; the current frame never silently falls
  // back to PnP.
  void prepare_frame(
    std::list<auto_aim::Armor> & armors,
    const auto_aim::Solver & solver,
    std::chrono::steady_clock::time_point timestamp,
    const Eigen::Matrix3d & gimbal_to_world);

  auto_aim::TrackerMeasurementOverrideResult override_measurement(
    auto_aim::Armor & armor);

  nlohmann::json diagnostics() const;

private:
  struct FrameDiagnostics
  {
    std::optional<std::uint64_t> selected_source_id;
    std::optional<double> dt_seconds;
    std::optional<double> pre_update_rmse_pixels;
    std::optional<double> post_update_rmse_pixels;
    std::optional<double> nis;
    bool initialization_frame = false;
    bool prediction_succeeded = false;
    bool update_attempted = false;
    bool update_accepted = false;
    bool reset = false;
    bool measurement_ready = false;
    bool measurement_injected = false;
    std::string status = "not_initialized";
    double processing_milliseconds = 0.0;
  };

  auto_aim::SimpleUvEkf filter_;
  AssociationConfig association_config_;
  auto_aim::Color enemy_color_ = auto_aim::Color::blue;
  std::optional<std::chrono::steady_clock::time_point> previous_frame_timestamp_;
  std::optional<std::chrono::steady_clock::time_point> last_measurement_timestamp_;
  auto_aim::Color tracked_color_ = auto_aim::Color::blue;
  auto_aim::ArmorName tracked_name_ = auto_aim::ArmorName::not_armor;
  double fixed_pitch_radians_ = 0.0;
  double fixed_roll_radians_ = 0.0;
  Eigen::Matrix3d current_gimbal_to_world_ = Eigen::Matrix3d::Identity();
  std::uint64_t segment_id_ = 0;
  unsigned int rejected_frame_count_ = 0;
  FrameDiagnostics frame_;

  bool initialize_from_pnp(
    std::list<auto_aim::Armor> & armors,
    const auto_aim::Solver & solver,
    std::chrono::steady_clock::time_point timestamp,
    const Eigen::Matrix3d & gimbal_to_world);

  void select_and_update(
    std::list<auto_aim::Armor> & armors,
    const Eigen::Matrix3d & gimbal_to_world);

  void retain_selected_candidate(std::list<auto_aim::Armor> & armors) const;
  void reset_filter(const std::string & status);
};

}  // namespace auto_aim_bridge
