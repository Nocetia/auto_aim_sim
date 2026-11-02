#include "sim_adapter.hpp"
#include "sim_client.hpp"

#include <unistd.h>

#include <chrono>
#include <cmath>
#include <condition_variable>
#include <csignal>
#include <exception>
#include <filesystem>
#include <fstream>
#include <iomanip>
#include <iostream>
#include <mutex>
#include <optional>
#include <stdexcept>
#include <string>
#include <thread>
#include <utility>

namespace
{

using auto_aim_bridge::Json;
using auto_aim_bridge::InputMode;
using auto_aim_bridge::Observation;
using auto_aim_bridge::PipelineResult;
using auto_aim_bridge::SteadyClock;

volatile std::sig_atomic_t stop_requested = 0;

void request_stop(int)
{
  stop_requested = 1;
}

struct Options
{
  std::string channel = "default";
  std::filesystem::path config_path = BRIDGE_DEFAULT_CONFIG_PATH;
  std::filesystem::path output_directory;
  InputMode input_mode = InputMode::exact_corners;
  double duration_seconds = 0.0;
  bool control = false;
  bool allow_fire = false;
  bool help = false;
};

Options parse_options(int argument_count, char ** arguments)
{
  Options options;
  bool explicit_dry_run = false;

  for (int index = 1; index < argument_count; ++index) {
    const std::string argument = arguments[index];

    if (argument == "--help" || argument == "-h") {
      options.help = true;
    } else if (argument == "--control") {
      options.control = true;
    } else if (argument == "--allow-fire") {
      options.allow_fire = true;
    } else if (argument == "--dry-run") {
      explicit_dry_run = true;
    } else if (argument.rfind("--channel=", 0) == 0) {
      options.channel = argument.substr(10);
    } else if (argument.rfind("--config=", 0) == 0) {
      options.config_path = argument.substr(9);
    } else if (argument.rfind("--output=", 0) == 0) {
      options.output_directory = argument.substr(9);
    } else if (argument.rfind("--input=", 0) == 0) {
      const std::string input_name = argument.substr(8);

      if (input_name == "exact-corners") {
        options.input_mode = InputMode::exact_corners;
      } else if (input_name == "raw-frame") {
        options.input_mode = InputMode::raw_frame;
      } else {
        throw std::invalid_argument(
          "--input must be exact-corners or raw-frame");
      }
    } else if (argument.rfind("--duration=", 0) == 0) {
      const std::string duration_text = argument.substr(11);
      std::size_t consumed = 0;
      options.duration_seconds = std::stod(duration_text, &consumed);

      if (consumed != duration_text.size() || !std::isfinite(options.duration_seconds) ||
          options.duration_seconds <= 0.0) {
        throw std::invalid_argument(
          "--duration must be a finite positive number of wall-clock seconds");
      }
    } else {
      throw std::invalid_argument("Unknown argument: " + argument);
    }
  }

  if (options.control && explicit_dry_run) {
    throw std::invalid_argument("Use either --dry-run or --control");
  }

  if (options.allow_fire && !options.control) {
    throw std::invalid_argument("--allow-fire requires --control");
  }

  return options;
}

void print_help()
{
  std::cout
    << "Usage: auto_aim_bridge [options]\n\n"
    << "  --channel=NAME      Simulator IPC channel (default: default)\n"
    << "  --input=MODE        exact-corners (default) or raw-frame\n"
    << "  --config=PATH       Detector/Tracker/Planner YAML (default: bundled config/default.yaml)\n"
    << "  --output=PATH       New or empty log directory\n"
    << "  --duration=SECONDS  Stop after this wall-clock duration; otherwise Ctrl+C\n"
    << "  --dry-run           Calculate and record without writing commands (default)\n"
    << "  --control           Enable gimbal tracking; firing remains disabled by default\n"
    << "  --allow-fire        Forward fresh Planner fire intent; requires --control\n"
    << "  --help              Show this help\n\n"
    << "exact-corners reads state.camera.armors and supports --headless.\n"
    << "raw-frame requests frame-*.raw/frame.json and requires graphical mode.\n"
    << "Start the simulator first, with --remote and a 1440x1080 camera.\n";
}

std::filesystem::path prepare_output_directory(const std::filesystem::path & requested_path)
{
  std::filesystem::path output_path = requested_path;

  if (output_path.empty()) {
    const auto unix_milliseconds = std::chrono::duration_cast<std::chrono::milliseconds>(
      std::chrono::system_clock::now().time_since_epoch()).count();

    output_path = std::filesystem::path("logs") /
      ("auto_aim_bridge_" + std::to_string(unix_milliseconds) + "_" + std::to_string(getpid()));
  }

  output_path = std::filesystem::absolute(output_path);

  if (std::filesystem::exists(output_path) &&
      (!std::filesystem::is_directory(output_path) || !std::filesystem::is_empty(output_path))) {
    throw std::invalid_argument(
      "--output must be a new or empty directory: " + output_path.string());
  }

  std::filesystem::create_directories(output_path);
  return output_path;
}

std::ofstream open_log(const std::filesystem::path & path)
{
  std::ofstream stream(path);
  stream.exceptions(std::ios::failbit | std::ios::badbit);
  return stream;
}

struct PendingObservation
{
  Observation observation;
  std::uint64_t generation = 0;
};

struct PlanningRequest
{
  double sim_time = 0.0;
  std::uint64_t generation = 0;
};

class AlgorithmWorker
{
public:
  AlgorithmWorker(
    const std::filesystem::path & config_path,
    const std::filesystem::path & output_directory,
    InputMode input_mode)
  : thread_([this, config_path, output_directory, input_mode] {
      run(config_path, output_directory, input_mode);
    })
  {
  }

  ~AlgorithmWorker()
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      stopping_ = true;
      pending_.reset();
    }

    ready_.notify_one();
    thread_.join();
  }

  void submit(Observation observation, std::uint64_t generation)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      pending_ = PendingObservation{std::move(observation), generation};
    }

    ready_.notify_one();
  }

  void invalidate()
  {
    std::lock_guard<std::mutex> lock(mutex_);
    pending_.reset();
    planning_request_.reset();
    latest_result_.reset();
  }

  void request_plan(double sim_time, std::uint64_t generation)
  {
    {
      std::lock_guard<std::mutex> lock(mutex_);
      planning_request_ = PlanningRequest{sim_time, generation};
    }

    ready_.notify_one();
  }

  std::optional<PipelineResult> latest_result()
  {
    std::lock_guard<std::mutex> lock(mutex_);

    if (failure_) {
      std::rethrow_exception(failure_);
    }

    return latest_result_;
  }

private:
  void run(
    const std::filesystem::path & config_path,
    const std::filesystem::path & output_directory,
    InputMode input_mode)
  {
    try {
      auto_aim_bridge::AutoAimPipeline pipeline(
        config_path,
        output_directory,
        input_mode);
      auto observations_log = open_log(output_directory / "observations.jsonl");

      while (true) {
        std::optional<PendingObservation> sample;
        std::optional<PlanningRequest> planning_request;

        {
          std::unique_lock<std::mutex> lock(mutex_);
          ready_.wait(lock, [this] {
            return stopping_ || pending_.has_value() || planning_request_.has_value();
          });

          if (stopping_) {
            return;
          }

          if (pending_) {
            sample = std::move(pending_);
            pending_.reset();

            if (planning_request_ &&
                planning_request_->generation == sample->generation &&
                planning_request_->sim_time <= sample->observation.sim_time) {
              planning_request_.reset();
            }
          } else {
            planning_request = planning_request_;
            planning_request_.reset();
          }
        }

        PipelineResult result;

        if (sample) {
          const auto processing_started = SteadyClock::now();
          result = pipeline.process(sample->observation, sample->generation);

          result.diagnostics["processing_ms"] = std::chrono::duration<double, std::milli>(
            SteadyClock::now() - processing_started).count();

          observations_log << Json({
            {"state", sample->observation.state},
            {"result", result.diagnostics}
          }).dump() << '\n';

          observations_log.flush();
        } else {
          result = pipeline.replan(
            planning_request->sim_time,
            planning_request->generation);
        }

        {
          std::lock_guard<std::mutex> lock(mutex_);
          latest_result_ = std::move(result);
        }
      }
    } catch (...) {
      std::lock_guard<std::mutex> lock(mutex_);
      failure_ = std::current_exception();
    }
  }

  std::mutex mutex_;
  std::condition_variable ready_;
  bool stopping_ = false;
  std::optional<PendingObservation> pending_;
  std::optional<PlanningRequest> planning_request_;
  std::optional<PipelineResult> latest_result_;
  std::exception_ptr failure_;
  std::thread thread_;
};

bool command_is_current(
  const PipelineResult & result,
  std::uint64_t generation,
  double current_sim_time,
  SteadyClock::time_point current_time)
{
  const double wall_age = std::chrono::duration<double>(
    current_time - result.observation_received_at).count();

  const double simulation_age = current_sim_time - result.planning_sim_time;

  return result.generation == generation && result.plan.control &&
    wall_age >= 0.0 && wall_age <= 0.1 &&
    simulation_age >= -1e-6 && simulation_age <= 0.03;
}

int run(const Options & options)
{
  const auto config_path = std::filesystem::absolute(options.config_path);
  const auto output_directory = prepare_output_directory(options.output_directory);
  auto commands_log = open_log(output_directory / "commands.jsonl");
  auto run_info = open_log(output_directory / "run.json");

  run_info << Json({
    {"channel", options.channel},
    {"control_enabled", options.control},
    {"fire_enabled", options.allow_fire},
    {"input", auto_aim_bridge::input_mode_name(options.input_mode)},
    {"detector", options.input_mode == InputMode::raw_frame
      ? Json({
          {"backend", "auto_aim::YOLO"},
          {"debug", false},
          {"effective_config", (output_directory / "detector_config.yaml").string()}
        })
      : Json(nullptr)},
    {"config", config_path.string()},
    {"simulator_revision", {
      {"commit", BRIDGE_SIM_COMMIT},
      {"dirty", BRIDGE_SIM_DIRTY != 0}
    }},
    {"autoaim_revision", {
      {"root", BRIDGE_AUTOAIM_ROOT},
      {"commit", BRIDGE_AUTOAIM_COMMIT},
      {"dirty", BRIDGE_AUTOAIM_DIRTY != 0}
    }},
    {"reduced_dof_pnp", AUTOAIM_USE_REDUCED_DOF_PNP != 0},
    {"uv_ekf", false}
  }).dump(2) << '\n';

  run_info.close();

  // Upstream detectors create their optional debug directories relative to
  // the process working directory. Keep those artifacts inside this run's
  // output directory instead of polluting the simulator checkout.
  std::filesystem::current_path(output_directory);

  // Declaration order ensures the client's stop command is sent before waiting
  // for the worker to finish if an exception unwinds this function.
  AlgorithmWorker worker(
    config_path,
    output_directory,
    options.input_mode);
  auto_aim_bridge::LocalSimClient client(options.channel);

  const auto started_at = SteadyClock::now();
  auto last_new_state_at = started_at;
  auto next_planning_at = started_at;
  auto next_command_at = started_at;
  auto next_report_at = started_at;

  std::optional<double> previous_state_sim_time;
  std::optional<double> previous_observation_sim_time;
  std::string current_context;
  std::uint64_t generation = 0;
  double current_sim_time = 0.0;
  bool active = false;
  bool stale_stop_was_sent = false;

  const std::string operating_mode = options.control
    ? (options.allow_fire ? "CONTROL + FIRE" : "CONTROL")
    : "DRY RUN";

  std::cout << "Auto-aim bridge: " << operating_mode << '\n'
            << "Input: " << auto_aim_bridge::input_mode_name(options.input_mode) << '\n'
            << "Channel: " << options.channel << '\n'
            << "Logs: " << output_directory << '\n';

  while (!stop_requested) {
    const auto current_time = SteadyClock::now();

    const double running_seconds = std::chrono::duration<double>(
      current_time - started_at).count();

    if (options.duration_seconds > 0.0 && running_seconds >= options.duration_seconds) {
      break;
    }

    if (const auto state = client.poll_state()) {
      auto state_snapshot = options.input_mode == InputMode::exact_corners
        ? auto_aim_bridge::adapt_state(*state)
        : auto_aim_bridge::inspect_state(*state, InputMode::raw_frame);

      last_new_state_at = current_time;
      current_sim_time = state_snapshot.sim_time;
      active = state_snapshot.active;
      stale_stop_was_sent = false;

      const bool context_changed = current_context != state_snapshot.context;
      const bool time_reversed = previous_state_sim_time &&
        state_snapshot.sim_time < *previous_state_sim_time;

      if (context_changed || time_reversed) {
        ++generation;
        current_context = state_snapshot.context;
        previous_state_sim_time.reset();
        previous_observation_sim_time.reset();
        client.cancel_frame_request();
        worker.invalidate();

        std::cout << "Generation " << generation << ": session="
                  << state->at("session_id").get<std::string>()
                  << ", active=" << std::boolalpha << active << '\n';
      }

      if (options.input_mode == InputMode::exact_corners &&
          active &&
          (!previous_state_sim_time ||
           state_snapshot.sim_time > *previous_state_sim_time)) {
        previous_observation_sim_time = state_snapshot.sim_time;
        worker.submit(std::move(state_snapshot), generation);
      }

      previous_state_sim_time = current_sim_time;
    }

    if (options.input_mode == InputMode::raw_frame) {
      if (auto frame = client.poll_frame()) {
        auto observation = auto_aim_bridge::adapt_frame(
          frame->metadata,
          std::move(frame->bgr),
          frame->received_at);

        const bool frame_is_current =
          observation.context == current_context &&
          observation.active &&
          (!previous_observation_sim_time ||
           observation.sim_time > *previous_observation_sim_time);

        if (frame_is_current) {
          previous_observation_sim_time = observation.sim_time;
          worker.submit(std::move(observation), generation);
        }
      }

      if (active && client.fresh() && !client.frame_request_pending()) {
        client.request_frame();
      }
    }

    auto result = worker.latest_result();
    const bool result_matches_generation = result && result->generation == generation;
    const bool state_is_fresh = client.fresh();

    if (active && state_is_fresh && result_matches_generation &&
        current_time >= next_planning_at) {
      worker.request_plan(current_sim_time, generation);
      next_planning_at = current_time + std::chrono::milliseconds(10);
    }

    result = worker.latest_result();
    const bool has_current_result = result && result->generation == generation;

    const bool may_control = options.control && active && state_is_fresh && has_current_result &&
      command_is_current(*result, generation, current_sim_time, current_time);

    const bool planner_requests_fire = has_current_result && result->plan.fire;
    const bool may_fire =
      options.allow_fire && may_control && result->tracker_state == "tracking" &&
      planner_requests_fire;

    const bool should_send_command =
      options.control && active && current_time >= next_command_at &&
      (state_is_fresh || !stale_stop_was_sent);

    if (should_send_command) {
      auto_aim::Plan command;

      if (may_control) {
        command = result->plan;
      }

      const auto published = client.send(
        may_control,
        may_fire,
        command.yaw,
        command.pitch,
        command.yaw_vel,
        command.pitch_vel,
        command.yaw_acc,
        command.pitch_acc);

      if (published) {
        commands_log << Json({
          {"generation", generation},
          {"source_sim_time", has_current_result ? result->source_sim_time : current_sim_time},
          {"planning_sim_time", has_current_result ? result->planning_sim_time : current_sim_time},
          {"state_sim_time", current_sim_time},
          {"seq", published->sequence},
          {"control", published->control},
          {"tracker_state", result ? result->tracker_state : "waiting"},
          {"planner_fire_intent", planner_requests_fire},
          {"fire", published->fire},
          {"yaw", command.yaw},
          {"pitch", command.pitch},
          {"yaw_vel", command.yaw_vel},
          {"pitch_vel", command.pitch_vel},
          {"yaw_acc", command.yaw_acc},
          {"pitch_acc", command.pitch_acc},
          {"ipc_feedback", client.state().at("ipc")}
        }).dump() << '\n';

        commands_log.flush();
      }

      if (!state_is_fresh) {
        stale_stop_was_sent = true;
      }

      next_command_at = current_time + std::chrono::milliseconds(20);
    }

    if (current_time >= next_report_at) {
      std::cout << std::fixed << std::setprecision(3)
                << "sim=" << current_sim_time
                << " tracker=" << (result ? result->tracker_state : "waiting")
                << " control=" << std::boolalpha << may_control
                << " planner_fire=" << planner_requests_fire
                << " fire=" << may_fire;

      if (result) {
        std::cout << " yaw=" << result->plan.yaw
                  << " pitch=" << result->plan.pitch
                  << " inputs=" << result->diagnostics.at("input_armors").get<std::size_t>();
      }

      std::cout << '\n';
      next_report_at = current_time + std::chrono::seconds(1);
    }

    if (current_time - last_new_state_at > std::chrono::seconds(3)) {
      throw std::runtime_error("No fresh simulator state for three seconds");
    }

    std::this_thread::sleep_for(std::chrono::milliseconds(2));
  }

  if (options.control && active) {
    client.send(false, false, 0.0, 0.0);
  }

  return 0;
}

}  // namespace

int main(int argument_count, char ** arguments)
{
  try {
    const Options options = parse_options(argument_count, arguments);

    if (options.help) {
      print_help();
      return 0;
    }

    std::signal(SIGINT, request_stop);
    std::signal(SIGTERM, request_stop);

    return run(options);
  } catch (const std::exception & error) {
    std::cerr << "auto_aim_bridge: " << error.what() << '\n';
    return 1;
  }
}
