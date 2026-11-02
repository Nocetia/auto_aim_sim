// MIT License. See ../../LICENSE.
#include "sim_client.hpp"

#include <algorithm>
#include <cerrno>
#include <cmath>
#include <cstdio>
#include <fcntl.h>
#include <opencv2/imgproc.hpp>
#include <regex>
#include <stdexcept>
#include <sys/file.h>
#include <sys/stat.h>
#include <unistd.h>

namespace auto_aim_bridge
{
namespace
{

constexpr std::int64_t maximum_sequence = 9007199254740991LL;
constexpr off_t maximum_json_bytes = 65536;
constexpr auto state_timeout = std::chrono::milliseconds(200);
constexpr auto frame_timeout = std::chrono::seconds(5);

constexpr int minimum_frame_width = 160;
constexpr int maximum_frame_width = 3840;
constexpr int minimum_frame_height = 120;
constexpr int maximum_frame_height = 2160;

bool is_protocol_integer(const nlohmann::json & value, std::int64_t minimum)
{
  if (!value.is_number_integer()) {
    return false;
  }

  if (value.is_number_unsigned()) {
    return value.get<std::uint64_t>() <= static_cast<std::uint64_t>(maximum_sequence);
  }

  const auto integer = value.get<std::int64_t>();
  return integer >= minimum && integer <= maximum_sequence;
}

int bounded_frame_dimension(
  const nlohmann::json & value,
  int minimum,
  int maximum,
  const char * field_name)
{
  if (!value.is_number()) {
    throw std::runtime_error(std::string(field_name) + " must be a number");
  }

  const double numeric_value = value.get<double>();

  if (!std::isfinite(numeric_value) ||
      numeric_value != std::floor(numeric_value) ||
      numeric_value < minimum ||
      numeric_value > maximum) {
    throw std::runtime_error(std::string(field_name) + " is outside the supported range");
  }

  return static_cast<int>(numeric_value);
}

std::size_t exact_byte_count(
  const nlohmann::json & value,
  std::size_t expected_byte_count)
{
  if (!value.is_number()) {
    throw std::runtime_error("frame.image_bytes must be a number");
  }

  const double numeric_value = value.get<double>();

  if (!std::isfinite(numeric_value) ||
      numeric_value != std::floor(numeric_value) ||
      numeric_value < 0.0 ||
      numeric_value != static_cast<double>(expected_byte_count)) {
    throw std::runtime_error("Frame image_bytes does not match its dimensions and encoding");
  }

  return expected_byte_count;
}

}  // namespace

LocalSimClient::LocalSimClient(const std::string & channel)
{
  if (!std::regex_match(channel, std::regex("[A-Za-z0-9_-]{1,80}"))) {
    throw std::runtime_error("Channel must contain 1-80 letters, digits, '_' or '-'");
  }

  const std::string directory_path = "/dev/shm/rm_aim_sim_" + channel;
  directory_descriptor_ = ::open(
    directory_path.c_str(),
    O_RDONLY | O_DIRECTORY | O_CLOEXEC | O_NOFOLLOW);

  if (directory_descriptor_ < 0) {
    throw std::runtime_error("Start the simulator first; cannot open " + directory_path);
  }

  struct stat directory_information {};
  const bool directory_is_valid =
    ::fstat(directory_descriptor_, &directory_information) == 0 &&
    directory_information.st_uid == ::getuid() &&
    (directory_information.st_mode & 0777) == 0700;

  if (!directory_is_valid) {
    ::close(directory_descriptor_);
    directory_descriptor_ = -1;
    throw std::runtime_error("IPC directory must belong to this UID with mode 0700");
  }

  lock_descriptor_ = ::openat(
    directory_descriptor_,
    "client.lock",
    O_CREAT | O_RDWR | O_CLOEXEC | O_NOFOLLOW,
    0600);

  if (lock_descriptor_ < 0 || ::flock(lock_descriptor_, LOCK_EX | LOCK_NB) != 0) {
    if (lock_descriptor_ >= 0) {
      ::close(lock_descriptor_);
      lock_descriptor_ = -1;
    }

    ::close(directory_descriptor_);
    directory_descriptor_ = -1;

    throw std::runtime_error("Cannot lock client.lock; another client may own this channel");
  }

  frame_client_token_ =
    std::to_string(::getpid()) + "-" +
    std::to_string(Clock::now().time_since_epoch().count());
}

LocalSimClient::~LocalSimClient()
{
  try {
    const bool simulator_accepts_commands =
      !latest_state_.empty() &&
      !latest_state_.at("paused").get<bool>() &&
      latest_state_.at("ipc").at("enabled").get<bool>();

    if (last_published_control_ && simulator_accepts_commands) {
      send(false, false, 0.0, 0.0);
    }
  } catch (...) {
    // Destruction must release the client lock even if the simulator has exited.
  }

  ::close(lock_descriptor_);
  ::close(directory_descriptor_);
}

std::optional<nlohmann::json> LocalSimClient::poll_state()
{
  auto incoming_state = read_json("state.json");

  if (!has_valid_state_envelope(incoming_state)) {
    return std::nullopt;
  }

  const bool publisher_changed =
    latest_state_.empty() ||
    incoming_state.at("publisher_id") != latest_state_.at("publisher_id");

  if (publisher_changed) {
    const double wall_time = std::chrono::duration<double>(
      std::chrono::system_clock::now().time_since_epoch()).count();
    const double state_age = wall_time - incoming_state.at("wall_unix_sec").get<double>();

    if (!std::isfinite(state_age) || state_age < -0.1 || state_age > 0.25) {
      return std::nullopt;
    }
  }

  const bool is_repeated_publication =
    !latest_state_.empty() &&
    incoming_state.at("publisher_id") == latest_state_.at("publisher_id") &&
    incoming_state.at("publication") == latest_state_.at("publication");

  if (is_repeated_publication) {
    return std::nullopt;
  }

  bool frame_context_changed = latest_state_.empty();

  if (!frame_context_changed) {
    try {
      frame_context_changed =
        frame_context(incoming_state) != frame_context(latest_state_);
    } catch (const nlohmann::json::exception &) {
      frame_context_changed = true;
    }
  }

  if (frame_context_changed) {
    cancel_frame_request();
  }

  latest_state_ = std::move(incoming_state);
  last_state_received_ = Clock::now();

  return latest_state_;
}

const nlohmann::json & LocalSimClient::state() const
{
  return latest_state_;
}

bool LocalSimClient::fresh() const
{
  return !latest_state_.empty() && Clock::now() - last_state_received_ <= state_timeout;
}

bool LocalSimClient::frame_request_pending() const
{
  return !pending_frame_request_.empty();
}

void LocalSimClient::request_frame()
{
  if (frame_request_pending() || !fresh()) {
    return;
  }

  const auto request_context = frame_context(latest_state_);
  const std::string request_identity =
    frame_client_token_ + "-" + std::to_string(frame_request_count_++);

  pending_frame_request_ = request_identity;
  pending_frame_context_ = request_context;
  frame_requested_at_ = Clock::now();

  try {
    write_json(
      "request.json",
      {
        {"publisher_id", latest_state_.at("publisher_id")},
        {"request_id", request_identity}
      });
  } catch (...) {
    cancel_frame_request();
    throw;
  }
}

std::optional<SimFrame> LocalSimClient::poll_frame()
{
  if (!frame_request_pending()) {
    return std::nullopt;
  }

  if (Clock::now() - frame_requested_at_ > frame_timeout) {
    cancel_frame_request();
    throw std::runtime_error(
      "Frame timeout: start the simulator in graphical mode, not --headless");
  }

  auto frame = read_json("frame.json");

  try {
    if (!frame.contains("request_id") || !frame.at("request_id").is_string()) {
      return std::nullopt;
    }

    if (frame.at("request_id").get_ref<const std::string &>() != pending_frame_request_) {
      return std::nullopt;
    }

    if (!fresh() ||
        frame.value("protocol", std::string()) != "rm_aim_sim/2" ||
        frame.value("type", std::string()) != "frame" ||
        frame_context(frame) != pending_frame_context_ ||
        frame_context(frame) != frame_context(latest_state_)) {
      cancel_frame_request();
      return std::nullopt;
    }

    const double frame_sim_time = frame.at("sim_time").get<double>();
    const double state_sim_time = latest_state_.at("sim_time").get<double>();
    const double frame_age = state_sim_time - frame_sim_time;

    if (!std::isfinite(frame_sim_time) ||
        !std::isfinite(state_sim_time) ||
        !std::isfinite(frame_age) ||
        frame_age < -0.1 ||
        frame_age > 0.2) {
      cancel_frame_request();
      return std::nullopt;
    }

    const auto & camera = frame.at("camera");
    const double camera_sim_time = camera.at("timestamp").get<double>();

    if (!std::isfinite(camera_sim_time) ||
        std::abs(camera_sim_time - frame_sim_time) > 1e-6 ||
        camera.at("tick") != frame.at("tick")) {
      throw std::runtime_error("Frame camera metadata is not from the same snapshot");
    }

    const int width = bounded_frame_dimension(
      camera.at("width"),
      minimum_frame_width,
      maximum_frame_width,
      "frame.camera.width");

    const int height = bounded_frame_dimension(
      camera.at("height"),
      minimum_frame_height,
      maximum_frame_height,
      "frame.camera.height");

    const auto encoding = frame.at("encoding").get<std::string>();
    const int channel_count =
      encoding == "rgb8" ? 3 :
      encoding == "rgba8" ? 4 : 0;

    if (channel_count == 0) {
      throw std::runtime_error("Frame encoding must be rgb8 or rgba8");
    }

    const std::string expected_filename =
      "frame-" + pending_frame_request_ + ".raw";

    if (frame.at("frame_file").get<std::string>() != expected_filename) {
      throw std::runtime_error("Frame file name does not match its request");
    }

    const std::size_t expected_byte_count =
      static_cast<std::size_t>(width) *
      static_cast<std::size_t>(height) *
      static_cast<std::size_t>(channel_count);

    exact_byte_count(frame.at("image_bytes"), expected_byte_count);

    const int frame_descriptor = ::openat(
      directory_descriptor_,
      expected_filename.c_str(),
      O_RDONLY | O_CLOEXEC | O_NOFOLLOW);

    if (frame_descriptor < 0) {
      throw std::runtime_error(
        "Frame file disappeared; reconnect after a simulator restart");
    }

    cv::Mat rgb_image;

    try {
      struct stat file_information {};

      if (::fstat(frame_descriptor, &file_information) != 0 ||
          !S_ISREG(file_information.st_mode) ||
          file_information.st_size != static_cast<off_t>(expected_byte_count)) {
        throw std::runtime_error("Frame raw file size or type does not match its metadata");
      }

      rgb_image.create(
        height,
        width,
        CV_MAKETYPE(CV_8U, channel_count));

      read_exactly(frame_descriptor, rgb_image.data, expected_byte_count);
    } catch (...) {
      ::close(frame_descriptor);
      throw;
    }

    ::close(frame_descriptor);

    SimFrame result;
    cv::cvtColor(
      rgb_image,
      result.bgr,
      channel_count == 3 ? cv::COLOR_RGB2BGR : cv::COLOR_RGBA2BGR);

    result.metadata = std::move(frame);
    result.received_at = Clock::now();

    cancel_frame_request();
    return result;
  } catch (...) {
    cancel_frame_request();
    throw;
  }
}

void LocalSimClient::cancel_frame_request()
{
  pending_frame_request_.clear();
  pending_frame_context_ = nlohmann::json::array();
  frame_requested_at_ = Clock::time_point{};
}

std::optional<PublishedCommand> LocalSimClient::send(
  bool control,
  bool fire,
  double yaw,
  double pitch,
  double yaw_velocity,
  double pitch_velocity,
  double yaw_acceleration,
  double pitch_acceleration)
{
  if (latest_state_.empty()) {
    return std::nullopt;
  }

  if (latest_state_.at("paused").get<bool>() ||
      !latest_state_.at("ipc").at("enabled").get<bool>()) {
    return std::nullopt;
  }

  for (const double number : {
      yaw,
      pitch,
      yaw_velocity,
      pitch_velocity,
      yaw_acceleration,
      pitch_acceleration}) {
    if (!std::isfinite(number) || std::abs(number) > 1000.0) {
      throw std::runtime_error("Command contains an invalid angle or feedforward value");
    }
  }

  const auto publisher = latest_state_.at("publisher_id").get<std::string>();
  const auto session = latest_state_.at("session_id").get<std::string>();

  if (sequence_publisher_ != publisher || sequence_session_ != session) {
    sequence_publisher_ = publisher;
    sequence_session_ = session;
    next_sequence_ = 0;
  }

  const auto last_accepted_sequence = latest_state_.at("ipc").at("last_seq").get<std::int64_t>();
  next_sequence_ = std::max(next_sequence_, last_accepted_sequence + 1);

  if (next_sequence_ > maximum_sequence) {
    throw std::runtime_error("Command sequence exhausted");
  }

  const bool control_is_allowed = control && fresh();
  const bool fire_is_allowed = fire && control_is_allowed;
  const std::int64_t published_sequence = next_sequence_++;

  const nlohmann::json command = {
    {"protocol", "rm_aim_sim/2"},
    {"type", "command"},
    {"publisher_id", publisher},
    {"session_id", session},
    {"seq", published_sequence},
    {"control", control_is_allowed},
    {"fire", fire_is_allowed},
    {"yaw", yaw},
    {"pitch", pitch},
    {"yaw_vel", yaw_velocity},
    {"pitch_vel", pitch_velocity},
    {"yaw_acc", yaw_acceleration},
    {"pitch_acc", pitch_acceleration}
  };

  write_json("command.json", command);
  last_published_control_ = control_is_allowed;

  return PublishedCommand{published_sequence, control_is_allowed, fire_is_allowed};
}

bool LocalSimClient::has_valid_state_envelope(const nlohmann::json & state)
{
  try {
    if (
      state.at("protocol") != "rm_aim_sim/2" ||
      state.at("type") != "state" ||
      !state.at("publisher_id").is_string() ||
      state.at("publisher_id").get_ref<const std::string &>().empty() ||
      !state.at("session_id").is_string() ||
      state.at("session_id").get_ref<const std::string &>().empty() ||
      !is_protocol_integer(state.at("publication"), 0) ||
      !state.at("wall_unix_sec").is_number() ||
      !state.at("paused").is_boolean()) {
      return false;
    }

    const auto & ipc = state.at("ipc");

    return
      ipc.at("enabled").is_boolean() &&
      is_protocol_integer(ipc.at("last_seq"), -1);
  } catch (const nlohmann::json::exception &) {
    return false;
  }
}

nlohmann::json LocalSimClient::frame_context(const nlohmann::json & packet)
{
  const auto & camera = packet.at("camera");

  return nlohmann::json::array({
    packet.at("publisher_id"),
    packet.at("session_id"),
    packet.at("appearance"),
    packet.at("paused"),
    packet.at("ipc").at("enabled"),
    packet.at("weapon"),
    packet.at("gimbal").at("origin"),
    camera.at("width"),
    camera.at("height"),
    camera.at("brightness_gain"),
    camera.at("K"),
    camera.at("distortion"),
    camera.at("R_camera2gimbal"),
    camera.at("t_camera2gimbal")
  });
}

void LocalSimClient::read_exactly(
  int descriptor,
  void * destination,
  std::size_t byte_count)
{
  auto * destination_bytes = static_cast<char *>(destination);

  while (byte_count > 0) {
    const auto bytes_read = ::read(descriptor, destination_bytes, byte_count);

    if (bytes_read < 0 && errno == EINTR) {
      continue;
    }

    if (bytes_read <= 0) {
      throw std::runtime_error("Incomplete IPC file read");
    }

    destination_bytes += bytes_read;
    byte_count -= static_cast<std::size_t>(bytes_read);
  }
}

nlohmann::json LocalSimClient::read_json(const char * filename) const
{
  const int descriptor = ::openat(
    directory_descriptor_,
    filename,
    O_RDONLY | O_CLOEXEC | O_NOFOLLOW);

  if (descriptor < 0) {
    return nlohmann::json::object();
  }

  std::string serialized_json;

  try {
    struct stat file_information {};

    if (
      ::fstat(descriptor, &file_information) != 0 ||
      !S_ISREG(file_information.st_mode) ||
      file_information.st_size < 0 ||
      file_information.st_size > maximum_json_bytes) {
      throw std::runtime_error("Invalid IPC JSON file size or type");
    }

    serialized_json.resize(static_cast<std::size_t>(file_information.st_size));
    read_exactly(descriptor, serialized_json.data(), serialized_json.size());
  } catch (...) {
    ::close(descriptor);
    throw;
  }

  ::close(descriptor);

  auto parsed_json = nlohmann::json::parse(serialized_json, nullptr, false);

  if (!parsed_json.is_object()) {
    return nlohmann::json::object();
  }

  return parsed_json;
}

void LocalSimClient::write_json(
  const char * filename,
  const nlohmann::json & value) const
{
  const std::string temporary_filename = std::string(filename) + ".client.tmp";
  const int descriptor = ::openat(
    directory_descriptor_,
    temporary_filename.c_str(),
    O_CREAT | O_TRUNC | O_WRONLY | O_CLOEXEC | O_NOFOLLOW,
    0600);

  if (descriptor < 0) {
    throw std::runtime_error("Cannot create IPC JSON temporary file");
  }

  try {
    const std::string serialized_json = value.dump();
    std::size_t bytes_written = 0;

    while (bytes_written < serialized_json.size()) {
      const auto written_count = ::write(
        descriptor,
        serialized_json.data() + bytes_written,
        serialized_json.size() - bytes_written);

      if (written_count < 0 && errno == EINTR) {
        continue;
      }

      if (written_count <= 0) {
        throw std::runtime_error("Incomplete command file write");
      }

      bytes_written += static_cast<std::size_t>(written_count);
    }
  } catch (...) {
    ::close(descriptor);
    ::unlinkat(directory_descriptor_, temporary_filename.c_str(), 0);
    throw;
  }

  ::close(descriptor);

  const int rename_status = ::renameat(
    directory_descriptor_,
    temporary_filename.c_str(),
    directory_descriptor_,
    filename);

  if (rename_status != 0) {
    ::unlinkat(directory_descriptor_, temporary_filename.c_str(), 0);
    throw std::runtime_error("Cannot atomically publish IPC JSON");
  }
}

}  // namespace auto_aim_bridge
