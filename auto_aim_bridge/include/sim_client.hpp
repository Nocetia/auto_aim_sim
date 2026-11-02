// MIT License. See ../../LICENSE.
#pragma once

#include <nlohmann/json.hpp>
#include <opencv2/core.hpp>

#include <chrono>
#include <cstddef>
#include <cstdint>
#include <optional>
#include <string>

namespace auto_aim_bridge
{

struct PublishedCommand
{
  std::int64_t sequence = -1;
  bool control = false;
  bool fire = false;
};

struct SimFrame
{
  cv::Mat bgr;
  nlohmann::json metadata;
  std::chrono::steady_clock::time_point received_at;
};

// One thread owns the client and all references returned by state().
// The protocol is a latest-value mailbox, so skipped publications are normal.
class LocalSimClient
{
public:
  explicit LocalSimClient(const std::string & channel);
  ~LocalSimClient();

  LocalSimClient(const LocalSimClient &) = delete;
  LocalSimClient & operator=(const LocalSimClient &) = delete;

  // Repeated, stale, missing, or malformed states do not refresh freshness.
  std::optional<nlohmann::json> poll_state();

  const nlohmann::json & state() const;
  bool fresh() const;

  // The frame mailbox has one request in flight. A completed frame owns its
  // BGR pixels, so the simulator may safely remove the raw file afterwards.
  bool frame_request_pending() const;
  void request_frame();
  std::optional<SimFrame> poll_frame();
  void cancel_frame_request();

  // Angles are absolute radians. Pitch is positive downward.
  // Firing is a continuous permission level, not a one-shot pulse. The client
  // always suppresses it unless fresh active control is also being published.
  // A paused or disabled simulator rejects every command, so no file is
  // published in that state. The return value identifies the exact write.
  std::optional<PublishedCommand> send(
    bool control,
    bool fire,
    double yaw,
    double pitch,
    double yaw_velocity = 0.0,
    double pitch_velocity = 0.0,
    double yaw_acceleration = 0.0,
    double pitch_acceleration = 0.0);

private:
  using Clock = std::chrono::steady_clock;

  static bool has_valid_state_envelope(const nlohmann::json & state);
  static nlohmann::json frame_context(const nlohmann::json & packet);
  static void read_exactly(int descriptor, void * destination, std::size_t byte_count);

  nlohmann::json read_json(const char * filename) const;
  void write_json(const char * filename, const nlohmann::json & value) const;

  int directory_descriptor_ = -1;
  int lock_descriptor_ = -1;

  nlohmann::json latest_state_ = nlohmann::json::object();
  Clock::time_point last_state_received_{};

  std::string frame_client_token_;
  std::string pending_frame_request_;
  nlohmann::json pending_frame_context_ = nlohmann::json::array();
  Clock::time_point frame_requested_at_{};
  std::uint64_t frame_request_count_ = 0;

  std::string sequence_publisher_;
  std::string sequence_session_;
  std::int64_t next_sequence_ = 0;
  bool last_published_control_ = false;
};

}  // namespace auto_aim_bridge
