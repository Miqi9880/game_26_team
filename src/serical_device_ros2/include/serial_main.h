#ifndef SERIAL_MAIN_NEW_H
#define SERIAL_MAIN_NEW_H

#include "protocol.h"
#include <cstdint>
#include <memory>
#include <mutex>
#include <string>
#include <thread>
#include <vector>

class SerialDevice;

/// Thread-safe serial bridge (process-wide singleton).
/// - RX: dedicated thread parses protocol_new frames into latest VisionData.
/// - TX: send_robot_ctrl() writes a downlink RobotCtrlData frame.
/// Use one process (component container) for all nodes so the port is opened once.
class SerialMain
{
public:
  static SerialMain & instance();

  bool init(const std::string & device_path = "/dev/robomaster", int baud = 921600);
  bool ok() const;

  void send_robot_ctrl(const RobotCtrlData & d);

  /// Copy the latest received VisionData (return false if none yet).
  bool get_vision(VisionData & out) const;

  uint64_t tx_count() const;
  uint64_t rx_count() const;
  uint64_t err_count() const;
  /// Monotonic counter of parsed VISION frames (to dedupe publishes).
  uint64_t vision_seq() const;

  void stop();
  ~SerialMain();

private:
  SerialMain() = default;
  SerialMain(const SerialMain &) = delete;
  SerialMain & operator=(const SerialMain &) = delete;

  void rx_loop();
  void feed(const uint8_t * p, size_t n);
  void try_parse();

  std::string device_path_ = "/dev/robomaster";
  int baud_ = 921600;
  std::shared_ptr<SerialDevice> dev_;
  bool ok_ = false;
  std::thread rx_thread_;

  mutable std::mutex mtx_;
  VisionData latest_vision_;
  bool have_vision_ = false;
  std::vector<uint8_t> buf_;
  uint8_t seq_ = 0;
  uint64_t tx_count_ = 0;
  uint64_t rx_count_ = 0;
  uint64_t err_count_ = 0;
  uint64_t vision_seq_ = 0;
};

#endif  // SERIAL_MAIN_NEW_H