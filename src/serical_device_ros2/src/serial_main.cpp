#include "serial_main.h"
#include "serial_device.h"
#include "crc.h"

#include <cstring>
#include <iostream>
#include <unistd.h>

SerialMain & SerialMain::instance()
{
  static SerialMain s;
  return s;
}

bool SerialMain::init(const std::string & device_path, int baud)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (ok_) { return true; }
  device_path_ = device_path;
  baud_ = baud;
  dev_ = std::make_shared<SerialDevice>(device_path_, baud_);
  if (!dev_->Init()) {
    std::cerr << "[serial_main] failed to open " << device_path_ << " @ " << baud_ << std::endl;
    dev_.reset();
    return false;
  }
  buf_.reserve(2048);
  ok_ = true;
  rx_thread_ = std::thread(&SerialMain::rx_loop, this);
  std::cout << "[serial_main] serial up: " << device_path_ << " @ " << baud_ << std::endl;
  return true;
}

bool SerialMain::ok() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return ok_;
}

void SerialMain::stop()
{
  {
    std::lock_guard<std::mutex> lock(mtx_);
    ok_ = false;
  }
  if (rx_thread_.joinable()) { rx_thread_.join(); }
}

SerialMain::~SerialMain()
{
  stop();
}

void SerialMain::send_robot_ctrl(const RobotCtrlData & d)
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!ok_ || !dev_) { return; }
  constexpr int payload = static_cast<int>(sizeof(RobotCtrlData));
  constexpr int total = FRAME_TOTAL(payload);
  uint8_t frame[total] = {0};
  frame[0] = HEADER_SOF;
  frame[1] = static_cast<uint8_t>(payload & 0xFF);
  frame[2] = static_cast<uint8_t>((payload >> 8) & 0xFF);
  frame[3] = seq_++;
  Append_CRC8_Check_Sum(frame, HEADER_LEN);
  frame[HEADER_LEN] = static_cast<uint8_t>(CHASSIS_CTRL_CMD_ID & 0xFF);
  frame[HEADER_LEN + 1] = static_cast<uint8_t>((CHASSIS_CTRL_CMD_ID >> 8) & 0xFF);
  std::memcpy(frame + HEADER_LEN + CMD_LEN, &d, sizeof(d));
  // CRC16 covers header+cmd+payload: length = payload + HEADER_LEN + CMD_LEN + CRC16_LEN
  Append_CRC16_Check_Sum(frame, payload + HEADER_LEN + CMD_LEN + CRC16_LEN);
  frame[total - 2] = END1_SOF;
  frame[total - 1] = END2_SOF;
  dev_->Write(frame, total);
  ++tx_count_;
}

bool SerialMain::get_vision(VisionData & out) const
{
  std::lock_guard<std::mutex> lock(mtx_);
  if (!have_vision_) { return false; }
  out = latest_vision_;
  return true;
}

uint64_t SerialMain::tx_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return tx_count_;
}
uint64_t SerialMain::rx_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return rx_count_;
}
uint64_t SerialMain::vision_seq() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return vision_seq_;
}
uint64_t SerialMain::err_count() const
{
  std::lock_guard<std::mutex> lock(mtx_);
  return err_count_;
}

void SerialMain::rx_loop()
{
  uint8_t chunk[512];
  while (true) {
    {
      std::lock_guard<std::mutex> lock(mtx_);
      if (!ok_) { break; }
    }
    const int n = dev_->Read(chunk, sizeof(chunk));
    if (n > 0) {
      feed(chunk, static_cast<size_t>(n));
    } else if (n == 0) {
      usleep(2000);
    } else {
      usleep(5000);
    }
  }
}

void SerialMain::feed(const uint8_t * p, size_t n)
{
  std::lock_guard<std::mutex> lock(mtx_);
  rx_count_ += n;
  buf_.insert(buf_.end(), p, p + n);
  try_parse();
}

void SerialMain::try_parse()
{
  while (buf_.size() >= HEADER_LEN) {
    // resync on SOF
    size_t sof = 0;
    while (sof < buf_.size() && buf_[sof] != HEADER_SOF) { ++sof; }
    if (sof > 0) { buf_.erase(buf_.begin(), buf_.begin() + sof); }
    if (buf_.size() < HEADER_LEN) { break; }

    const uint16_t payload = static_cast<uint16_t>(buf_[1]) |
                             (static_cast<uint16_t>(buf_[2]) << 8);
    const size_t total = static_cast<size_t>(FRAME_TOTAL(payload));
    if (buf_.size() < total) { break; }  // wait for more bytes

    if (!Verify_CRC8_Check_Sum(buf_.data(), HEADER_LEN) ||
        !Verify_CRC16_Check_Sum(buf_.data(), payload + HEADER_LEN + CMD_LEN + CRC16_LEN))
    {
      ++err_count_;
      buf_.erase(buf_.begin(), buf_.begin() + HEADER_LEN);
      continue;
    }
    if (buf_[total - 2] != END1_SOF || buf_[total - 1] != END2_SOF) {
      ++err_count_;
      buf_.erase(buf_.begin(), buf_.begin() + HEADER_LEN);
      continue;
    }

    const uint16_t cmd = static_cast<uint16_t>(buf_[HEADER_LEN]) |
                         (static_cast<uint16_t>(buf_[HEADER_LEN + 1]) << 8);
    // MCU currently sends 46 B (protocol_new VisionData minus trailing u8 game_progress).
    // Accept payload <= full size and zero-fill the missing tail bytes.
    if (cmd == VISION_ID && payload <= sizeof(VisionData) && payload >= 44) {
      VisionData tmp{};
      std::memcpy(&tmp, buf_.data() + HEADER_LEN + CMD_LEN, payload);
      latest_vision_ = tmp;
      have_vision_ = true;
      ++vision_seq_;
    }
    buf_.erase(buf_.begin(), buf_.begin() + total);
  }
  if (buf_.size() > 4096) { buf_.clear(); ++err_count_; }
}