#ifndef ROBOMASTER_ROBOT_H
#define ROBOMASTER_ROBOT_H

#include <iostream>
#include <thread>
#include <utility>
#include <cstddef>
#include "serial_device.h"
#include "serial_frame_parser.h"
#include "protocol_new.hpp"
#include "crc.h"
#include <memory> 

static_assert(sizeof(io::FrameHeader) == 5, "Unexpected protocol frame header size");
static_assert(sizeof(io::VisionData) == 47, "Unexpected protocol vision payload size");
static_assert(sizeof(io::RobotCtrlData) == 26, "Unexpected protocol control payload size");
static_assert(sizeof(io::MsgEndInfo) == 2, "Unexpected protocol frame tail size");

static_assert(offsetof(io::FrameHeader, sof) == 0, "Unexpected frame header sof offset");
static_assert(offsetof(io::FrameHeader, data_length) == 1, "Unexpected frame header length offset");
static_assert(offsetof(io::FrameHeader, seq) == 3, "Unexpected frame header sequence offset");
static_assert(offsetof(io::FrameHeader, crc8) == 4, "Unexpected frame header CRC8 offset");

static_assert(offsetof(io::VisionData, id) == 0, "Unexpected vision id offset");
static_assert(offsetof(io::VisionData, mode) == 2, "Unexpected vision mode offset");
static_assert(offsetof(io::VisionData, yaw) == 4, "Unexpected vision yaw offset");
static_assert(offsetof(io::VisionData, yaw_vel) == 8, "Unexpected vision yaw velocity offset");
static_assert(offsetof(io::VisionData, pitch) == 12, "Unexpected vision pitch offset");
static_assert(offsetof(io::VisionData, pitch_vel) == 16, "Unexpected vision pitch velocity offset");
static_assert(offsetof(io::VisionData, roll) == 20, "Unexpected vision roll offset");
static_assert(offsetof(io::VisionData, quaternion) == 24, "Unexpected vision quaternion offset");
static_assert(offsetof(io::VisionData, shoot_speed) == 40, "Unexpected vision shoot speed offset");
static_assert(offsetof(io::VisionData, bullet_count) == 44, "Unexpected vision bullet count offset");
static_assert(offsetof(io::VisionData, game_progress) == 46, "Unexpected vision game progress offset");

static_assert(offsetof(io::RobotCtrlData, yaw) == 0, "Unexpected control yaw offset");
static_assert(offsetof(io::RobotCtrlData, yaw_vel) == 4, "Unexpected control yaw velocity offset");
static_assert(offsetof(io::RobotCtrlData, yaw_acc) == 8, "Unexpected control yaw acceleration offset");
static_assert(offsetof(io::RobotCtrlData, pitch) == 12, "Unexpected control pitch offset");
static_assert(offsetof(io::RobotCtrlData, pitch_vel) == 16, "Unexpected control pitch velocity offset");
static_assert(offsetof(io::RobotCtrlData, pitch_acc) == 20, "Unexpected control pitch acceleration offset");
static_assert(offsetof(io::RobotCtrlData, target_lock) == 24, "Unexpected control target lock offset");
static_assert(offsetof(io::RobotCtrlData, fire_command) == 25, "Unexpected control fire command offset");

static_assert(offsetof(io::MsgEndInfo, end1) == 0, "Unexpected frame tail first byte offset");
static_assert(offsetof(io::MsgEndInfo, end2) == 1, "Unexpected frame tail second byte offset");

class SerialMain {
public:
	// Hardware access is opt-in so a default-constructed node is safe for dry-runs.
	SerialMain(std::string device_path = "/dev/robomaster", bool auto_init = false);
	
	~SerialMain() = default;
	
	void SenderMain(const io::RobotCtrlData &control);           // 发送新版控制数据
	
	bool CommInit();
	bool Enable();
	void SetDevicePath(std::string device_path);
	bool IsEnabled() const noexcept { return serial_enabled_; }
	
	bool ReceiverMain();                                        // 读取数据
	
	void SearchFrameSOF(uint8_t *frame, uint16_t total_len);
	
	// Legacy wrapper for callers that own a full receive buffer.
	uint16_t ReceiveDataSolve(const uint8_t *frame);
	// Parse exactly one frame from a buffer with a known available length.
	uint16_t ReceiveDataSolve(const uint8_t *frame, std::size_t frame_length);
	
	uint16_t SenderPackSolve(uint8_t *data, uint16_t data_length,
							 uint16_t cmd_id, uint8_t *send_buf);
	io::VisionData vision_msg_{};

private:
	
	//! Device Information and Buffer Allocation
	std::string device_path_;
	std::shared_ptr<SerialDevice> device_ptr_;
	std::unique_ptr<uint8_t[]> recv_buff_;
	std::unique_ptr<uint8_t[]> send_buff_;
	const unsigned int BUFF_LENGTH = 512;
	
	//! Frame Information
	io::FrameHeader frame_receive_header_{};
	io::FrameHeader frame_send_header_{};
	
	/** @brief specific protocol data are defined here
	 *         payload structures are defined in protocol_new.hpp
	 */
	
	io::RobotCtrlData robot_ctrl{};
	SerialFrameParser frame_parser_{};
	bool serial_enabled_{false};
};
//}

#endif // ROBOMASTER_ROBOT_H
