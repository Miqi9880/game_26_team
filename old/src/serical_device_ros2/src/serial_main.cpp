#include "serial_main.h"

#include <array>
#include <cstddef>
#include <cstring>

SerialMain::SerialMain(std::string device_path, bool auto_init) : device_path_(std::move(device_path))
{
	if (auto_init && !Enable())
	{
		std::cout<<"serial init error!!!!!!!!!!"<<std::endl;
	};
}

void SerialMain::SetDevicePath(std::string device_path)
{
	if (!serial_enabled_)
	{
		device_path_ = std::move(device_path);
	}
}

bool SerialMain::Enable()
{
	if (serial_enabled_)
	{
		return true;
	}
	return CommInit();
}

void SerialMain::SenderMain(const io::RobotCtrlData &control)
{
	robot_ctrl = control;
	if (!serial_enabled_ || !device_ptr_ || !send_buff_)
	{
		return;
	}

	uint16_t send_length = SenderPackSolve(
		reinterpret_cast<uint8_t *>(&robot_ctrl), sizeof(io::RobotCtrlData),
		io::CHASSIS_CTRL_CMD_ID, send_buff_.get());
	if (send_length == 0 || device_ptr_->Write(send_buff_.get(), send_length) != send_length)
	{
		std::cerr << "Serial control frame write failed or was incomplete" << std::endl;
	}
}

bool SerialMain::CommInit()
{
	if (serial_enabled_)
	{
		return true;
	}
	
	// Historical compatibility/default baud rate from the existing serial path.
	// The actual vehicle baud rate is not confirmed by the current electrical
	// control contract; this value is not production validation.
	device_ptr_ = std::make_shared<SerialDevice>(device_path_, 115200);
	
	if (!device_ptr_->Init())
	{
		device_ptr_.reset();
		return false;
	}
	
	recv_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	send_buff_ = std::unique_ptr<uint8_t[]>(new uint8_t[BUFF_LENGTH]);
	
	frame_receive_header_ = io::FrameHeader{};
	frame_send_header_ = io::FrameHeader{};
	frame_parser_.Reset();
	serial_enabled_ = true;
	
	return true;
}

bool SerialMain::ReceiverMain()
{
	if (!serial_enabled_ || !device_ptr_ || !recv_buff_)
	{
		return false;
	}

	std::array<uint8_t, 128> read_buffer{};
	const int read_length = device_ptr_->ReadSome(read_buffer.data(), read_buffer.size());
	if (read_length <= 0)
	{
		return false;
	}

	bool got_vision = false;
	frame_parser_.Feed(
		read_buffer.data(), static_cast<std::size_t>(read_length),
		[this, &got_vision](const uint8_t *frame, std::size_t frame_length) {
			if (ReceiveDataSolve(frame, frame_length) != 0)
			{
				got_vision = true;
			}
			return true;
		});
	return got_vision;
}

void SerialMain::SearchFrameSOF(uint8_t *frame, uint16_t total_len)
{
	uint16_t i;
	// uint16_t index = 0;
	// int a = 0;
	
//	std::cout<<total_len<<std::endl;
	for (i = 0; i < total_len;)
	{
		if (*frame == io::HEADER_SOF)
		{
			ReceiveDataSolve(frame);
			i = total_len;
		}
		else
		{
			frame++;
			i++;
		}
	}
}

uint16_t SerialMain::ReceiveDataSolve(const uint8_t *frame)
{
	return ReceiveDataSolve(frame, BUFF_LENGTH);
}

uint16_t SerialMain::ReceiveDataSolve(const uint8_t *frame, std::size_t frame_length)
{
	if (frame == nullptr || frame_length < sizeof(io::FrameHeader) + 2 + 2 + sizeof(io::MsgEndInfo) ||
		*frame != io::HEADER_SOF)
	{
		return 0;
	}

	io::FrameHeader header{};
	std::memcpy(&header, frame, sizeof(header));
	if (header.data_length > BUFF_LENGTH - 11)
	{
		std::cout << "Protocol payload exceeds receive buffer: "
		          << header.data_length << std::endl;
		return 0;
	}

	const std::size_t expected_length = static_cast<std::size_t>(header.data_length) + 11;
	if (frame_length < expected_length)
	{
		return 0;
	}

	if (!Verify_CRC8_Check_Sum(const_cast<uint8_t *>(frame), sizeof(io::FrameHeader)))
	{
		std::cout << "CRC8 error!!" << std::endl;
		return 0;
	}

	const std::size_t tail_offset = expected_length - sizeof(io::MsgEndInfo);
	if (frame[tail_offset] != io::END1_SOF || frame[tail_offset + 1] != io::END2_SOF)
	{
		std::cout << "Frame tail error!!" << std::endl;
		return 0;
	}

	if (!Verify_CRC16_Check_Sum(
		const_cast<uint8_t *>(frame), static_cast<uint32_t>(header.data_length + 9)))
	{
		std::cout << "CRC16 error!!" << std::endl;
		return 0;
	}

	uint16_t cmd_id = 0;
	std::memcpy(&cmd_id, frame + sizeof(io::FrameHeader), sizeof(cmd_id));
	const std::size_t payload_offset = sizeof(io::FrameHeader) + sizeof(cmd_id);
	if (cmd_id != io::VISION_ID || header.data_length != sizeof(io::VisionData))
	{
		return 0;
	}

	std::memcpy(&vision_msg_, frame + payload_offset, sizeof(io::VisionData));
	frame_receive_header_ = header;
	return static_cast<uint16_t>(expected_length);
}

uint16_t SerialMain::SenderPackSolve(uint8_t *data, uint16_t data_length,
									 uint16_t cmd_id, uint8_t *send_buf)
{
	if (data == nullptr || send_buf == nullptr || data_length > BUFF_LENGTH - 11)
	{
		return 0;
	}

	uint16_t index = 0;
	frame_send_header_.sof = io::HEADER_SOF;
	frame_send_header_.data_length = data_length;
	frame_send_header_.seq++;
	
	Append_CRC8_Check_Sum(reinterpret_cast<uint8_t *>(&frame_send_header_), sizeof(io::FrameHeader));
	
	memcpy(send_buf, &frame_send_header_, sizeof(io::FrameHeader));//assign frame header
	
	index += sizeof(io::FrameHeader);
	
	memcpy(send_buf + index, &cmd_id, sizeof(uint16_t));//assign cmd
	
	index += sizeof(uint16_t);
	
	memcpy(send_buf + index, data, data_length);//assign data
	
	// The MCU-confirmed upper-computer -> lower-computer 0x0102 control
	// frame ends immediately after CRC16.  Other packet layouts retain their
	// existing tail so the Vision receive protocol remains unchanged.
	Append_CRC16_Check_Sum(send_buf, data_length + 9);

	if (cmd_id != io::CHASSIS_CTRL_CMD_ID)
	{
		const io::MsgEndInfo frame_end{};
		memcpy(send_buf + data_length + 9, &frame_end, sizeof(frame_end));
		return data_length + 9 + sizeof(frame_end);
	}

	return data_length + 9;
}
