/****************************************************************************
 *  Copyright (C) 2019 RoboMaster.
 *
 *  This program is free software: you can redistribute it and/or modify
 *  it under the terms of the GNU General Public License as published by
 *  the Free Software Foundation, either version 3 of the License, or
 *  (at your option) any later version.
 *
 *  This program is distributed in the hope that it will be useful,
 *  but WITHOUT ANY WARRANTY; without even the implied warranty of 
 *  MERCHANTABILITY or FITNESS FOR A PARTICULAR PURPOSE.  See the
 *  GNU General Public License for more details.
 *
 *  You should have received a copy of the GNU General Public License
 *  along with this program. If not, see <http://www.gnu.org/licenses/>.
 ***************************************************************************/

#include "serial_device.h"
#include <cerrno>
#include <iostream>
#include <poll.h>

//namespace robomaster {
SerialDevice::SerialDevice(std::string port_name,
						   int baudrate) :
		port_name_(port_name),
		baudrate_(baudrate),
		data_bits_(8),
		parity_bits_('N'),
		stop_bits_(1) {}

SerialDevice::~SerialDevice() {
	CloseDevice();
}

// 初始化
bool SerialDevice::Init() {
	
	std::cout << "Attempting to open device " << port_name_
			  << " with baudrate " << baudrate_ << std::endl;
	if (port_name_.empty()) {
		port_name_ = "/dev/ttyUSB0";
	}
	if (OpenDevice() && ConfigDevice()) {
		FD_ZERO(&serial_fd_set_);
		FD_SET(serial_fd_, &serial_fd_set_);
		std::cout << "Serial started successfully." << std::endl;
		return true;
	} else {
		std::cerr << "Failed to start serial " << port_name_ << std::endl;
		CloseDevice();
		return false;
	}
}

// 打开设备
bool SerialDevice::OpenDevice() {

#ifdef __arm__
	serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NONBLOCK);
#elif __x86_64__
	serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#else
	serial_fd_ = open(port_name_.c_str(), O_RDWR | O_NOCTTY);
#endif
	
	if (serial_fd_ < 0) {
		std::cerr << "Cannot open device "
				  << port_name_ << std::endl;
		return false;
	}
	
	return true;
}

// 关闭通讯协议接口
bool SerialDevice::CloseDevice() {
	if (serial_fd_ >= 0) {
		close(serial_fd_);
	}
	serial_fd_ = -1;
	return true;
}

// 配置设备
bool SerialDevice::ConfigDevice() {
	int st_baud[] = {B4800, B9600, B19200, B38400,
					 B57600, B115200, B230400, B921600};
	int std_rate[] = {4800, 9600, 19200, 38400, 57600, 115200,
					  230400, 921600, 1000000, 1152000, 3000000};
	int i, j;
	/* save current port parameter */
	if (tcgetattr(serial_fd_, &old_termios_) != 0) {
		std::cerr << "fail to save current port" << std::endl;
		return false;
	}
	memset(&new_termios_, 0, sizeof(new_termios_));
	
	/* config the size of char */
	new_termios_.c_cflag |= CLOCAL | CREAD;
	new_termios_.c_cflag &= ~CSIZE;
	
	/* config data bit */
	switch (data_bits_) {
		case 7:
			new_termios_.c_cflag |= CS7;
			break;
		case 8:
			new_termios_.c_cflag |= CS8;
			break;
		default:
			new_termios_.c_cflag |= CS8;
			break; //8N1 default config
	}
	/* config the parity bit */
	switch (parity_bits_) {
		/* odd */
		case 'O':
		case 'o':
			new_termios_.c_cflag |= PARENB;
			new_termios_.c_cflag |= PARODD;
			break;
			/* even */
		case 'E':
		case 'e':
			new_termios_.c_cflag |= PARENB;
			new_termios_.c_cflag &= ~PARODD;
			break;
			/* none */
		case 'N':
		case 'n':
			new_termios_.c_cflag &= ~PARENB;
			break;
		default:
			new_termios_.c_cflag &= ~PARENB;
			break; //8N1 default config
	}
	/* config baudrate */
	j = sizeof(std_rate) / 4;
	for (i = 0; i < j; ++i) {
		if (std_rate[i] == baudrate_) {
			/* set standard baudrate */
			cfsetispeed(&new_termios_, st_baud[i]);
			cfsetospeed(&new_termios_, st_baud[i]);
			break;
		}
	}
	/* config stop bit */
	if (stop_bits_ == 1)
		new_termios_.c_cflag &= ~CSTOPB;
	else if (stop_bits_ == 2)
		new_termios_.c_cflag |= CSTOPB;
	else
		new_termios_.c_cflag &= ~CSTOPB; //8N1 default config

	/* ReadSome() uses poll() for readiness; read must return the available chunk. */
	new_termios_.c_cc[VTIME] = 0;
	new_termios_.c_cc[VMIN] = 0;
	
	/* using the raw data mode */
	new_termios_.c_lflag &= ~(ICANON | ECHO | ECHOE | ISIG);
	new_termios_.c_oflag &= ~OPOST;
	
	/* flush the hardware fifo */
	tcflush(serial_fd_, TCIFLUSH);
	
	/* activite the configuration */
	if ((tcsetattr(serial_fd_, TCSANOW, &new_termios_)) != 0) {
		std::cerr << "failed to activate serial configuration" << std::endl;
		return false;
	}
	return true;
	
}

int SerialDevice::Read(uint8_t *buf, int len) {
	if (len <= 0) {
		return 0;
	}
	return ReadSome(buf, static_cast<std::size_t>(len));
}

int SerialDevice::ReadSome(uint8_t *buf, std::size_t len) {
	if (serial_fd_ < 0 || buf == nullptr || len == 0) {
		return -1;
	}

	pollfd descriptor{serial_fd_, POLLIN, 0};
	int ready = poll(&descriptor, 1, 0);
	if (ready == 0) {
		return 0;
	}
	if (ready < 0) {
		return errno == EINTR ? 0 : -1;
	}
	if ((descriptor.revents & (POLLERR | POLLHUP | POLLNVAL)) != 0) {
		return -1;
	}

	ssize_t ret = read(serial_fd_, buf, len);
	if (ret > 0) {
		return static_cast<int>(ret);
	}
	if (ret < 0 && (errno == EINTR || errno == EAGAIN || errno == EWOULDBLOCK)) {
		return 0;
	}
	return ret == 0 ? 0 : -1;
}

int SerialDevice::ReadUntil2(uint8_t *buf, uint8_t end1, uint8_t end2, uint8_t max_len) {
	if (buf == nullptr || max_len == 0) {
		return -1;
	}

	const int ret = ReadSome(buf, max_len);
	if (ret <= 0) {
		return ret;
	}
	for (int i = 1; i < ret; ++i) {
		if (buf[i - 1] == end1 && buf[i] == end2) {
			return 1;
		}
	}
	return 0;
}

// 发送数据函数
int SerialDevice::Write(const uint8_t *buf, int len) {
	if (serial_fd_ < 0 || buf == nullptr || len <= 0) {
		return -1;
	}

	std::size_t written = 0;
	while (written < static_cast<std::size_t>(len)) {
		const ssize_t ret = write(serial_fd_, buf + written, static_cast<std::size_t>(len) - written);
		if (ret > 0) {
			written += static_cast<std::size_t>(ret);
			continue;
		}
		if (ret < 0 && errno == EINTR) {
			continue;
		}
		if (ret < 0 && (errno == EAGAIN || errno == EWOULDBLOCK)) {
			pollfd descriptor{serial_fd_, POLLOUT, 0};
			const int ready = poll(&descriptor, 1, 100);
			if (ready > 0 && (descriptor.revents & POLLOUT) != 0) {
				continue;
			}
		}
		break;
	}
	return written == 0 ? -1 : static_cast<int>(written);
}
//}
