#include "karmel_hardware/serial_port.hpp"

#include <fcntl.h>
#include <termios.h>
#include <unistd.h>

#include <cerrno>
#include <cstring>

namespace karmel_hardware
{

namespace
{
speed_t to_speed(int baud)
{
  switch (baud) {
    case 9600: return B9600;
    case 19200: return B19200;
    case 38400: return B38400;
    case 57600: return B57600;
    case 115200: return B115200;
    case 230400: return B230400;
    case 460800: return B460800;
    case 921600: return B921600;
    default: return 0;
  }
}
}  // namespace

SerialPort::~SerialPort()
{
  close();
}

bool SerialPort::open(const std::string & device, int baud)
{
  close();
  const speed_t speed = to_speed(baud);
  if (speed == 0) {
    error_ = "unsupported baud rate " + std::to_string(baud);
    return false;
  }

  // O_NOCTTY: this port must not become our controlling terminal.
  // O_NONBLOCK: read() returns immediately; ros2_control's read() must never stall the loop.
  fd_ = ::open(device.c_str(), O_RDWR | O_NOCTTY | O_NONBLOCK);
  if (fd_ < 0) {
    error_ = "open " + device + ": " + std::strerror(errno);
    return false;
  }

  termios tty{};
  if (tcgetattr(fd_, &tty) != 0) {
    error_ = std::string("tcgetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  // Raw mode: no echo, no line editing, no CR/LF translation, 8 data bits, no parity, 1 stop bit.
  cfmakeraw(&tty);
  tty.c_cflag |= (CLOCAL | CREAD);
  tty.c_cflag &= ~CRTSCTS;
  cfsetispeed(&tty, speed);
  cfsetospeed(&tty, speed);
  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 0;
  if (tcsetattr(fd_, TCSANOW, &tty) != 0) {
    error_ = std::string("tcsetattr: ") + std::strerror(errno);
    close();
    return false;
  }
  tcflush(fd_, TCIOFLUSH);  // drop stale bytes from before we opened the port
  return true;
}

void SerialPort::close()
{
  if (fd_ >= 0) {
    ::close(fd_);
    fd_ = -1;
  }
}

bool SerialPort::write(const std::string & data)
{
  if (fd_ < 0) {
    return false;
  }
  size_t written = 0;
  int retries = 0;
  while (written < data.size()) {
    const ssize_t n = ::write(fd_, data.data() + written, data.size() - written);
    if (n > 0) {
      written += static_cast<size_t>(n);
    } else if (n < 0 && (errno == EAGAIN || errno == EWOULDBLOCK) && retries++ < 10) {
      usleep(500);  // output buffer full: give the USB stack half a millisecond
    } else {
      error_ = std::string("write: ") + (n < 0 ? std::strerror(errno) : "no progress");
      return false;
    }
  }
  return true;
}

bool SerialPort::read_available(std::string & out)
{
  if (fd_ < 0) {
    return false;
  }
  char buffer[512];
  while (true) {
    const ssize_t n = ::read(fd_, buffer, sizeof(buffer));
    if (n > 0) {
      out.append(buffer, static_cast<size_t>(n));
    } else if (n == 0 || errno == EAGAIN || errno == EWOULDBLOCK) {
      return true;  // nothing more right now
    } else {
      error_ = std::string("read: ") + std::strerror(errno);
      return false;
    }
  }
}

}  // namespace karmel_hardware
