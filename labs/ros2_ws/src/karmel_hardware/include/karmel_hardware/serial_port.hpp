// A minimal POSIX serial port (termios), non-blocking, no external dependencies.
#ifndef KARMEL_HARDWARE__SERIAL_PORT_HPP_
#define KARMEL_HARDWARE__SERIAL_PORT_HPP_

#include <string>

namespace karmel_hardware
{

class SerialPort
{
public:
  SerialPort() = default;
  ~SerialPort();
  SerialPort(const SerialPort &) = delete;
  SerialPort & operator=(const SerialPort &) = delete;

  /// Open `device` at `baud` in raw 8N1 mode. On failure returns false and sets error().
  bool open(const std::string & device, int baud);
  void close();
  bool is_open() const {return fd_ >= 0;}

  /// Write all bytes (blocks for at most a few ms on a USB CDC port). False on I/O error.
  bool write(const std::string & data);

  /// Append whatever bytes are available right now to `out` (never blocks). False on I/O error.
  bool read_available(std::string & out);

  const std::string & error() const {return error_;}

private:
  int fd_ = -1;
  std::string error_;
};

}  // namespace karmel_hardware

#endif  // KARMEL_HARDWARE__SERIAL_PORT_HPP_
