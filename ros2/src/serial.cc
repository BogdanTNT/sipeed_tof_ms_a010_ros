#include "serial.hh"

#include <chrono>
#include <iostream>
#include <stdexcept>
#include <string>
#include <thread>
#include <vector>

#ifndef WIN32
#include <errno.h>
#include <fcntl.h>
#include <string.h>
#include <sys/ioctl.h>
#include <termios.h>
#include <unistd.h>
#endif

#ifndef WIN32
#define MS_A010_DEFAULT_SERIAL_DEV_PATH "/dev/ttyUSB0"
#else
#error "NOT SUPPORTED WIN32 NOW"
#define MS_A010_DEFAULT_SERIAL_DEV_PATH "COMx"
#endif

Serial::Serial() : Serial(MS_A010_DEFAULT_SERIAL_DEV_PATH) {}

Serial::Serial(const std::string &dev_path) : dev_path_(dev_path) {
  if (this->dev_path_.empty()) {
    std::cerr << "Error: no dev path provided" << std::endl;
    return;
  }
  if (!this->configure_serial()) {
    std::cerr << "Error: failed to configure serial device" << std::endl;
    return;
  }
}

#ifndef WIN32
Serial::~Serial() {
  if (fd_ != -1) {
    close(fd_);
  }
}
#else
#endif

static inline int serial_setup(int serial_port, speed_t baudrate);

bool Serial::configure_serial() {
  fd_ = -1;

  if ((fd_ = open(this->dev_path_.c_str(), O_RDWR | O_NOCTTY | O_NDELAY)) <
      0) {
    std::cerr << "path for serial can't open" << std::endl;
    return false;
  }
  fcntl(fd_, F_SETFL, 0);

  if (serial_setup(fd_, B115200) < 0) {
    std::cerr << "setup serial failed" << std::endl;
    close(fd_);
    fd_ = -1;
    return false;
  }

  return true;
}

bool Serial::isOpen() const { return fd_ != -1; }

void Serial::drain() const {
  if (fd_ == -1) {
    return;
  }
  tcflush(fd_, TCIFLUSH);
}

std::vector<uint8_t> Serial::readBytes() const {
  if (fd_ == -1) {
    return {};
  }
  uint8_t read_buf[16 * 1024];
  ssize_t bytes_read = read(fd_, read_buf, sizeof(read_buf));
  if (bytes_read < 0) {
    return {};
  }
  return std::vector<uint8_t>(read_buf, read_buf + bytes_read);
}

void Serial::writeBytes(const std::vector<uint8_t> &vec) const {
  if (fd_ == -1) {
    return;
  }
  write(fd_, vec.data(), vec.size());
}

std::string Serial::readString() const {
  std::vector<uint8_t> v = this->readBytes();
  return std::string(v.cbegin(), v.cend());
}

void Serial::writeString(const std::string &s) const {
  this->writeBytes(std::vector<uint8_t>(s.cbegin(), s.cend()));
}

bool Serial::sendCommand(const std::string &command,
                         const std::string &expected_response,
                         int timeout_ms) const {
  if (fd_ == -1) {
    return false;
  }

  this->drain();
  this->writeString(command);
  if (expected_response.empty()) {
    return true;
  }

  auto start_time = std::chrono::steady_clock::now();
  std::string response;
  while (std::chrono::duration_cast<std::chrono::milliseconds>(
             std::chrono::steady_clock::now() - start_time)
             .count() < timeout_ms) {
    response += this->readString();
    if (response.find(expected_response) != std::string::npos) {
      return true;
    }
    std::this_thread::sleep_for(std::chrono::milliseconds(10));
  }

  std::cerr << "Timeout or error on command: [" << command.substr(0, command.size() - 1)
            << "]. Expected [" << expected_response << "], Got [" << response
            << "]" << std::endl;
  return false;
}

/* 115200, 8, N, 1 */
static inline int serial_setup(int serial_port, speed_t baudrate) {
  struct termios tty;
  // Read in existing settings, and handle any error
  if (tcgetattr(serial_port, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcgetattr: " << strerror(errno)
              << std::endl;
    return -1;
  }
  tty.c_cflag &= ~PARENB;  // Clear parity bit, disabling parity (most common)
  tty.c_cflag &= ~CSTOPB;  // Clear stop field, only one stop bit used in
                           // communication (most common)
  tty.c_cflag &= ~CSIZE;   // Clear all the size bits, then use one of the
                           // statements below
  tty.c_cflag |= CS8;      // 8 bits per byte (most common)
  tty.c_cflag &=
      ~CRTSCTS;  // Disable RTS/CTS hardware flow control (most common)
  tty.c_cflag |=
      (CREAD | CLOCAL);  // Turn on READ & ignore ctrl lines (CLOCAL = 1)

  tty.c_lflag &= ~ICANON;
  tty.c_lflag &= ~ECHO;    // Disable echo
  tty.c_lflag &= ~ECHOE;   // Disable erasure
  tty.c_lflag &= ~ECHONL;  // Disable new-line echo
  tty.c_lflag &= ~ISIG;    // Disable interpretation of INTR, QUIT and SUSP

  tty.c_iflag &= ~(IXON | IXOFF | IXANY);  // Turn off s/w flow ctrl
  tty.c_iflag &= ~(IGNBRK | BRKINT | PARMRK | ISTRIP | INLCR | IGNCR |
                   ICRNL);  // Disable any special handling of received bytes

  tty.c_oflag &= ~OPOST;  // Prevent special interpretation of output bytes
                          // (e.g. newline chars)
  tty.c_oflag &=
      ~ONLCR;  // Prevent conversion of newline to carriage return/line feed

  tty.c_cc[VMIN] = 0;
  tty.c_cc[VTIME] = 1;  // Wait for up to 100ms, returning as soon as data is
                        // received.

  cfsetspeed(&tty, baudrate);
  // Save tty settings, also checking for error
  if (tcsetattr(serial_port, TCSANOW, &tty) != 0) {
    std::cerr << "Error" << errno << "from tcsetattr: " << strerror(errno)
              << std::endl;
    return -1;
  }
  return 0;
}
#if TEST_FUNC_ENABLE
static int test() {
  Serial ser{};
  std::vector<uint8_t> vec;
  std::string s;

  bool cap = false;

  char curr_key;
  while (1) {
    std::cin >> curr_key;
    std::cout << "[handler] curr_key: " << curr_key << std::endl;
    switch (curr_key) {
      case 'a':
        ser << "AT\r";
        break;
      case 's':
        ser << "AT+DISP=3\r";
        cap = true;
        break;
      case 'p':
        ser << "AT+DISP=1\r";
        ser >> s;
        while (!s.empty()) {
          ser >> s;
        }
        cap = false;
        break;
      case 'c':
        ser << "AT+COEFF?\r";
        break;
      case 'w':
        std::cout << std::string(vec.begin(), vec.end()) << std::endl;
        std::vector<uint8_t>().swap(vec);
        break;
    }

    ser >> s;
    if (cap) {
      std::cout << "vec size: " << vec.size() << std::endl;
      vec.insert(vec.end(), s.begin(), s.end());
    } else {
      std::cout << s << std::endl;
    }
  }

  return 0;
}
#endif

// int main(int argc, char const *argv[]) {
//   test();
//   return 0;
// }
