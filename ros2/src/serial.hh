#pragma once

#include <cstdint>
#include <string>
#include <vector>

class Serial {
 private:
  std::string dev_path_;
  int fd_ = -1;

  bool configure_serial();
  std::vector<uint8_t> readBytes() const;
  void writeBytes(const std::vector<uint8_t> &vec) const;

 public:
  Serial();
  explicit Serial(const std::string &dev_path);
  ~Serial();
  Serial(const Serial &) = delete;
  Serial &operator=(const Serial &) = delete;

  bool isOpen() const;
  void drain() const;
  std::string readString() const;
  void writeString(const std::string &s) const;
  bool sendCommand(const std::string &command,
                   const std::string &expected_response,
                   int timeout_ms = 1000) const;

  inline Serial &operator>>(std::string &s) {
    s = this->readString();
    return *this;
  }

  inline Serial &operator<<(const std::string &s) {
    this->writeString(s);
    return *this;
  }
};
