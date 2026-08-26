#pragma once

#include <cstddef>
#include <cstdint>

namespace armhat_test_clock {

inline std::uint64_t nowUs = 0;

inline void reset(std::uint64_t value = 0) { nowUs = value; }

inline void advanceUs(std::uint64_t value) { nowUs += value; }

}  // namespace armhat_test_clock

inline unsigned long micros() {
  return static_cast<unsigned long>(armhat_test_clock::nowUs);
}

inline unsigned long millis() {
  return static_cast<unsigned long>(armhat_test_clock::nowUs / 1000u);
}

inline void delay(unsigned long milliseconds) {
  armhat_test_clock::advanceUs(static_cast<std::uint64_t>(milliseconds) * 1000u);
}

inline void delayMicroseconds(unsigned int microseconds) {
  armhat_test_clock::advanceUs(microseconds);
}

constexpr std::uint32_t SERIAL_8N1 = 0;

class Stream {
 public:
  virtual ~Stream() = default;
  virtual int available() = 0;
  virtual int read() = 0;
  virtual std::size_t write(const std::uint8_t* data, std::size_t size) = 0;
  virtual void flush() {}
};

class HardwareSerial : public Stream {
 public:
  virtual void begin(std::uint32_t, std::uint32_t, std::int8_t, std::int8_t) {}
};

class EspClass {
 public:
  std::uint64_t getEfuseMac() const { return (std::uint64_t{1} << 8) | 1; }
};

inline EspClass ESP;
