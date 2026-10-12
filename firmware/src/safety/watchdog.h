// Heartbeat watchdog and command time-to-live. All times in ms (wrap-safe subtraction).
#pragma once
#include <cstdint>

namespace safety {

// Expires when no valid frame has arrived from the Pi for `timeout_ms`.
class Heartbeat {
 public:
  explicit Heartbeat(uint32_t timeout_ms) : timeout_(timeout_ms) {}
  void touch(uint32_t now) {
    last_ = now;
    seen_ = true;
  }
  bool lost(uint32_t now) const { return !seen_ || (now - last_) > timeout_; }

 private:
  uint32_t timeout_;
  uint32_t last_ = 0;
  bool seen_ = false;
};

// A command stays in force for ttl ms after it was received; the MCU never acts on a stale command.
class CommandTtl {
 public:
  void arm(uint32_t now, uint32_t ttl_ms) {
    t0_ = now;
    ttl_ = ttl_ms;
    armed_ = true;
  }
  void disarm() { armed_ = false; }
  bool active(uint32_t now) const { return armed_ && (now - t0_) <= ttl_; }
  bool expiredNow(uint32_t now) {
    if (armed_ && (now - t0_) > ttl_) {
      armed_ = false;
      return true;
    }
    return false;
  }

 private:
  uint32_t t0_ = 0, ttl_ = 0;
  bool armed_ = false;
};

}  // namespace safety
