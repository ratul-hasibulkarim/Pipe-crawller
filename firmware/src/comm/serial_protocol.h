// Streaming frame parser for [AA 55][len][type][payload][crc16]. Static buffer, no
// allocation, resynchronises on garbage, counts CRC/length errors.
#pragma once
#include <cstddef>
#include <cstdint>

#include "protocol_gen.h"

namespace comm {

struct Frame {
  proto::MsgType type;
  const uint8_t* payload;
  uint16_t len;
};

class FrameParser {
 public:
  // Feed one byte; returns true when a complete, valid frame is available in `out`
  // (valid until the next call).
  bool feed(uint8_t b, Frame& out);
  uint16_t errors() const { return errors_; }
  void reset() { st_ = St::SYNC0; }

 private:
  enum class St : uint8_t { SYNC0, SYNC1, LEN0, LEN1, TYPE, PAYLOAD, CRC0, CRC1 };
  St st_ = St::SYNC0;
  uint8_t buf_[proto::kMaxPayload + 3]{};  // len(2) type(1) payload
  uint16_t len_ = 0, idx_ = 0, crc_rx_ = 0;
  uint16_t errors_ = 0;
};

}  // namespace comm
