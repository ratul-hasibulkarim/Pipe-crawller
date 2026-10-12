#include "comm/serial_protocol.h"

namespace comm {

bool FrameParser::feed(uint8_t b, Frame& out) {
  switch (st_) {
    case St::SYNC0:
      if (b == proto::kSync0) st_ = St::SYNC1;
      return false;
    case St::SYNC1:
      st_ = (b == proto::kSync1) ? St::LEN0 : (b == proto::kSync0 ? St::SYNC1 : St::SYNC0);
      return false;
    case St::LEN0:
      buf_[0] = b;
      st_ = St::LEN1;
      return false;
    case St::LEN1:
      buf_[1] = b;
      len_ = static_cast<uint16_t>(buf_[0] | (b << 8));
      if (len_ > proto::kMaxPayload) {
        ++errors_;
        st_ = St::SYNC0;
        return false;
      }
      st_ = St::TYPE;
      return false;
    case St::TYPE:
      buf_[2] = b;
      idx_ = 0;
      st_ = len_ ? St::PAYLOAD : St::CRC0;
      return false;
    case St::PAYLOAD:
      buf_[3 + idx_++] = b;
      if (idx_ >= len_) st_ = St::CRC0;
      return false;
    case St::CRC0:
      crc_rx_ = b;
      st_ = St::CRC1;
      return false;
    case St::CRC1: {
      crc_rx_ = static_cast<uint16_t>(crc_rx_ | (b << 8));
      st_ = St::SYNC0;
      if (proto::crc16(buf_, len_ + 3u) != crc_rx_) {
        ++errors_;
        return false;
      }
      out.type = static_cast<proto::MsgType>(buf_[2]);
      out.payload = buf_ + 3;
      out.len = len_;
      return true;
    }
  }
  return false;
}

}  // namespace comm
