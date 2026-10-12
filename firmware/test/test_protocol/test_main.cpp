// Wire protocol: CRC, framing round trip, resync on garbage, bad CRC / length rejection.
#include <unity.h>

#include <cstring>
#include <vector>

#include "comm/serial_protocol.h"
#include "protocol_gen.h"

void setUp() {}
void tearDown() {}

void test_crc16_ccitt_known_vector() {
  const char* s = "123456789";
  TEST_ASSERT_EQUAL_HEX16(0x29B1, proto::crc16(reinterpret_cast<const uint8_t*>(s), 9));
}

static std::vector<proto::TelemetryMsg> decodeAll(const std::vector<uint8_t>& bytes, comm::FrameParser& p) {
  std::vector<proto::TelemetryMsg> out;
  for (uint8_t b : bytes) {
    comm::Frame f{};
    if (p.feed(b, f) && f.type == proto::MsgType::TELEMETRY) {
      proto::TelemetryMsg t;
      if (t.unpack(f.payload, f.len)) out.push_back(t);
    }
  }
  return out;
}

static std::vector<uint8_t> encodeTel(float diameter, uint32_t faults) {
  proto::TelemetryMsg t;
  t.est_diameter_mm = diameter;
  t.faults = faults;
  t.grip_force_n[2] = 12.5f;
  t.state = static_cast<uint8_t>(proto::State::CRAWLING);
  uint8_t buf[256];
  const size_t n = proto::encode(t, buf, sizeof(buf));
  return std::vector<uint8_t>(buf, buf + n);
}

void test_round_trip_all_fields() {
  comm::FrameParser p;
  auto msgs = decodeAll(encodeTel(178.4f, 0x105), p);
  TEST_ASSERT_EQUAL(1, msgs.size());
  TEST_ASSERT_EQUAL_FLOAT(178.4f, msgs[0].est_diameter_mm);
  TEST_ASSERT_EQUAL_UINT32(0x105, msgs[0].faults);
  TEST_ASSERT_EQUAL_FLOAT(12.5f, msgs[0].grip_force_n[2]);
  TEST_ASSERT_EQUAL(static_cast<int>(proto::State::CRAWLING), msgs[0].state);
}

void test_resync_after_garbage_and_split_feed() {
  comm::FrameParser p;
  std::vector<uint8_t> s = {0x00, 0xAA, 0x12, 0xAA, 0xAA};
  auto a = encodeTel(100, 1), b = encodeTel(200, 2);
  s.insert(s.end(), a.begin(), a.end());
  s.insert(s.end(), {0x55, 0x55, 0xAA});
  s.insert(s.end(), b.begin(), b.end());
  auto msgs = decodeAll(s, p);
  TEST_ASSERT_EQUAL(2, msgs.size());
  TEST_ASSERT_EQUAL_FLOAT(200.0f, msgs[1].est_diameter_mm);
}

void test_bad_crc_rejected_and_counted() {
  comm::FrameParser p;
  auto a = encodeTel(100, 1);
  a[10] ^= 0x40;  // flip one payload bit
  auto msgs = decodeAll(a, p);
  TEST_ASSERT_EQUAL(0, msgs.size());
  TEST_ASSERT_EQUAL(1, p.errors());
  auto good = decodeAll(encodeTel(150, 0), p);  // parser recovers
  TEST_ASSERT_EQUAL(1, good.size());
}

void test_oversize_length_rejected() {
  comm::FrameParser p;
  std::vector<uint8_t> s = {0xAA, 0x55, 0xFF, 0x7F, 0x80};
  auto msgs = decodeAll(s, p);
  TEST_ASSERT_EQUAL(0, msgs.size());
  TEST_ASSERT_EQUAL(1, p.errors());
}

void test_command_sizes_fixed() {
  TEST_ASSERT_EQUAL(10, proto::DriveMsg::kSize);
  proto::DriveMsg d;
  d.linear_mm_s = -60;
  d.ttl_ms = 200;
  uint8_t buf[32];
  TEST_ASSERT_EQUAL(d.pack(buf), proto::DriveMsg::kSize);
  proto::DriveMsg e;
  TEST_ASSERT_TRUE(e.unpack(buf, proto::DriveMsg::kSize));
  TEST_ASSERT_EQUAL(-60, e.linear_mm_s);
  TEST_ASSERT_FALSE(e.unpack(buf, 3));
}

int main() {
  UNITY_BEGIN();
  RUN_TEST(test_crc16_ccitt_known_vector);
  RUN_TEST(test_round_trip_all_fields);
  RUN_TEST(test_resync_after_garbage_and_split_feed);
  RUN_TEST(test_bad_crc_rejected_and_counted);
  RUN_TEST(test_oversize_length_rejected);
  RUN_TEST(test_command_sizes_fixed);
  return UNITY_END();
}
