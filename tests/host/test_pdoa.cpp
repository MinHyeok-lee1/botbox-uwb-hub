// Host tests for the 0x244 packer and publisher (App/pdoa).
#include <cstdint>
#include <cstdio>
#include <cstring>

#include "pdoa_publisher.hpp"
#include "pdoa_result.hpp"

namespace {

int failures = 0;

void check(bool condition, const char *what, int line) {
  if (!condition) {
    std::printf("FAIL line %d: %s\n", line, what);
    failures++;
  }
}
#define CHECK(x) check((x), #x, __LINE__)

bool frame_equals(const uint8_t *actual, const uint8_t (&expected)[8], int line) {
  if (std::memcmp(actual, expected, 8) == 0) {
    return true;
  }
  std::printf("FAIL line %d: got", line);
  for (int i = 0; i < 8; i++) std::printf(" %02X", actual[i]);
  std::printf(" want");
  for (int i = 0; i < 8; i++) std::printf(" %02X", expected[i]);
  std::printf("\n");
  failures++;
  return false;
}

using pdoa::Lateral;
using pdoa::Result;
using pdoa::Source;

constexpr uint32_t NOW = 100000U;

Result lateral_result(uint16_t distance, int16_t phase, Lateral lat, Source src, uint8_t quality,
                      uint32_t age_ms) {
  Result r;
  r.tag_lost = false;
  r.distance_valid = true;
  r.distance_mm = distance;
  r.lateral_valid = true;
  r.lateral = lat;
  r.source = src;
  r.phase_cdeg = phase;
  r.quality = quality;
  r.has_observation = true;
  r.observed_ms = NOW - age_ms;
  return r;
}

// BBX v0.2 §7, all eight rows, byte exact.
void test_golden_vectors() {
  uint8_t out[8];

  pdoa::pack_result_frame(lateral_result(1500, 250, Lateral::CENTER, Source::OBSERVED, 85, 40),
                          0x10, false, NOW, out);
  frame_equals(out, {0xDC, 0x05, 0xFA, 0x00, 0x10, 0x35, 0x55, 0x0A}, __LINE__);

  pdoa::pack_result_frame(lateral_result(2000, 3500, Lateral::LEFT, Source::OBSERVED, 78, 40),
                          0x11, false, NOW, out);
  frame_equals(out, {0xD0, 0x07, 0xAC, 0x0D, 0x11, 0x36, 0x4E, 0x0A}, __LINE__);

  pdoa::pack_result_frame(lateral_result(2000, 3500, Lateral::LEFT, Source::OBSERVED, 78, 80),
                          0x11, false, NOW, out);
  frame_equals(out, {0xD0, 0x07, 0xAC, 0x0D, 0x11, 0x36, 0x4E, 0x14}, __LINE__);

  pdoa::pack_result_frame(lateral_result(1800, -3000, Lateral::RIGHT, Source::OBSERVED, 80, 40),
                          0x12, false, NOW, out);
  frame_equals(out, {0x08, 0x07, 0x48, 0xF4, 0x12, 0x37, 0x50, 0x0A}, __LINE__);

  pdoa::pack_result_frame(lateral_result(2050, 3400, Lateral::LEFT, Source::HELD, 45, 120), 0x13,
                          false, NOW, out);
  frame_equals(out, {0x02, 0x08, 0x48, 0x0D, 0x13, 0x3A, 0x2D, 0x1E}, __LINE__);

  pdoa::pack_result_frame(lateral_result(2100, 1900, Lateral::LEFT, Source::REFERENCE, 70, 40),
                          0x14, false, NOW, out);
  frame_equals(out, {0x34, 0x08, 0x6C, 0x07, 0x14, 0x3E, 0x46, 0x0A}, __LINE__);

  Result unknown = lateral_result(1700, 0, Lateral::UNKNOWN, Source::NONE, 30, 40);
  unknown.lateral_valid = false;
  pdoa::pack_result_frame(unknown, 0x15, false, NOW, out);
  frame_equals(out, {0xA4, 0x06, 0xFF, 0x7F, 0x15, 0x10, 0x1E, 0x0A}, __LINE__);

  Result lost;
  lost.tag_lost = true;
  lost.has_observation = true;
  lost.observed_ms = NOW - 260;
  pdoa::pack_result_frame(lost, 0x16, false, NOW, out);
  frame_equals(out, {0xFF, 0xFF, 0xFF, 0x7F, 0x16, 0x40, 0xFF, 0x41}, __LINE__);
}

// v0.2 §3: sender guarantees hold whatever the estimator hands over.
void test_invariants() {
  uint8_t out[8];

  // TAG_LOST wipes valid distance and lateral.
  Result r = lateral_result(1500, 250, Lateral::CENTER, Source::OBSERVED, 85, 40);
  r.tag_lost = true;
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[0] == 0xFF && out[1] == 0xFF);
  CHECK(out[2] == 0xFF && out[3] == 0x7F);
  CHECK(out[5] == 0x40);

  // Distance invalid alone is a normal combination.
  r = lateral_result(1500, -2500, Lateral::RIGHT, Source::OBSERVED, 60, 40);
  r.distance_valid = false;
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[0] == 0xFF && out[1] == 0xFF);
  CHECK(out[5] == (0x03 | 0x04 | 0x20));

  // LAT_VALID with UNKNOWN lateral, NONE source or bad phase -> lateral invalid.
  r = lateral_result(1500, 250, Lateral::UNKNOWN, Source::OBSERVED, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[5] == 0x10 && out[2] == 0xFF && out[3] == 0x7F);
  r = lateral_result(1500, 250, Lateral::LEFT, Source::NONE, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[5] == 0x10);
  r = lateral_result(1500, 18000, Lateral::LEFT, Source::OBSERVED, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[5] == 0x10 && out[3] == 0x7F);
  r = lateral_result(1500, -18001, Lateral::RIGHT, Source::OBSERVED, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[5] == 0x10);
  r = lateral_result(1500, -18000, Lateral::RIGHT, Source::OBSERVED, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[5] == 0x37 && out[2] == 0xB0 && out[3] == 0xB9);

  // Distance valid but 0xFFFF -> invalid.
  r = lateral_result(0xFFFF, 0, Lateral::CENTER, Source::OBSERVED, 85, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK((out[5] & 0x10) == 0);

  // Quality 101-254 is never sent.
  r = lateral_result(1500, 0, Lateral::CENTER, Source::OBSERVED, 101, 40);
  pdoa::pack_result_frame(r, 1, false, NOW, out);
  CHECK(out[6] == 0xFF);
  r.quality = 100;
  pdoa::pack_result_frame(r, 1, true, NOW, out);
  CHECK(out[6] == 100);
  CHECK((out[5] & 0x80) != 0);
}

void test_rf_age() {
  CHECK(pdoa::encode_rf_age(false, NOW, NOW) == 0xFF);
  CHECK(pdoa::encode_rf_age(true, NOW, NOW) == 0);
  CHECK(pdoa::encode_rf_age(true, NOW - 3, NOW) == 0);
  CHECK(pdoa::encode_rf_age(true, NOW - 4, NOW) == 1);
  CHECK(pdoa::encode_rf_age(true, NOW - 1016, NOW) == 0xFE);
  CHECK(pdoa::encode_rf_age(true, NOW - 1019, NOW) == 0xFE);
  CHECK(pdoa::encode_rf_age(true, NOW - 1020, NOW) == 0xFF);
  CHECK(pdoa::encode_rf_age(true, NOW - 100000, NOW) == 0xFF);
  CHECK(pdoa::encode_rf_age(true, NOW + 5, NOW) == 0);  // future stamp
  CHECK(pdoa::encode_rf_age(true, 0xFFFFFFF0U, 0x10U) == 8);  // tick wrap: 32 ms
  CHECK(pdoa::encode_rf_age(true, 0U, 40U) == 10);  // tick 0 is a valid time
}

void test_publisher_sequence() {
  pdoa::Publisher p;
  const uint32_t t0 = 5000U;
  p.restart(t0);
  CHECK(p.sequence() == 0);
  CHECK(p.result().tag_lost);
  CHECK(p.is_restarted(t0));
  CHECK(p.is_restarted(t0 + 999));
  CHECK(!p.is_restarted(t0 + 1000));

  Result r = lateral_result(1500, 250, Lateral::CENTER, Source::OBSERVED, 85, 0);
  r.observed_ms = t0 + 10;
  p.submit(r, true);
  CHECK(p.sequence() == 1);

  // Same content, no new observation: resend, seq unchanged.
  p.submit(r, false);
  CHECK(p.sequence() == 1);

  // New observation with identical values is still a new result.
  p.submit(r, true);
  CHECK(p.sequence() == 2);

  // Content change without a new observation (e.g. tag lost): one bump.
  Result lost;
  lost.has_observation = true;
  lost.observed_ms = r.observed_ms;
  p.submit(lost, false);
  CHECK(p.sequence() == 3);
  p.submit(lost, false);
  CHECK(p.sequence() == 3);

  // rf_age grows on resends of the same result.
  uint8_t a[8];
  uint8_t b[8];
  p.build_frame(r.observed_ms + 40, a);
  p.build_frame(r.observed_ms + 80, b);
  CHECK(a[4] == b[4]);
  CHECK(a[7] == 10 && b[7] == 20);

  // Wrap 255 -> 0.
  for (int i = 0; i < 253; i++) {
    p.submit(r, true);
  }
  CHECK(p.sequence() == 0);

  // Reset mid-run: lost result, one bump, RESTARTED again.
  const uint32_t t1 = t0 + 20000;
  p.restart(t1);
  CHECK(p.sequence() == 1);
  CHECK(p.is_restarted(t1 + 500));
  uint8_t f[8];
  p.build_frame(t1 + 500, f);
  CHECK((f[5] & 0xC0) == 0xC0);
  p.build_frame(t1 + 1000, f);
  CHECK((f[5] & 0x80) == 0);
}

void test_publisher_timing() {
  pdoa::Publisher p;
  CHECK(!p.is_due(0));  // not started
  const uint32_t t0 = 1000U;
  p.restart(t0);
  CHECK(p.is_due(t0));
  p.mark_sent(t0);
  CHECK(!p.is_due(t0 + 39));
  CHECK(p.is_due(t0 + 40));

  // Fixed cadence: a late poll does not shift the next tick.
  p.mark_sent(t0 + 47);
  CHECK(!p.is_due(t0 + 79));
  CHECK(p.is_due(t0 + 80));

  // A tick that could not be sent stays due.
  CHECK(p.is_due(t0 + 90));
  CHECK(p.is_due(t0 + 100));

  // After a long stall the cadence restarts from now, no burst.
  p.mark_sent(t0 + 2000);
  CHECK(!p.is_due(t0 + 2039));
  CHECK(p.is_due(t0 + 2040));

  // Tick counter wrap.
  pdoa::Publisher w;
  w.restart(0xFFFFFFF0U);
  w.mark_sent(0xFFFFFFF0U);
  CHECK(!w.is_due(0x00000017U));
  CHECK(w.is_due(0x00000018U));
}

} // namespace

int main() {
  test_golden_vectors();
  test_invariants();
  test_rf_age();
  test_publisher_sequence();
  test_publisher_timing();
  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("pdoa host tests: all passed\n");
  return 0;
}
