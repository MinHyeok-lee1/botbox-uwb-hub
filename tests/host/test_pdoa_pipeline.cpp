// Host tests for the IF-0 decoder and the IF-0 -> E2 Core -> 0x244 pipeline.
// The vendored Core is compiled unchanged with UWB_V3_ONLY=1.
#include <cstdint>
#include <cstdio>

#include "pdoa_if0.hpp"
#include "pdoa_pipeline.hpp"
#include "pdoa_publisher.hpp"

namespace {

int failures = 0;

void check(bool condition, const char *what, int line) {
  if (!condition) {
    std::printf("FAIL line %d: %s\n", line, what);
    failures++;
  }
}
#define CHECK(x) check((x), #x, __LINE__)

using pdoa::Calibration;
using pdoa::Lateral;
using pdoa::Observation;
using pdoa::Pipeline;
using pdoa::Result;
using pdoa::Source;

// IF-0 contract §6 test vectors.
void test_decode_vectors() {
  Observation obs{};
  const uint8_t obs_ok[8] = {0x5A, 0xC3, 0x2E, 0xFB, 0xF1, 0x29, 0x09, 0x03};
  CHECK(pdoa::decode_observation(obs_ok, obs));
  CHECK(obs.sn == 0x5A && obs.phase_valid && obs.range_valid && obs.reason == 0);
  CHECK(obs.cal_valid && obs.cal_gen_lsb);
  CHECK(obs.pdoa_raw == -1234 && obs.sts_q == 241 && obs.distance_mm == 2345);
  CHECK(obs.obs_age_ms == 3);

  const uint8_t obs_bad[8] = {0x5A, 0x00, 0xFF, 0x7F, 0xFF, 0xFF, 0xFF, 0xFF};
  CHECK(pdoa::decode_observation(obs_bad, obs));
  CHECK(!obs.phase_valid && !obs.range_valid && !obs.cal_valid);
  CHECK(obs.pdoa_raw == 0x7FFF && obs.sts_q == 0xFF && obs.obs_age_ms == 0xFF);

  // RANGE_VALID with 0xFFFF distance is not a usable range.
  const uint8_t obs_range_ffff[8] = {0x01, 0x02, 0, 0, 240, 0xFF, 0xFF, 1};
  CHECK(pdoa::decode_observation(obs_range_ffff, obs));
  CHECK(!obs.range_valid);
  // CAL_GEN_LSB without CAL_VALID is never sent.
  const uint8_t obs_gen_no_cal[8] = {0x01, 0x80, 0, 0, 240, 0x10, 0x00, 1};
  CHECK(!pdoa::decode_observation(obs_gen_no_cal, obs));
  // Reason bits pass through (ADC_WARN alone).
  const uint8_t obs_adc[8] = {0x01, 0x23, 0, 0, 240, 0x10, 0x00, 1};
  CHECK(pdoa::decode_observation(obs_adc, obs));
  CHECK(obs.reason == 0x08);

  Calibration cal{};
  const uint8_t cal_ok[8] = {0x01, 0x5F, 0x6A, 0x00, 0x45, 0x23, 0x6C, 0xA7};
  CHECK(pdoa::decode_calibration(cal_ok, cal));
  CHECK(cal.state == 1 && cal.phi0_mdeg == 27231 && cal.generation == 0x2345);
  CHECK(cal.source_nvm && cal.cal_valid && !cal.dirty && cal.lr_sign == 3 && cal.boot_id == 0xA7);
  CHECK(cal.usable());

  const uint8_t cal_none[8] = {0x00, 0xFF, 0xFF, 0x7F, 0x00, 0x00, 0x13, 0xA7};
  CHECK(pdoa::decode_calibration(cal_none, cal));
  CHECK(cal.state == 0 && cal.phi0_mdeg == pdoa::IF0_PHI0_INVALID);
  CHECK(cal.nvm_mismatch && cal.capture_active && cal.dirty && cal.lr_sign == 0);
  CHECK(!cal.usable());

  // Negative i24: -20.004 deg = 0xFFB1DC.
  const uint8_t cal_neg[8] = {0x02, 0xDC, 0xB1, 0xFF, 0x01, 0x00, 0x68, 0x01};
  CHECK(pdoa::decode_calibration(cal_neg, cal));
  CHECK(cal.phi0_mdeg == -20004 && cal.usable());

  // Reserved: state 3, LR_SIGN 2, b7 set, boot_id 0.
  const uint8_t cal_state3[8] = {0x03, 0, 0, 0, 0, 0, 0x68, 1};
  CHECK(!pdoa::decode_calibration(cal_state3, cal));
  const uint8_t cal_sign2[8] = {0x01, 0, 0, 0, 0, 0, 0x48, 1};
  CHECK(!pdoa::decode_calibration(cal_sign2, cal));
  const uint8_t cal_b7[8] = {0x01, 0, 0, 0, 0, 0, 0xE8, 1};
  CHECK(!pdoa::decode_calibration(cal_b7, cal));
  const uint8_t cal_boot0[8] = {0x01, 0, 0, 0, 0, 0, 0x68, 0};
  CHECK(!pdoa::decode_calibration(cal_boot0, cal));

  // Sign +1 is decodable but not usable by the Core (it applies -1).
  const uint8_t cal_plus[8] = {0x02, 0, 0, 0, 0, 0, 0x28, 1};
  CHECK(pdoa::decode_calibration(cal_plus, cal));
  CHECK(!cal.usable());
}

Calibration make_cal(int32_t phi0_mdeg, uint16_t generation, uint8_t boot_id) {
  Calibration cal{};
  cal.state = 1;
  cal.phi0_mdeg = phi0_mdeg;
  cal.generation = generation;
  cal.cal_valid = true;
  cal.source_nvm = true;
  cal.lr_sign = pdoa::IF0_LR_SIGN_MINUS;
  cal.boot_id = boot_id;
  return cal;
}

Observation make_obs(uint8_t sn, int16_t raw, uint16_t distance, uint16_t generation) {
  Observation obs{};
  obs.sn = sn;
  obs.phase_valid = true;
  obs.range_valid = true;
  obs.reason = 0;
  obs.cal_valid = true;
  obs.cal_gen_lsb = (generation & 1U) != 0U;
  obs.pdoa_raw = raw;
  obs.sts_q = 250;
  obs.distance_mm = distance;
  obs.obs_age_ms = 5;
  return obs;
}

constexpr uint32_t T0 = 100000U;
constexpr uint32_t PERIOD = 40U;
constexpr uint16_t GEN = 1U;

// Feeds count observations of the same phase; returns the next SN.
uint8_t feed(Pipeline &p, uint8_t sn, uint32_t &t, unsigned count, int16_t raw, uint16_t generation) {
  for (unsigned i = 0; i < count; i++) {
    const Pipeline::ObservationOutcome o =
        p.on_observation(make_obs(sn, raw, 2000, generation), t);
    check(o.accepted, "observation accepted", __LINE__);
    sn = static_cast<uint8_t>(sn + 1U);
    t += PERIOD;
  }
  return sn;
}

// Same scenario as the Core's examples/minimal.c, through the IF-0 adapter.
void test_left_follow_and_lost() {
  static Pipeline p; // the Core instance is about 12.5 kB
  p = Pipeline();
  CHECK(p.init());

  Result r = p.read(T0);
  CHECK(r.tag_lost && !r.has_observation);

  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), T0));
  uint32_t t = T0 + 10;
  uint8_t sn = feed(p, 253, t, 8, -1024, GEN); // wraps 255 -> 0
  const uint32_t last_rx = t - PERIOD;

  r = p.read(last_rx);
  CHECK(!r.tag_lost);
  CHECK(r.distance_valid && r.distance_mm == 2000);
  CHECK(r.lateral_valid && r.lateral == Lateral::LEFT);
  CHECK(r.source == Source::OBSERVED || r.source == Source::REFERENCE);
  CHECK(r.phase_cdeg > 0); // LEFT is positive
  CHECK(r.quality <= 100);
  CHECK(r.has_observation && r.observed_ms == last_rx - 5);
  CHECK(p.stats().obs_missing == 0 && p.stats().obs_accepted == 8);

  // rf_age counts from the RF observation, not the reception.
  CHECK(pdoa::encode_rf_age(r.has_observation, r.observed_ms, last_rx + 35) == 10);

  // 250 ms after the last arrival: still valid; 251 ms: tag lost.
  r = p.read(last_rx + 250);
  CHECK(!r.tag_lost && r.lateral_valid);
  r = p.read(last_rx + 251);
  CHECK(r.tag_lost && !r.lateral_valid && !r.distance_valid);
  CHECK(r.has_observation); // rf_age keeps growing from the last observation
  (void)sn;
}

void test_sn_rules() {
  static Pipeline p;
  p = Pipeline();
  CHECK(p.init());
  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), T0));
  uint32_t t = T0;
  uint8_t sn = feed(p, 10, t, 3, -1024, GEN); // 10, 11, 12

  // Duplicate is dropped.
  Pipeline::ObservationOutcome o = p.on_observation(make_obs(12, -1024, 2000, GEN), t);
  CHECK(!o.accepted && p.stats().obs_duplicate == 1);

  // Gap of two: accepted, two missing.
  o = p.on_observation(make_obs(15, -1024, 2000, GEN), t);
  CHECK(o.accepted && p.stats().obs_missing == 2);
  t += PERIOD;

  // Late frames are dropped. Unrelated late frames (14, 13, 3) and one stale
  // frame repeated (14, 14, 14) do not restart the stream.
  o = p.on_observation(make_obs(14, -1024, 2000, GEN), t);
  CHECK(!o.accepted && !o.restarted);
  o = p.on_observation(make_obs(13, -1024, 2000, GEN), t);
  CHECK(!o.accepted && !o.restarted);
  o = p.on_observation(make_obs(3, -1024, 2000, GEN), t);
  CHECK(!o.accepted && !o.restarted);
  for (int i = 0; i < 3; i++) {
    o = p.on_observation(make_obs(14, -1024, 2000, GEN), t);
    CHECK(!o.accepted && !o.restarted);
  }
  CHECK(p.stats().restarts_regression == 0);

  // A late stream that progresses on its own (4, 5, 6) is a restarted anchor.
  o = p.on_observation(make_obs(4, -1024, 2000, GEN), t);
  CHECK(!o.accepted);
  t += PERIOD;
  o = p.on_observation(make_obs(5, -1024, 2000, GEN), t);
  CHECK(!o.accepted);
  t += PERIOD;
  o = p.on_observation(make_obs(6, -1024, 2000, GEN), t);
  CHECK(o.accepted && o.restarted && p.stats().restarts_regression == 1);
  t += PERIOD;
  // The new stream continues from SN 6.
  o = p.on_observation(make_obs(7, -1024, 2000, GEN), t);
  CHECK(o.accepted && !o.restarted);
  // Lateral needs a CAL received after the restart point (old boot's CAL).
  for (uint8_t i = 8; i < 16; i++) {
    CHECK(p.on_observation(make_obs(i, -1024, 2000, GEN), t).accepted);
    t += PERIOD;
  }
  Result r = p.read(t - PERIOD);
  CHECK(r.distance_valid && !r.lateral_valid);
  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), t)); // same identity
  for (uint8_t i = 16; i < 26; i++) {
    p.on_observation(make_obs(i, -1024, 2000, GEN), t);
    t += PERIOD;
  }
  r = p.read(t - PERIOD);
  CHECK(r.lateral_valid);
  (void)sn;
}

void test_calibration_rules() {
  static Pipeline p;
  p = Pipeline();
  CHECK(p.init());
  uint32_t t = T0;

  // OBS before any CAL: distance yes, lateral no.
  uint8_t sn = feed(p, 1, t, 5, -1024, GEN);
  Result r = p.read(t - PERIOD);
  CHECK(r.distance_valid && !r.lateral_valid && !r.tag_lost);
  CHECK(p.stats().obs_without_center == 5);

  // First CAL is no restart; lateral comes after enough samples.
  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), t));
  sn = feed(p, sn, t, 8, -1024, GEN);
  r = p.read(t - PERIOD);
  CHECK(r.lateral_valid && r.lateral == Lateral::LEFT);

  // Same CAL again (1 Hz repeat) or only DIRTY changed: no restart.
  Calibration again = make_cal(12000, GEN, 5);
  again.dirty = true;
  CHECK(!p.on_calibration(again, t));

  // OBS whose CAL_GEN_LSB does not match: no center for it.
  const uint32_t before = p.stats().obs_without_center;
  CHECK(p.on_observation(make_obs(sn, -1024, 2000, 2), t).accepted);
  sn++;
  t += PERIOD;
  CHECK(p.stats().obs_without_center == before + 1);
  r = p.read(t - PERIOD);
  CHECK(!r.lateral_valid && r.distance_valid);

  // Generation change resets the estimator (restart).
  CHECK(p.on_calibration(make_cal(12000, 2, 5), t));
  CHECK(p.stats().restarts_cal == 1 && p.stats().restarts_boot_id == 0);
  r = p.read(t);
  CHECK(r.tag_lost && !r.has_observation); // nothing since the reset

  // boot_id change: restart, SN tracking starts again (old SN accepted).
  sn = feed(p, sn, t, 4, -1024, 2);
  CHECK(p.on_calibration(make_cal(12000, 2, 9), t));
  CHECK(p.stats().restarts_boot_id == 1);
  CHECK(p.on_observation(make_obs(0, -1024, 2000, 2), t).accepted);
  t += PERIOD;

  // CAL lost (3 s without CAL_STATE): lateral stops even with new OBS.
  sn = 1;
  sn = feed(p, sn, t, 8, -1024, 2);
  r = p.read(t - PERIOD);
  CHECK(r.lateral_valid);
  const uint32_t cal_time = t - 8 * PERIOD; // when the boot_id CAL was given, roughly
  uint32_t late = cal_time + pdoa::CAL_TIMEOUT_MS + 10;
  while (t < late) {
    CHECK(p.on_observation(make_obs(sn, -1024, 2000, 2), t).accepted);
    sn++;
    t += PERIOD;
  }
  r = p.read(t - PERIOD);
  CHECK(!r.lateral_valid && r.distance_valid);

  // Unusable CAL (sign +1) never gives lateral.
  Calibration plus = make_cal(12000, 2, 9);
  plus.lr_sign = pdoa::IF0_LR_SIGN_PLUS;
  CHECK(p.on_calibration(plus, t)); // identity changed -> restart
  sn = feed(p, sn, t, 8, -1024, 2);
  r = p.read(t - PERIOD);
  CHECK(!r.lateral_valid && r.distance_valid);
}

void test_silence_and_age() {
  static Pipeline p;
  p = Pipeline();
  CHECK(p.init());
  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), T0));
  uint32_t t = T0;
  uint8_t sn = feed(p, 50, t, 4, -1024, GEN);

  // 999 ms silence: no restart. 1000 ms: restart (boot_id may repeat).
  uint32_t last = t - PERIOD;
  Pipeline::ObservationOutcome o =
      p.on_observation(make_obs(sn, -1024, 2000, GEN), last + 999);
  CHECK(o.accepted && !o.restarted);
  sn++;
  last += 999;
  o = p.on_observation(make_obs(sn, -1024, 2000, GEN), last + 1000);
  CHECK(o.accepted && o.restarted && p.stats().restarts_silence == 1);
  last += 1000;

  // obs_age unknown: the result has no RF time (rf_age 0xFF).
  Observation unknown = make_obs(static_cast<uint8_t>(sn + 1), -1024, 2000, GEN);
  unknown.obs_age_ms = pdoa::IF0_OBS_AGE_UNKNOWN;
  CHECK(p.on_observation(unknown, last + 40).accepted);
  Result r = p.read(last + 40);
  CHECK(!r.has_observation);
  CHECK(pdoa::encode_rf_age(r.has_observation, r.observed_ms, last + 40) == 0xFF);

  // A large obs_age jump would move the time backwards: the Core must not see
  // a restart, so the observation is still accepted.
  Observation old = make_obs(static_cast<uint8_t>(sn + 2), -1024, 2000, GEN);
  old.obs_age_ms = 200;
  CHECK(p.on_observation(old, last + 80).accepted);
  CHECK(p.stats().obs_core_refused == 0);
  r = p.read(last + 80);
  CHECK(r.has_observation && r.observed_ms == last + 80 - 200);

  // Range invalid alone: lateral may stay, distance goes.
  Observation no_range = make_obs(static_cast<uint8_t>(sn + 3), -1024, 0xFFFF, GEN);
  no_range.range_valid = false;
  CHECK(p.on_observation(no_range, last + 120).accepted);
  r = p.read(last + 120);
  CHECK(!r.distance_valid);
}

// Codex review 2026-11-20: duplicates, CAL_VALID-only change, center change,
// time underflow near boot.
void test_review_cases() {
  static Pipeline p;
  p = Pipeline();
  CHECK(p.init());
  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), T0));
  uint32_t t = T0;
  uint8_t sn = feed(p, 20, t, 10, -1024, GEN);
  const uint32_t last = t - PERIOD;
  Result r = p.read(last);
  CHECK(r.lateral_valid);

  // The same frame repeated for 1.2 s: no silence restart, no new result,
  // and the tag goes lost after 250 ms (duplicates do not refresh).
  const uint8_t repeated = static_cast<uint8_t>(sn - 1U);
  for (uint32_t dt = 40; dt <= 1200; dt += 40) {
    Pipeline::ObservationOutcome o =
        p.on_observation(make_obs(repeated, -1024, 2000, GEN), last + dt);
    CHECK(!o.accepted && !o.restarted);
  }
  CHECK(p.stats().restarts_silence == 0);
  r = p.read(last + 1200);
  CHECK(r.tag_lost && !r.distance_valid);

  // Back to new frames: no restart (frames never stopped), then valid again.
  t = last + 1240;
  sn = feed(p, sn, t, 10, -1024, GEN);
  r = p.read(t - PERIOD);
  CHECK(!r.tag_lost && r.lateral_valid);

  // CAL_VALID cleared with the same identity: lateral stops at once.
  Calibration invalid = make_cal(12000, GEN, 5);
  invalid.cal_valid = false;
  CHECK(!p.on_calibration(invalid, t - PERIOD));
  r = p.read(t - PERIOD);
  CHECK(!r.lateral_valid && r.distance_valid);

  // The center given to the Core changes (now none): the Core resets, which
  // is reported as a restart.
  Pipeline::ObservationOutcome o = p.on_observation(make_obs(sn, -1024, 2000, GEN), t);
  CHECK(o.accepted && o.restarted && p.stats().restarts_center == 1);

  // obs_age longer than the hub uptime: no RF time, no wrapped time.
  static Pipeline q;
  q = Pipeline();
  CHECK(q.init());
  Observation early = make_obs(1, -1024, 2000, GEN);
  early.obs_age_ms = 20;
  CHECK(q.on_observation(early, 10).accepted);
  CHECK(q.stats().obs_age_underflow == 1);
  r = q.read(10);
  CHECK(!r.has_observation && r.distance_valid);
  CHECK(q.on_observation(make_obs(2, -1024, 2000, GEN), 50).accepted);
  CHECK(q.stats().obs_core_refused == 0);
}

// The publisher bumps seq once per accepted observation and on restarts.
void test_with_publisher() {
  static Pipeline p;
  p = Pipeline();
  CHECK(p.init());
  pdoa::Publisher pub;
  pub.restart(T0);
  const uint8_t start = pub.sequence();

  CHECK(!p.on_calibration(make_cal(12000, GEN, 5), T0));
  uint32_t t = T0;
  for (uint8_t sn = 0; sn < 5; sn++) {
    const Pipeline::ObservationOutcome o = p.on_observation(make_obs(sn, -1024, 2000, GEN), t);
    if (o.restarted) pub.restart(t);
    if (o.accepted) pub.submit(p.read(t), true);
    pub.submit(p.read(t + 10), false); // a poll in between: no new result
    t += PERIOD;
  }
  CHECK(static_cast<uint8_t>(pub.sequence() - start) == 5);

  uint8_t frame[8];
  pub.build_frame(t, frame);
  CHECK((frame[5] & 0x10) != 0); // DIST_VALID
  CHECK((frame[5] & 0x80) != 0); // still in the 1 s RESTARTED window
}

} // namespace

int main() {
  test_decode_vectors();
  test_left_follow_and_lost();
  test_sn_rules();
  test_calibration_rules();
  test_silence_and_age();
  test_review_cases();
  test_with_publisher();
  if (failures != 0) {
    std::printf("%d failure(s)\n", failures);
    return 1;
  }
  std::printf("pdoa pipeline host tests: all passed\n");
  return 0;
}
