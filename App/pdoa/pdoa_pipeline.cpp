#include "pdoa_pipeline.hpp"

#include <cmath>

#include "uwb_core.h"

namespace pdoa {

namespace {

UwbCore core_instance;

UwbCore *as_core(void *core) { return static_cast<UwbCore *>(core); }

constexpr uint8_t SN_LATE_FIRST = 128U;
constexpr uint16_t DISTANCE_SATURATED = 0xFFFEU;
constexpr uint32_t DISTANCE_SATURATED_MM = 65534U;
constexpr double CDEG_PER_DEG = 100.0;
constexpr double QUALITY_SCALE = 100.0;
constexpr int32_t PHASE_WRAP_CDEG = 18000;

/* Core direction order (UNKNOWN, LEFT, CENTER, RIGHT) -> 0x244 lat. */
Lateral to_lateral(UwbDirection direction) {
  switch (direction) {
  case UWB_LEFT:   return Lateral::LEFT;
  case UWB_CENTER: return Lateral::CENTER;
  case UWB_RIGHT:  return Lateral::RIGHT;
  default:         return Lateral::UNKNOWN;
  }
}

Source to_source(UwbOrigin origin) {
  switch (origin) {
  case UWB_CURRENT_OBSERVATION: return Source::OBSERVED;
  case UWB_CONTINUITY_HOLD:     return Source::HELD;
  case UWB_BOUNDARY_REFERENCE:  return Source::REFERENCE;
  default:                      return Source::NONE;
  }
}

/* round(100 * deviation) in [-18000, 17999]; +18000 is the same angle as
   -18000. Returns false when it cannot be sent. */
bool to_phase_cdeg(double deviation_deg, int16_t &out) {
  if (!std::isfinite(deviation_deg)) {
    return false;
  }
  long cdeg = std::lround(deviation_deg * CDEG_PER_DEG);
  if (cdeg == PHASE_WRAP_CDEG) {
    cdeg = -PHASE_WRAP_CDEG;
  }
  if (cdeg < PHASE_CDEG_MIN || cdeg > PHASE_CDEG_MAX) {
    return false;
  }
  out = static_cast<int16_t>(cdeg);
  return true;
}

uint8_t to_quality(double score) {
  if (!std::isfinite(score) || score < 0.0 || score > 1.0) {
    return QUALITY_INVALID;
  }
  return static_cast<uint8_t>(std::lround(score * QUALITY_SCALE));
}

/* Wrap-safe "a is later than b". */
bool after(uint32_t a, uint32_t b) { return static_cast<int32_t>(a - b) > 0; }

} // namespace

bool Pipeline::init() {
  core_ = &core_instance;
  const UwbConfig config = uwb_research_config();
  return uwb_core_init(as_core(core_), &config) && uwb_core_method_enabled(as_core(core_), UWB_V3);
}

uint64_t Pipeline::extend(uint32_t ms) {
  if (!clock_started_) {
    clock_started_ = true;
    clock_last_ = ms;
    clock_wide_ = ms;
    return clock_wide_;
  }
  const int32_t delta = static_cast<int32_t>(ms - clock_last_);
  const uint64_t wide = clock_wide_ + static_cast<int64_t>(delta);
  if (delta > 0) {
    clock_last_ = ms;
    clock_wide_ = wide;
  }
  return wide;
}

void Pipeline::reset_estimator() {
  uwb_core_reset(as_core(core_));
  has_obs_ = false;
  obs_rf_time_known_ = false;
  last_center_valid_ = false;
  last_center_mdeg_ = 0;
}

void Pipeline::restart_sn() {
  sn_started_ = false;
  candidate_started_ = false;
  regression_run_ = 0U;
}

void Pipeline::restart_stream(uint32_t epoch_ms) {
  restart_sn();
  reset_estimator();
  /* The cached CAL may belong to the old boot: wait for one received after
     the restart point. */
  need_new_cal_ = true;
  epoch_ms_ = epoch_ms;
}

bool Pipeline::calibration_fresh(uint32_t now_ms) const {
  return has_cal_ && !need_new_cal_ && cal_.usable() &&
         static_cast<uint32_t>(now_ms - cal_rx_ms_) < CAL_TIMEOUT_MS;
}

bool Pipeline::on_calibration(const Calibration &cal, uint32_t rx_ms) {
  stats_.cal_received++;
  if (!cal.usable()) {
    stats_.cal_unusable++;
  }

  bool restarted = false;
  if (has_cal_ && !cal.same_identity(cal_)) {
    stats_.restarts_cal++;
    if (cal.boot_id != cal_.boot_id) {
      stats_.restarts_boot_id++;
      restart_sn();
    }
    reset_estimator();
    restarted = true;
  }
  if (need_new_cal_ && !after(epoch_ms_, rx_ms)) {
    need_new_cal_ = false;
  }
  cal_ = cal;
  cal_rx_ms_ = rx_ms;
  has_cal_ = true;
  return restarted;
}

Pipeline::ObservationOutcome Pipeline::on_observation(const Observation &obs, uint32_t rx_ms) {
  ObservationOutcome outcome = {false, false};

  /* Backup reboot detection: OBS frames came back after a silence. Measured
     from any OBS frame, so a repeated (duplicate) frame is not silence. */
  const bool silence =
      has_rx_ && static_cast<uint32_t>(rx_ms - last_rx_ms_) >= OBS_SILENCE_RESTART_MS;
  const uint32_t silence_start = last_rx_ms_;
  has_rx_ = true;
  last_rx_ms_ = rx_ms;
  if (silence) {
    stats_.restarts_silence++;
    restart_stream(silence_start);
    outcome.restarted = true;
  }

  if (!sn_started_) {
    sn_started_ = true;
    sequence_++;
  } else {
    const uint8_t d = static_cast<uint8_t>(obs.sn - sn_last_);
    if (d == 0U) {
      stats_.obs_duplicate++;
      return outcome;
    }
    if (d >= SN_LATE_FIRST) {
      stats_.obs_late++;
      /* A restarted anchor sends a new, progressing SN stream; a delayed or
         repeated old frame does not progress. */
      const uint8_t step = static_cast<uint8_t>(obs.sn - candidate_sn_);
      if (candidate_started_ && step >= 1U && step < SN_LATE_FIRST) {
        regression_run_++;
      } else if (!candidate_started_ || step != 0U) {
        regression_run_ = 1U;
      }
      candidate_started_ = true;
      candidate_sn_ = obs.sn;
      if (regression_run_ < SN_REGRESSION_RESTART_COUNT) {
        return outcome;
      }
      stats_.restarts_regression++;
      restart_stream(rx_ms);
      outcome.restarted = true;
      sn_started_ = true;
      sequence_++;
    } else {
      stats_.obs_missing += static_cast<uint32_t>(d - 1U);
      sequence_ += d;
      candidate_started_ = false;
      regression_run_ = 0U;
    }
  }
  sn_last_ = obs.sn;

  /* Observation time on the widened hub clock: no underflow near boot. */
  const uint64_t rx_wide = extend(rx_ms);
  bool rf_time_known = obs.obs_age_ms < IF0_OBS_AGE_SATURATED;
  uint32_t age_ms = rf_time_known ? obs.obs_age_ms : 0U;
  if (obs.obs_age_ms == IF0_OBS_AGE_SATURATED) {
    age_ms = IF0_OBS_AGE_SATURATED; /* at least this old */
  }
  if (rx_wide < age_ms) {
    stats_.obs_age_underflow++;
    rf_time_known = false;
    age_ms = 0U;
  }
  if (!rf_time_known) {
    stats_.obs_no_rf_time++;
  }
  const uint32_t observed_ms = rx_ms - age_ms;
  uint32_t core_tick = observed_ms;
  if (has_obs_ && after(core_tick_ms_, core_tick)) {
    core_tick = core_tick_ms_;
  }

  const bool center = calibration_fresh(rx_ms) && obs.cal_valid &&
                      obs.cal_gen_lsb == ((cal_.generation & 1U) != 0U);
  const int32_t center_mdeg = center ? cal_.phi0_mdeg : 0;
  if (!center) {
    stats_.obs_without_center++;
  }
  /* The Core resets its estimators when the center changes: report it. */
  if (has_obs_ && (center != last_center_valid_ || center_mdeg != last_center_mdeg_)) {
    stats_.restarts_center++;
    outcome.restarted = true;
  }

  UwbObservation input = {};
  input.arrival_ms = rx_wide;
  input.tick_ms = core_tick;
  input.sequence = sequence_;
  input.center_mdeg = center_mdeg;
  input.raw_pdoa = obs.pdoa_raw;
  /* 0xFF (invalid) becomes 0 so it fails the q gate; 0xFE stays 254. */
  input.sts_q = static_cast<int16_t>(obs.sts_q == IF0_STS_Q_INVALID ? 0U : obs.sts_q);
  input.reason_flags = obs.reason;
  input.pdoa_valid = obs.phase_valid && obs.pdoa_raw != IF0_PDOA_INVALID;
  input.center_valid = center;

  if (uwb_core_observe(as_core(core_), &input) != UWB_ACCEPTED) {
    /* Refused input does not refresh anything. */
    stats_.obs_core_refused++;
    return outcome;
  }

  UwbRangeObservation range = {};
  range.tick_ms = core_tick;
  range.sequence = sequence_;
  range.valid = obs.range_valid;
  range.distance_mm = static_cast<int32_t>(
      obs.distance_mm == DISTANCE_SATURATED ? DISTANCE_SATURATED_MM : obs.distance_mm);
  if (uwb_core_observe_range(as_core(core_), &range) != UWB_ACCEPTED) {
    stats_.range_core_refused++;
  }

  stats_.obs_accepted++;
  has_obs_ = true;
  core_tick_ms_ = core_tick;
  obs_rf_time_known_ = rf_time_known;
  obs_rf_ms_ = observed_ms;
  last_center_valid_ = center;
  last_center_mdeg_ = center_mdeg;
  outcome.accepted = true;
  return outcome;
}

Result Pipeline::read(uint32_t now_ms) {
  Result out; /* tag lost, nothing valid */
  UwbTrackingResult r;
  if (!has_obs_ || !uwb_core_read(as_core(core_), UWB_V3, extend(now_ms), &r) || !r.has_observation) {
    return out;
  }

  /* The result is always backed by the last accepted observation: in the V3
     build a direction or distance is only valid when that observation is. */
  out.has_observation = obs_rf_time_known_;
  out.observed_ms = obs_rf_ms_;
  out.tag_lost = r.tag_lost;
  if (r.tag_lost) {
    return out;
  }

  if (r.distance_valid && r.distance_mm >= 0 &&
      r.distance_mm <= static_cast<int32_t>(DISTANCE_SATURATED_MM)) {
    out.distance_valid = true;
    out.distance_mm = static_cast<uint16_t>(r.distance_mm);
  }

  int16_t phase = PHASE_INVALID;
  const Lateral lateral = to_lateral(r.decision.direction);
  const Source source = to_source(r.decision.origin);
  /* A lost CAL stops lateral output even without a new observation. */
  if (r.direction_valid && calibration_fresh(now_ms) && lateral != Lateral::UNKNOWN &&
      source != Source::NONE && to_phase_cdeg(r.decision.deviation_deg, phase)) {
    out.lateral_valid = true;
    out.lateral = lateral;
    out.source = source;
    out.phase_cdeg = phase;
  }
  out.quality = to_quality(r.decision.score);
  return out;
}

} // namespace pdoa
