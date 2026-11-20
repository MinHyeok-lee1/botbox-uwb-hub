/**
 * @file pdoa_pipeline.hpp
 * @brief IF-0 R1 input -> E2 Core (UWB_V3_ONLY) -> 0x244 result.
 *
 * Pure code (no HAL, no RTOS); one instance per anchor stream, all calls from
 * one task. Times are hub milliseconds (RTOS tick at 1 kHz).
 *
 * Rules, from docs/IF0_BYTE_CONTRACT_20261120.md §4 and the Core's
 * INTEGRATION.md:
 * - SN unwrap: d = (u8)(sn - last); 0 duplicate (dropped), 1-127 accepted
 *   (d - 1 missing), 128-255 late (dropped).
 * - Observation time = RX time - obs_age (1 ms). 0xFE/0xFF ages, or an age
 *   longer than the hub uptime, give no RF time: the Core still gets an
 *   estimate, the result reports no RF time (rf_age 0xFF). The time handed to
 *   the Core never goes backwards (the Core would take it for an anchor
 *   restart); the reported RF time stays the reconstructed one.
 * - CAL pairing: center used only when the cached CAL is usable and younger
 *   than 3 s, the OBS says CAL_VALID and its CAL_GEN_LSB matches. Lateral is
 *   published only while the cached CAL is usable and fresh.
 * - Estimator reset (reported as a restart, RESTARTED on 0x244):
 *   - CAL identity change (state, phi0, generation, LR_SIGN, boot_id);
 *   - the center given to the Core changes (the Core resets itself);
 *   - backup reboot detection, because boot_id can repeat: an OBS after 1 s
 *     without any OBS frame, or three late SNs that progress among
 *     themselves (a new SN stream). These also require a CAL received after
 *     the restart point before lateral is used again.
 *   boot_id change and the backups restart SN tracking.
 * - Known limit: the Core does not say which sample supports a direction, so
 *   rf_age counts from the newest accepted observation, which is inside the
 *   Core's window but may not support the published direction.
 *
 * The Core's C header stays out of this header: its global names (e.g.
 * UwbDirection) clash with the 3-anchor code. The Core instance (about
 * 12.5 kB) is a single static object in pdoa_pipeline.cpp, so only one
 * Pipeline may be active; init() takes it over.
 */
#pragma once

#include <cstdint>

#include "pdoa_if0.hpp"
#include "pdoa_result.hpp"

namespace pdoa {

/* Backup reboot detection: OBS silence after which the next OBS restarts the
   stream (UWB team, 2026-11-20: start at 1 s, tune on the vehicle). */
constexpr uint32_t OBS_SILENCE_RESTART_MS = 1000U;
/* Late/reordered SN frames in a row that mean the anchor restarted. */
constexpr uint32_t SN_REGRESSION_RESTART_COUNT = 3U;
/* CAL_STATE comes at 1 Hz; older than this means the calibration is lost. */
constexpr uint32_t CAL_TIMEOUT_MS = 3000U;

struct PipelineStats {
  uint32_t obs_accepted;
  uint32_t obs_duplicate;
  uint32_t obs_late;
  uint32_t obs_missing;      /* gaps counted from SN steps */
  uint32_t obs_bad_frame;    /* decoder refused */
  uint32_t obs_core_refused; /* Core returned something else than ACCEPTED */
  uint32_t range_core_refused;
  uint32_t obs_no_rf_time;   /* obs_age 0xFE/0xFF */
  uint32_t obs_without_center;
  uint32_t cal_received;
  uint32_t cal_bad_frame;
  uint32_t cal_unusable;     /* received, but no usable center */
  uint32_t restarts_cal;     /* CAL identity changed */
  uint32_t restarts_boot_id; /* of which boot_id changed */
  uint32_t restarts_silence;
  uint32_t restarts_regression;
  uint32_t restarts_center;  /* the Core reset itself: center changed */
  uint32_t obs_age_underflow; /* obs_age larger than the hub uptime */
};

class Pipeline {
public:
  /** Initializes the Core. Returns false if the Core refuses its config. */
  bool init();

  /**
   * A decoded CAL_STATE. Returns true when the estimator was reset (the
   * caller restarts the 0x244 publisher).
   */
  bool on_calibration(const Calibration &cal, uint32_t rx_ms);

  struct ObservationOutcome {
    bool accepted;  /* new RF observation fed to the Core */
    bool restarted; /* the estimator was reset before it */
  };
  ObservationOutcome on_observation(const Observation &obs, uint32_t rx_ms);

  /** Current result, evaluated at now_ms (call it even when no input comes). */
  Result read(uint32_t now_ms);

  const PipelineStats &stats() const { return stats_; }
  bool has_calibration() const { return has_cal_; }
  const Calibration &calibration() const { return cal_; }

private:
  uint64_t extend(uint32_t ms);
  void reset_estimator();
  void restart_sn();
  void restart_stream(uint32_t epoch_ms);
  bool calibration_fresh(uint32_t now_ms) const;

  void *core_ = nullptr; /* the module's Core instance, set by init() */
  PipelineStats stats_{};

  /* Hub clock widened to 64 bits for the Core's arrival times. */
  bool clock_started_ = false;
  uint32_t clock_last_ = 0U;
  uint64_t clock_wide_ = 0U;

  /* SN tracking. */
  bool sn_started_ = false;
  uint8_t sn_last_ = 0U;
  uint32_t sequence_ = 0U; /* extended, never goes backwards */
  /* Restart candidate: late SNs that progress on their own. */
  bool candidate_started_ = false;
  uint8_t candidate_sn_ = 0U;
  uint32_t regression_run_ = 0U;

  /* Any decoded OBS frame, accepted or not: silence is measured from it. */
  bool has_rx_ = false;
  uint32_t last_rx_ms_ = 0U;

  /* Last accepted observation. */
  bool has_obs_ = false;
  uint32_t core_tick_ms_ = 0U;
  bool obs_rf_time_known_ = false;
  uint32_t obs_rf_ms_ = 0U;
  /* Center the last accepted observation gave the Core; a change makes the
     Core reset itself. */
  bool last_center_valid_ = false;
  int32_t last_center_mdeg_ = 0;

  /* Cached calibration. */
  bool has_cal_ = false;
  Calibration cal_{};
  uint32_t cal_rx_ms_ = 0U;
  /* After a backup reboot detection the cached CAL may be from the old boot:
     only a CAL received after epoch_ms_ counts. */
  bool need_new_cal_ = false;
  uint32_t epoch_ms_ = 0U;
};

} // namespace pdoa
