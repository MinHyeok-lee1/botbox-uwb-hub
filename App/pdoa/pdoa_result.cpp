#include "pdoa_result.hpp"

namespace pdoa {

namespace {

constexpr uint8_t STATUS_SOURCE_SHIFT = 2U;
constexpr uint8_t STATUS_DIST_VALID = 1U << 4;
constexpr uint8_t STATUS_LAT_VALID = 1U << 5;
constexpr uint8_t STATUS_TAG_LOST = 1U << 6;
constexpr uint8_t STATUS_RESTARTED = 1U << 7;

bool is_lateral_known(Lateral lateral) {
  return lateral == Lateral::CENTER || lateral == Lateral::LEFT || lateral == Lateral::RIGHT;
}

bool is_source_lateral(Source source) {
  return source == Source::OBSERVED || source == Source::HELD || source == Source::REFERENCE;
}

void put_u16(uint8_t *bytes, uint16_t value) {
  bytes[0] = static_cast<uint8_t>(value & 0xFFU);
  bytes[1] = static_cast<uint8_t>(value >> 8);
}

} // namespace

bool same_content(const Result &a, const Result &b) {
  return a.tag_lost == b.tag_lost && a.distance_valid == b.distance_valid &&
         a.distance_mm == b.distance_mm && a.lateral_valid == b.lateral_valid &&
         a.lateral == b.lateral && a.source == b.source && a.phase_cdeg == b.phase_cdeg &&
         a.quality == b.quality && a.has_observation == b.has_observation &&
         a.observed_ms == b.observed_ms;
}

Result normalized(const Result &result) {
  Result out = result;

  if (out.quality > QUALITY_MAX) {
    out.quality = QUALITY_INVALID;
  }
  if (out.tag_lost) {
    out.distance_valid = false;
    out.lateral_valid = false;
  }
  if (!out.distance_valid || out.distance_mm == DISTANCE_INVALID) {
    out.distance_valid = false;
    out.distance_mm = DISTANCE_INVALID;
  }
  if (!out.lateral_valid || !is_lateral_known(out.lateral) || !is_source_lateral(out.source) ||
      out.phase_cdeg < PHASE_CDEG_MIN || out.phase_cdeg > PHASE_CDEG_MAX) {
    out.lateral_valid = false;
    out.lateral = Lateral::UNKNOWN;
    out.source = Source::NONE;
    out.phase_cdeg = PHASE_INVALID;
  }
  return out;
}

uint8_t encode_rf_age(bool has_observation, uint32_t observed_ms, uint32_t now_ms) {
  if (!has_observation) {
    return RF_AGE_INVALID;
  }
  /* An observation stamped in the future (clock skew) counts as age 0. */
  const uint32_t age_ms = time_reached(now_ms, observed_ms) ? now_ms - observed_ms : 0U;
  if (age_ms >= RF_AGE_SATURATION_MS) {
    return RF_AGE_INVALID;
  }
  return static_cast<uint8_t>(age_ms / RF_AGE_UNIT_MS);
}

void pack_result_frame(const Result &result, uint8_t sequence, bool restarted, uint32_t now_ms,
                       uint8_t out[RESULT_FRAME_LENGTH]) {
  const Result r = normalized(result);

  uint8_t status = static_cast<uint8_t>(static_cast<uint8_t>(r.lateral) |
                                        (static_cast<uint8_t>(r.source) << STATUS_SOURCE_SHIFT));
  if (r.distance_valid) {
    status |= STATUS_DIST_VALID;
  }
  if (r.lateral_valid) {
    status |= STATUS_LAT_VALID;
  }
  if (r.tag_lost) {
    status |= STATUS_TAG_LOST;
  }
  if (restarted) {
    status |= STATUS_RESTARTED;
  }

  put_u16(&out[0], r.distance_mm);
  put_u16(&out[2], static_cast<uint16_t>(r.phase_cdeg));
  out[4] = sequence;
  out[5] = status;
  out[6] = r.quality;
  out[7] = encode_rf_age(r.has_observation, r.observed_ms, now_ms);
}

} // namespace pdoa
