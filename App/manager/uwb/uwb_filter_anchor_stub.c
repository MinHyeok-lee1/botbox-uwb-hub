#include "uwb_filter.h"

__attribute__((weak)) twr_dist_filter_t *
twr_get_filter_by_anchor(uint16_t anchor_id) {
  (void)anchor_id;
  return (twr_dist_filter_t *)0;
}
