/**
 * @file can_loadtest.c
 * @brief Load test of one CAN port on a real bus between two boards.
 *
 * Both boards run this same firmware. The board with the lower CAN identifier
 * is the master: it steps through the phases below once, 10 s each, and puts
 * its phase number in every frame so the other board follows. Each frame also
 * carries a sequence number and a pattern, which the receiver uses to count
 * missed and corrupted frames.
 *
 * Two-way phases are paced below bus capacity: when both boards saturate the
 * bus, the lower identifier wins every arbitration and the other board never
 * transmits, which turns the phase into a one-way test. The last phase stalls
 * the checker on purpose to measure how much delay the RX queue absorbs; its
 * losses are a measurement and are left out of the pass/fail verdict.
 *
 * A generator task produces the traffic and a higher-priority checker task
 * consumes received frames, drives the phases and prints the reports.
 */
#include "can_loadtest.h"

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#include "FreeRTOS.h"
#include "board_config.h"
#include "can_port.h"
#include "cmsis_os.h"
#include "console.h"
#include "task.h"

#define LOADTEST_PORT BOARD_CAN_TEST_PORT
#define LOADTEST_GENERATOR_STACK_WORDS 256U
#define LOADTEST_CHECKER_STACK_WORDS 512U
#define LOADTEST_PHASE_MS 10000U
#define LOADTEST_REPORT_MS 1000U
#define LOADTEST_DONE_REPORT_MS 5000U
#define LOADTEST_STALL_INTERVAL_MS 100U
#define LOADTEST_STALL_STEP_MS 1U /* The stall grows by this much every second. */
#define LOADTEST_SEND_TIMEOUT_MS 100U
#define LOADTEST_BASE_ID 0x100U
#define LOADTEST_NODE_MASK 0xFFU
#define LOADTEST_CLASSIC_LENGTH 8U
#define LOADTEST_SEQUENCE_BYTES 4U
#define LOADTEST_PHASE_INDEX 4U
#define LOADTEST_PATTERN_START 5U
/* A sequence that jumps backwards by more than this means the peer restarted. */
#define LOADTEST_SEQUENCE_REWIND 0x80000000U

/* Frames sent per period. A burst of 0 means saturated: send as fast as the
   TX queue accepts frames. */
typedef struct
{
  uint32_t burst;
  uint32_t period_ms;
} loadtest_rate_t;

#define RATE_SLOW      {1U, 100U} /* 10 frames/s */
#define RATE_500       {1U, 2U}
#define RATE_1000      {1U, 1U}
#define RATE_2000      {2U, 1U}
#define RATE_SATURATED {0U, 0U}

typedef enum
{
  KIND_CLASSIC,
  KIND_FD,
  KIND_FD_BRS,
  KIND_COUNT,
  KIND_MIXED = KIND_COUNT,
} loadtest_kind_t;

typedef struct
{
  const char *name;
  loadtest_rate_t master_rate;
  loadtest_rate_t follower_rate;
  loadtest_kind_t kind;
} loadtest_phase_t;

#define PHASE_STALL 6U
#define PHASE_DONE 7U

static const loadtest_phase_t PHASES[PHASE_DONE + 1U] = {
  {"P0 warm-up      ", RATE_SLOW, RATE_SLOW, KIND_FD_BRS},
  {"P1 master sat   ", RATE_SATURATED, RATE_SLOW, KIND_FD_BRS},
  {"P2 follower sat ", RATE_SLOW, RATE_SATURATED, KIND_FD_BRS},
  {"P3 both 500 brs ", RATE_500, RATE_500, KIND_FD_BRS},
  {"P4 both 1000 brs", RATE_1000, RATE_1000, KIND_FD_BRS},
  {"P5 both 2000 cla", RATE_2000, RATE_2000, KIND_CLASSIC},
  {"P6 stall rx test", RATE_SATURATED, RATE_SLOW, KIND_FD_BRS},
  {"done            ", RATE_SLOW, RATE_SLOW, KIND_FD_BRS},
};

/* Counters that a per-phase or per-second figure is derived from. */
typedef struct
{
  uint32_t tx_sent;
  uint32_t tx_timeouts;
  uint32_t rx_ok;
  uint32_t rx_bad;
  uint32_t rx_missed;
  uint32_t rx_fifo_lost;
  uint32_t rx_queue_dropped;
  uint32_t tx_failed;
  uint32_t bus_off_events;
} loadtest_totals_t;

static StaticTask_t generator_control;
static StackType_t generator_stack[LOADTEST_GENERATOR_STACK_WORDS];
static TaskHandle_t generator_handle;

static StaticTask_t checker_control;
static StackType_t checker_stack[LOADTEST_CHECKER_STACK_WORDS];
static TaskHandle_t checker_handle;

/* Written by the checker task, read by the generator task. */
static volatile uint32_t current_phase;
static volatile bool is_master;

/* Written by the generator task, read by the checker task. */
static volatile uint32_t tx_sent;
static volatile uint32_t tx_timeouts;

/* Checker task state. */
static uint32_t own_id;
static uint32_t peer_id;
static uint32_t peer_sequence;
static bool peer_seen;
static uint32_t peer_restarts;
static uint32_t rx_ok;
static uint32_t rx_bad;
static uint32_t rx_missed;
static uint32_t rx_ignored_before_sync;
static uint32_t last_tx_error_count;
static uint32_t last_rx_error_count;
static uint32_t tx_error_rises;
static uint32_t rx_error_rises;
static uint32_t rx_queue_high_water;
static TickType_t phase_start_tick;
static TickType_t phase_entry_tick; /* When this board entered its current phase. */
static TickType_t next_stall_tick;
static uint32_t stall_ms;
/* Set only after the stall phase has been entered and its baseline recorded. */
static bool stall_active;

static uint32_t derive_own_id(void)
{
  uint32_t uid = HAL_GetUIDw0() ^ HAL_GetUIDw1() ^ HAL_GetUIDw2();
  uint32_t node = (uid ^ (uid >> 8U) ^ (uid >> 16U) ^ (uid >> 24U)) & LOADTEST_NODE_MASK;

  return LOADTEST_BASE_ID | node;
}

static void build_frame(can_frame_t *frame, loadtest_kind_t kind, uint32_t sequence, uint32_t phase)
{
  if (kind == KIND_MIXED)
  {
    kind = (loadtest_kind_t)(sequence % KIND_COUNT);
  }

  frame->id = own_id;
  frame->extended = false;
  frame->fd = (kind != KIND_CLASSIC);
  frame->brs = (kind == KIND_FD_BRS);
  frame->length = frame->fd ? CAN_FRAME_MAX_LENGTH : LOADTEST_CLASSIC_LENGTH;
  for (size_t i = 0U; i < LOADTEST_SEQUENCE_BYTES; i++)
  {
    frame->data[i] = (uint8_t)(sequence >> (8U * i));
  }
  frame->data[LOADTEST_PHASE_INDEX] = (uint8_t)phase;
  for (size_t i = LOADTEST_PATTERN_START; i < frame->length; i++)
  {
    frame->data[i] = (uint8_t)(sequence + i);
  }
}

static void generator_task(void *argument)
{
  /* Static to keep the 72-byte frame off the task stack. */
  static can_frame_t frame;
  uint32_t sequence = 0U;

  (void)argument;

  for (;;)
  {
    uint32_t phase = current_phase;
    const loadtest_phase_t *config = &PHASES[phase];
    const loadtest_rate_t *rate = is_master ? &config->master_rate : &config->follower_rate;
    bool saturated = (rate->burst == 0U);
    uint32_t burst = saturated ? 1U : rate->burst;
    TickType_t timeout = saturated ? pdMS_TO_TICKS(LOADTEST_SEND_TIMEOUT_MS) : 0U;

    for (uint32_t i = 0U; i < burst; i++)
    {
      build_frame(&frame, config->kind, sequence, phase);
      /* The sequence only advances for frames that were queued, so every gap
         the peer sees is a frame lost after this point. */
      if (can_port_send(LOADTEST_PORT, &frame, timeout))
      {
        sequence++;
        tx_sent++;
      }
      else
      {
        tx_timeouts++;
      }
    }

    if (!saturated)
    {
      vTaskDelay(pdMS_TO_TICKS(rate->period_ms));
    }
  }
}

static bool frame_is_valid(const can_frame_t *frame, uint32_t *sequence)
{
  uint32_t value = 0U;

  /* The header must be one of the three shapes the generator produces. */
  if (frame->extended || (frame->brs && !frame->fd) ||
      (frame->length != (frame->fd ? CAN_FRAME_MAX_LENGTH : LOADTEST_CLASSIC_LENGTH)))
  {
    return false;
  }
  if (peer_seen && (frame->id != peer_id))
  {
    return false;
  }
  for (size_t i = 0U; i < LOADTEST_SEQUENCE_BYTES; i++)
  {
    value |= (uint32_t)frame->data[i] << (8U * i);
  }
  if (frame->data[LOADTEST_PHASE_INDEX] > PHASE_DONE)
  {
    return false;
  }
  for (size_t i = LOADTEST_PATTERN_START; i < frame->length; i++)
  {
    if (frame->data[i] != (uint8_t)(value + i))
    {
      return false;
    }
  }

  *sequence = value;
  return true;
}

static void check_frame(const can_frame_t *frame)
{
  uint32_t sequence;

  if (!frame_is_valid(frame, &sequence))
  {
    /* Until the peer runs this test too, it may still be sending frames from
       older firmware; those are not corruption. */
    if (peer_seen)
    {
      rx_bad++;
    }
    else
    {
      rx_ignored_before_sync++;
    }
    return;
  }

  if (!peer_seen)
  {
    /* The first frame fixes the roles; the master starts the cycle from here. */
    peer_seen = true;
    peer_id = frame->id;
    is_master = (own_id < peer_id);
    phase_start_tick = xTaskGetTickCount();
    console_printf("loadtest: own=0x%03lX peer=0x%03lX role=%s\r\n", (unsigned long)own_id,
                   (unsigned long)peer_id, is_master ? "master" : "follower");
  }
  else
  {
    uint32_t skipped = sequence - (peer_sequence + 1U);

    if (skipped >= LOADTEST_SEQUENCE_REWIND)
    {
      peer_restarts++;
    }
    else
    {
      rx_missed += skipped;
    }
  }

  peer_sequence = sequence;
  rx_ok++;
  if (!is_master)
  {
    current_phase = frame->data[LOADTEST_PHASE_INDEX];
  }
}

static void advance_master_phase(void)
{
  TickType_t now = xTaskGetTickCount();

  if (!is_master || !peer_seen || (current_phase >= PHASE_DONE))
  {
    return;
  }
  if ((now - phase_start_tick) < pdMS_TO_TICKS(LOADTEST_PHASE_MS))
  {
    return;
  }
  phase_start_tick = now;
  current_phase = current_phase + 1U;
}

/* Hold the checker off the RX queue for a while, ten times a second, with the
   hold growing every second of the phase. */
static void stall_if_due(void)
{
  TickType_t now = xTaskGetTickCount();

  if (!stall_active || ((int32_t)(now - next_stall_tick) < 0))
  {
    return;
  }

  stall_ms = LOADTEST_STALL_STEP_MS *
             (1U + (uint32_t)((now - phase_entry_tick) / pdMS_TO_TICKS(LOADTEST_REPORT_MS)));
  vTaskDelay(pdMS_TO_TICKS(stall_ms));
  next_stall_tick = xTaskGetTickCount() + pdMS_TO_TICKS(LOADTEST_STALL_INTERVAL_MS);
}

static void read_totals(loadtest_totals_t *totals)
{
  can_port_stats_t stats;

  can_port_get_stats(LOADTEST_PORT, &stats);
  totals->tx_sent = tx_sent;
  totals->tx_timeouts = tx_timeouts;
  totals->rx_ok = rx_ok;
  totals->rx_bad = rx_bad;
  totals->rx_missed = rx_missed;
  totals->rx_fifo_lost = stats.rx_fifo_lost;
  totals->rx_queue_dropped = stats.rx_queue_dropped;
  totals->tx_failed = stats.tx_failed;
  totals->bus_off_events = stats.bus_off_events;

  rx_queue_high_water = stats.rx_queue_high_water;
  /* Count samples where an error counter went up, which means new bus errors.
     Before the peer answers, unacknowledged frames push the transmit counter
     up and it only decays with later successful frames, so the warm-up phase
     is left out and the absolute value is not used. */
  if (current_phase != 0U)
  {
    if (stats.tx_error_count > last_tx_error_count)
    {
      tx_error_rises++;
    }
    if (stats.rx_error_count > last_rx_error_count)
    {
      rx_error_rises++;
    }
  }
  last_tx_error_count = stats.tx_error_count;
  last_rx_error_count = stats.rx_error_count;
}

static void report_second(uint32_t phase, const loadtest_totals_t *now,
                          const loadtest_totals_t *previous)
{
  console_printf("%s| tx %4lu/s rx %4lu/s | miss=%lu bad=%lu fifo=%lu qdrop=%lu\r\n",
                 PHASES[phase].name, (unsigned long)(now->tx_sent - previous->tx_sent),
                 (unsigned long)(now->rx_ok - previous->rx_ok), (unsigned long)now->rx_missed,
                 (unsigned long)now->rx_bad, (unsigned long)now->rx_fifo_lost,
                 (unsigned long)now->rx_queue_dropped);
}

static void report_stall_second(const loadtest_totals_t *now, const loadtest_totals_t *previous)
{
  console_printf("%s| stall %2lu ms rx %4lu/s | miss+%lu qdrop+%lu fifo+%lu rxq_max=%lu\r\n",
                 PHASES[PHASE_STALL].name, (unsigned long)stall_ms,
                 (unsigned long)(now->rx_ok - previous->rx_ok),
                 (unsigned long)(now->rx_missed - previous->rx_missed),
                 (unsigned long)(now->rx_queue_dropped - previous->rx_queue_dropped),
                 (unsigned long)(now->rx_fifo_lost - previous->rx_fifo_lost),
                 (unsigned long)rx_queue_high_water);
}

static void report_phase(uint32_t phase, const loadtest_totals_t *now,
                         const loadtest_totals_t *start)
{
  console_printf("== %s sent=%lu rcvd=%lu miss=%lu bad=%lu fifo=%lu qdrop=%lu\r\n",
                 PHASES[phase].name, (unsigned long)(now->tx_sent - start->tx_sent),
                 (unsigned long)(now->rx_ok - start->rx_ok),
                 (unsigned long)(now->rx_missed - start->rx_missed),
                 (unsigned long)(now->rx_bad - start->rx_bad),
                 (unsigned long)(now->rx_fifo_lost - start->rx_fifo_lost),
                 (unsigned long)(now->rx_queue_dropped - start->rx_queue_dropped));
  console_printf("==   txfail=%lu txto=%lu busoff=%lu tec_rise=%lu rec_rise=%lu rxq_max=%lu\r\n",
                 (unsigned long)(now->tx_failed - start->tx_failed),
                 (unsigned long)(now->tx_timeouts - start->tx_timeouts),
                 (unsigned long)(now->bus_off_events - start->bus_off_events),
                 (unsigned long)tx_error_rises, (unsigned long)rx_error_rises,
                 (unsigned long)rx_queue_high_water);
}

/* @param judged Totals when the stall phase began; the verdict covers
                 everything before it. */
static void report_final(const loadtest_totals_t *totals, const loadtest_totals_t *judged)
{
  bool passed = (judged->rx_missed == 0U) && (judged->rx_bad == 0U) &&
                (judged->rx_fifo_lost == 0U) && (judged->rx_queue_dropped == 0U) &&
                (totals->rx_bad == 0U) && (totals->tx_failed == 0U) &&
                (totals->bus_off_events == 0U) && (tx_error_rises == 0U) &&
                (rx_error_rises == 0U);

  console_printf("#### loadtest %s P0-P5 (%s 0x%03lX): miss=%lu bad=%lu fifo=%lu qdrop=%lu\r\n",
                 passed ? "PASS" : "FAIL", is_master ? "master" : "follower",
                 (unsigned long)own_id, (unsigned long)judged->rx_missed,
                 (unsigned long)judged->rx_bad, (unsigned long)judged->rx_fifo_lost,
                 (unsigned long)judged->rx_queue_dropped);
  console_printf("#### totals incl. stall: sent=%lu rcvd=%lu miss=%lu bad=%lu\r\n",
                 (unsigned long)totals->tx_sent, (unsigned long)totals->rx_ok,
                 (unsigned long)totals->rx_missed, (unsigned long)totals->rx_bad);
  console_printf("#### fifo=%lu qdrop=%lu txfail=%lu txto=%lu busoff=%lu peer_restarts=%lu\r\n",
                 (unsigned long)totals->rx_fifo_lost, (unsigned long)totals->rx_queue_dropped,
                 (unsigned long)totals->tx_failed, (unsigned long)totals->tx_timeouts,
                 (unsigned long)totals->bus_off_events, (unsigned long)peer_restarts);
  console_printf("#### tec_rise=%lu rec_rise=%lu rxq_max=%lu/16 presync=%lu console_drops=%lu\r\n",
                 (unsigned long)tx_error_rises, (unsigned long)rx_error_rises,
                 (unsigned long)rx_queue_high_water, (unsigned long)rx_ignored_before_sync,
                 (unsigned long)console_get_drop_count());
  console_printf("#### stack free words: can_tx=%lu checker=%lu generator=%lu\r\n",
                 (unsigned long)can_port_tx_task_stack_free(),
                 (unsigned long)uxTaskGetStackHighWaterMark(checker_handle),
                 (unsigned long)uxTaskGetStackHighWaterMark(generator_handle));
}

static void checker_task(void *argument)
{
  /* Static to keep the 72-byte frame off the task stack. */
  static can_frame_t frame;
  loadtest_totals_t previous = {0};
  loadtest_totals_t phase_start = {0};
  loadtest_totals_t judged = {0};
  uint32_t reported_phase = 0U;
  TickType_t deadline = xTaskGetTickCount() + pdMS_TO_TICKS(LOADTEST_REPORT_MS);

  (void)argument;

  for (;;)
  {
    int32_t remaining = (int32_t)(deadline - xTaskGetTickCount());

    if ((remaining > 0) && can_port_receive(LOADTEST_PORT, &frame, (TickType_t)remaining))
    {
      check_frame(&frame);
    }
    advance_master_phase();
    stall_if_due();

    uint32_t phase = current_phase;
    bool phase_changed = (phase != reported_phase);

    if (!phase_changed && ((int32_t)(deadline - xTaskGetTickCount()) > 0))
    {
      continue;
    }

    loadtest_totals_t now;

    read_totals(&now);
    if (phase_changed)
    {
      report_phase(reported_phase, &now, &phase_start);
      phase_start = now;
      reported_phase = phase;
      phase_entry_tick = xTaskGetTickCount();
      next_stall_tick = phase_entry_tick;
      stall_active = (phase == PHASE_STALL);
      if (stall_active)
      {
        judged = now;
      }
    }
    else if (phase == PHASE_STALL)
    {
      report_stall_second(&now, &previous);
    }
    else if (phase < PHASE_DONE)
    {
      report_second(phase, &now, &previous);
    }
    if (phase >= PHASE_DONE)
    {
      report_final(&now, &judged);
    }

    previous = now;
    deadline = xTaskGetTickCount() +
               pdMS_TO_TICKS((phase >= PHASE_DONE) ? LOADTEST_DONE_REPORT_MS : LOADTEST_REPORT_MS);
  }
}

void can_loadtest_init(void)
{
  static const osThreadAttr_t generator_attributes = {
    .name = "can_load_gen",
    .cb_mem = &generator_control,
    .cb_size = sizeof(generator_control),
    .stack_mem = generator_stack,
    .stack_size = sizeof(generator_stack),
    .priority = osPriorityBelowNormal,
  };
  static const osThreadAttr_t checker_attributes = {
    .name = "can_load_chk",
    .cb_mem = &checker_control,
    .cb_size = sizeof(checker_control),
    .stack_mem = checker_stack,
    .stack_size = sizeof(checker_stack),
    .priority = osPriorityNormal,
  };

  own_id = derive_own_id();
  checker_handle = (TaskHandle_t)osThreadNew(checker_task, NULL, &checker_attributes);
  generator_handle = (TaskHandle_t)osThreadNew(generator_task, NULL, &generator_attributes);
}
