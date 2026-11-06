/**
 * @file can_port.h
 * @brief Queued CAN FD frame transport on the FDCAN peripherals.
 *
 * Received frames are queued from the interrupt; transmission goes through a
 * software queue drained by the CAN TX task. All functions are task-context
 * only, and any number of tasks may send to or receive from the same port. A
 * received frame goes to exactly one of the tasks waiting on that port.
 */
#ifndef CAN_PORT_H
#define CAN_PORT_H

#include <stdbool.h>
#include <stdint.h>

#include "FreeRTOS.h"

#ifdef __cplusplus
extern "C" {
#endif

#define CAN_FRAME_MAX_LENGTH 64U

typedef enum
{
  CAN_PORT_1,
  CAN_PORT_2,
  CAN_PORT_COUNT,
} can_port_id_t;

typedef struct
{
  uint32_t id;
  uint32_t rx_tick; /* RTOS tick at reception, set by the driver. Ignored on send. */
  uint8_t length; /* Payload bytes: 0-8, 12, 16, 20, 24, 32, 48 or 64. */
  bool extended;  /* 29-bit identifier. */
  bool fd;        /* CAN FD frame format. */
  bool brs;       /* Bit rate switching, only meaningful with fd. */
  uint8_t data[CAN_FRAME_MAX_LENGTH];
} can_frame_t;

typedef struct
{
  bool enabled;
  /* The peripheral is transmitted to itself internally and is disconnected
     from its pins. For tests without a bus. */
  bool internal_loopback;
} can_port_config_t;

typedef struct
{
  bool enabled;
  bool started;      /* On the bus. Set by the CAN TX task shortly after start-up. */
  bool start_failed; /* The peripheral refused its configuration; the port stays off. */
  bool bus_off;
  uint8_t tx_error_count;
  uint8_t rx_error_count;
  uint32_t rx_frames;
  uint32_t rx_queue_dropped; /* Software RX queue was full. */
  uint32_t rx_queue_high_water; /* Most frames ever waiting in the RX queue. */
  uint32_t rx_fifo_lost;     /* Hardware RX FIFO overflowed. */
  uint32_t tx_frames;
  uint32_t tx_failed;
  uint32_t bus_off_events;
  /* Live protocol status. The error codes reset to 7 (unchanged) once read. */
  uint8_t last_error;
  uint8_t data_last_error;
  uint8_t tdc_value; /* Secondary sample point position, in kernel clock periods. */
  uint8_t tdc_offset; /* Configured compensation offset, 0 when it is off. */
  /* Protocol status captured at the most recent bus-off (LEC codes: 1 stuff,
     2 form, 3 ack, 4 bit1, 5 bit0, 6 crc, 7 unchanged). */
  uint8_t bus_off_last_error;
  uint8_t bus_off_data_last_error;
  uint8_t bus_off_tdc_value;
} can_port_stats_t;

/**
 * Create the queues and the CAN TX task. Call once before the scheduler starts.
 *
 * The ports themselves are started by the CAN TX task once the scheduler runs;
 * the outcome is reported in can_port_stats_t (started, start_failed).
 *
 * @param config One entry per port, indexed by can_port_id_t.
 * @return false if a queue or the task could not be created.
 */
bool can_port_init(const can_port_config_t config[CAN_PORT_COUNT]);

/**
 * Queue a frame for transmission.
 *
 * A true result means the frame was queued, not that it reached the bus: frames
 * wait in the queue while the port is in bus-off or has no peer to acknowledge
 * them, and frames already in the hardware are discarded by bus-off recovery.
 *
 * @return false if the port is disabled or failed to start, the frame is
 *         invalid or the TX queue stayed full for @p timeout.
 */
bool can_port_send(can_port_id_t port_id, const can_frame_t *frame, TickType_t timeout);

/**
 * Frames waiting for transmission: those in the software TX queue plus those
 * in the three hardware TX buffers. A frame the CAN TX task is handing over
 * at this moment is not counted. A periodic sender can use this to skip a
 * cycle instead of letting old frames pile up while nobody acknowledges them.
 */
uint32_t can_port_tx_pending(can_port_id_t port_id);

/** Wait for a received frame. @return false on timeout. */
bool can_port_receive(can_port_id_t port_id, can_frame_t *frame, TickType_t timeout);

void can_port_get_stats(can_port_id_t port_id, can_port_stats_t *stats);

/** Smallest amount of free stack the CAN TX task has ever had, in words. */
uint32_t can_port_tx_task_stack_free(void);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PORT_H */
