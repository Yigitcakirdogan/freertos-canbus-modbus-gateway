/**
 * @file    gateway_types.h
 * @author  Yigit Cakirdogan
 * @brief   Shared data structures for CAN 2.0B frames and Modbus RTU register banks.
 */

#ifndef GATEWAY_TYPES_H
#define GATEWAY_TYPES_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stddef.h>
#include <stdint.h>

#define GATEWAY_INPUT_REG_COUNT    16U
#define GATEWAY_HOLDING_REG_COUNT  16U

/* Known CAN IDs */
#define CAN_ID_INV_STATUS_1        0x180UL
#define CAN_ID_BMS_PACK_STAT       0x280UL
#define CAN_ID_INV_CMD_SET         0x200UL

/**
 * @brief CAN 2.0B Frame Structure.
 */
typedef struct {
    uint32_t id;           /**< 11-bit standard or 29-bit extended identifier */
    bool     is_extended;  /**< true for 29-bit IDE, false for 11-bit         */
    bool     is_rtr;       /**< Remote transmission request flag              */
    uint8_t  dlc;          /**< Data length code (0..8 bytes)                 */
    uint8_t  data[8];      /**< Payload bytes                                 */
    uint32_t timestamp_ms; /**< Reception tick timestamp                      */
} can_frame_t;

/**
 * @brief Shared Industrial Gateway Register Bank.
 */
typedef struct {
    uint16_t input_regs[GATEWAY_INPUT_REG_COUNT];     /**< Read-Only telemetry (FC04)  */
    uint16_t holding_regs[GATEWAY_HOLDING_REG_COUNT]; /**< Read/Write setpoints (FC03) */
    uint32_t can_rx_frames_ok;
    uint32_t can_tx_frames_ok;
    uint32_t modbus_requests_ok;
    uint32_t modbus_crc_errors;
    uint32_t modbus_exceptions;
    uint32_t last_inv_heartbeat_ms;
    uint32_t last_bms_heartbeat_ms;
} gateway_register_bank_t;

#ifdef __cplusplus
}
#endif

#endif /* GATEWAY_TYPES_H */
