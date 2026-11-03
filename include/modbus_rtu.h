/**
 * @file    modbus_rtu.h
 * @author  Yigit Cakirdogan
 * @brief   Deterministic Modbus RTU Slave protocol engine (FC03, FC04, FC06, FC10).
 */

#ifndef MODBUS_RTU_H
#define MODBUS_RTU_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gateway_types.h"

#define MODBUS_MAX_ADU_LENGTH              256U

/* Standard Modbus Function Codes */
#define MODBUS_FC_READ_HOLDING_REGISTERS   0x03U
#define MODBUS_FC_READ_INPUT_REGISTERS     0x04U
#define MODBUS_FC_WRITE_SINGLE_REGISTER    0x06U
#define MODBUS_FC_WRITE_MULTIPLE_REGISTERS 0x10U

/* Standard Modbus Exception Codes */
#define MODBUS_EX_ILLEGAL_FUNCTION         0x01U
#define MODBUS_EX_ILLEGAL_DATA_ADDRESS     0x02U
#define MODBUS_EX_ILLEGAL_DATA_VALUE       0x03U

typedef struct {
    uint8_t                  slave_address;
    gateway_register_bank_t *reg_bank;
    void                   (*on_holding_reg_written)(uint16_t start_addr, uint16_t count);
} modbus_rtu_slave_t;

/**
 * @brief Compute standard Modbus RTU CRC-16 (polynomial 0xA001, init 0xFFFF).
 */
uint16_t modbus_crc16_compute(const uint8_t *data, size_t length);

/**
 * @brief Initialize a Modbus RTU slave instance.
 */
bool modbus_rtu_init(modbus_rtu_slave_t *ctx,
                     uint8_t slave_address,
                     gateway_register_bank_t *reg_bank);

/**
 * @brief Process a received Modbus RTU ADU frame and generate the response ADU.
 * @param  ctx       Pointer to slave context.
 * @param  rx_frame  Pointer to incoming RTU frame (including 2-byte CRC).
 * @param  rx_len    Length of incoming frame in bytes.
 * @param  tx_frame  Output buffer for response ADU (must be >= MODBUS_MAX_ADU_LENGTH).
 * @return Length of response frame in bytes (0 if silent/unaddressed or CRC failed).
 */
size_t modbus_rtu_process_frame(modbus_rtu_slave_t *ctx,
                                const uint8_t *rx_frame,
                                size_t rx_len,
                                uint8_t *tx_frame);

#ifdef __cplusplus
}
#endif

#endif /* MODBUS_RTU_H */
