/**
 * @file    modbus_rtu.c
 * @author  Yigit Cakirdogan
 * @brief   Implementation of Modbus RTU Slave protocol parser & CRC-16 engine.
 */

#include "modbus_rtu.h"
#include <string.h>

uint16_t modbus_crc16_compute(const uint8_t *data, size_t length)
{
    uint16_t crc = 0xFFFFU;
    if (data == NULL) {
        return crc;
    }

    for (size_t i = 0U; i < length; i++) {
        crc ^= (uint16_t)data[i];
        for (uint8_t bit = 0U; bit < 8U; bit++) {
            if ((crc & 0x0001U) != 0U) {
                crc = (uint16_t)((crc >> 1U) ^ 0xA001U);
            } else {
                crc >>= 1U;
            }
        }
    }
    return crc;
}

static size_t append_crc_and_finish(uint8_t *tx_frame, size_t payload_len)
{
    uint16_t crc = modbus_crc16_compute(tx_frame, payload_len);
    tx_frame[payload_len]      = (uint8_t)(crc & 0xFFU);        /* CRC Low Byte  */
    tx_frame[payload_len + 1U] = (uint8_t)((crc >> 8U) & 0xFFU);/* CRC High Byte */
    return payload_len + 2U;
}

static size_t build_exception_response(modbus_rtu_slave_t *ctx,
                                       uint8_t function_code,
                                       uint8_t exception_code,
                                       uint8_t *tx_frame)
{
    ctx->reg_bank->modbus_exceptions++;
    tx_frame[0] = ctx->slave_address;
    tx_frame[1] = (uint8_t)(function_code | 0x80U);
    tx_frame[2] = exception_code;
    return append_crc_and_finish(tx_frame, 3U);
}

bool modbus_rtu_init(modbus_rtu_slave_t *ctx,
                     uint8_t slave_address,
                     gateway_register_bank_t *reg_bank)
{
    if ((ctx == NULL) || (reg_bank == NULL) || (slave_address == 0U) || (slave_address > 247U)) {
        return false;
    }

    memset(ctx, 0, sizeof(*ctx));
    ctx->slave_address = slave_address;
    ctx->reg_bank      = reg_bank;
    return true;
}

size_t modbus_rtu_process_frame(modbus_rtu_slave_t *ctx,
                                const uint8_t *rx_frame,
                                size_t rx_len,
                                uint8_t *tx_frame)
{
    if ((ctx == NULL) || (rx_frame == NULL) || (tx_frame == NULL) || (rx_len < 4U)) {
        return 0U;
    }

    /* Check Slave Address first */
    if (rx_frame[0] != ctx->slave_address) {
        return 0U;
    }

    /* Validate CRC-16 */
    uint16_t expected_crc = modbus_crc16_compute(rx_frame, rx_len - 2U);
    uint16_t received_crc = (uint16_t)rx_frame[rx_len - 2U]
                          | ((uint16_t)rx_frame[rx_len - 1U] << 8U);

    if (expected_crc != received_crc) {
        ctx->reg_bank->modbus_crc_errors++;
        return 0U;
    }

    uint8_t fc = rx_frame[1];

    switch (fc) {
    case MODBUS_FC_READ_HOLDING_REGISTERS:
    case MODBUS_FC_READ_INPUT_REGISTERS: {
        if (rx_len != 8U) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE, tx_frame);
        }

        uint16_t start_addr = ((uint16_t)rx_frame[2] << 8U) | rx_frame[3];
        uint16_t reg_count  = ((uint16_t)rx_frame[4] << 8U) | rx_frame[5];
        uint16_t max_regs   = (fc == MODBUS_FC_READ_HOLDING_REGISTERS)
                            ? GATEWAY_HOLDING_REG_COUNT
                            : GATEWAY_INPUT_REG_COUNT;
        const uint16_t *bank = (fc == MODBUS_FC_READ_HOLDING_REGISTERS)
                             ? ctx->reg_bank->holding_regs
                             : ctx->reg_bank->input_regs;

        if ((reg_count == 0U) || (reg_count > 125U)) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE, tx_frame);
        }
        if (((uint32_t)start_addr + reg_count) > max_regs) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS, tx_frame);
        }

        tx_frame[0] = ctx->slave_address;
        tx_frame[1] = fc;
        tx_frame[2] = (uint8_t)(reg_count * 2U);

        size_t idx = 3U;
        for (uint16_t i = 0U; i < reg_count; i++) {
            uint16_t val = bank[start_addr + i];
            tx_frame[idx++] = (uint8_t)((val >> 8U) & 0xFFU);
            tx_frame[idx++] = (uint8_t)(val & 0xFFU);
        }

        ctx->reg_bank->modbus_requests_ok++;
        return append_crc_and_finish(tx_frame, idx);
    }

    case MODBUS_FC_WRITE_SINGLE_REGISTER: {
        if (rx_len != 8U) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE, tx_frame);
        }

        uint16_t reg_addr = ((uint16_t)rx_frame[2] << 8U) | rx_frame[3];
        uint16_t reg_val  = ((uint16_t)rx_frame[4] << 8U) | rx_frame[5];

        if (reg_addr >= GATEWAY_HOLDING_REG_COUNT) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS, tx_frame);
        }

        ctx->reg_bank->holding_regs[reg_addr] = reg_val;
        if (ctx->on_holding_reg_written != NULL) {
            ctx->on_holding_reg_written(reg_addr, 1U);
        }

        memcpy(tx_frame, rx_frame, 6U);
        ctx->reg_bank->modbus_requests_ok++;
        return append_crc_and_finish(tx_frame, 6U);
    }

    case MODBUS_FC_WRITE_MULTIPLE_REGISTERS: {
        if (rx_len < 9U) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE, tx_frame);
        }

        uint16_t start_addr = ((uint16_t)rx_frame[2] << 8U) | rx_frame[3];
        uint16_t reg_count  = ((uint16_t)rx_frame[4] << 8U) | rx_frame[5];
        uint8_t  byte_count = rx_frame[6];

        if ((reg_count == 0U) || (byte_count != (reg_count * 2U)) || (rx_len != (9U + byte_count))) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_VALUE, tx_frame);
        }
        if (((uint32_t)start_addr + reg_count) > GATEWAY_HOLDING_REG_COUNT) {
            return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_DATA_ADDRESS, tx_frame);
        }

        for (uint16_t i = 0U; i < reg_count; i++) {
            size_t offset = 7U + ((size_t)i * 2U);
            ctx->reg_bank->holding_regs[start_addr + i] =
                ((uint16_t)rx_frame[offset] << 8U) | rx_frame[offset + 1U];
        }

        if (ctx->on_holding_reg_written != NULL) {
            ctx->on_holding_reg_written(start_addr, reg_count);
        }

        memcpy(tx_frame, rx_frame, 6U);
        ctx->reg_bank->modbus_requests_ok++;
        return append_crc_and_finish(tx_frame, 6U);
    }

    default:
        return build_exception_response(ctx, fc, MODBUS_EX_ILLEGAL_FUNCTION, tx_frame);
    }
}
