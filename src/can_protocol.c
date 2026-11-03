/**
 * @file    can_protocol.c
 * @author  Yigit Cakirdogan
 * @brief   Implementation of CAN 2.0B signal decoding/encoding and node supervision.
 */

#include "can_protocol.h"
#include <string.h>

bool can_protocol_ingest_frame(gateway_register_bank_t *bank, const can_frame_t *frame)
{
    if ((bank == NULL) || (frame == NULL) || frame->is_rtr) {
        return false;
    }

    switch (frame->id) {
    case CAN_ID_INV_STATUS_1:
        if (frame->dlc < 6U) {
            return false;
        }
        /* Little-endian CAN signal unpacking -> Modbus 16-bit registers */
        bank->input_regs[0] = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8U); /* RPM */
        bank->input_regs[1] = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8U); /* Temp */
        bank->input_regs[2] = (uint16_t)frame->data[4] | ((uint16_t)frame->data[5] << 8U); /* Vdc */
        bank->last_inv_heartbeat_ms = frame->timestamp_ms;
        bank->can_rx_frames_ok++;
        return true;

    case CAN_ID_BMS_PACK_STAT:
        if (frame->dlc < 4U) {
            return false;
        }
        bank->input_regs[3] = (uint16_t)frame->data[0] | ((uint16_t)frame->data[1] << 8U); /* Pack Current */
        bank->input_regs[4] = (uint16_t)frame->data[2] | ((uint16_t)frame->data[3] << 8U); /* SoC */
        bank->last_bms_heartbeat_ms = frame->timestamp_ms;
        bank->can_rx_frames_ok++;
        return true;

    default:
        return false;
    }
}

void can_protocol_build_cmd_frame(const gateway_register_bank_t *bank, can_frame_t *out_frame)
{
    if ((bank == NULL) || (out_frame == NULL)) {
        return;
    }

    memset(out_frame, 0, sizeof(*out_frame));
    out_frame->id          = CAN_ID_INV_CMD_SET;
    out_frame->is_extended = false;
    out_frame->is_rtr      = false;
    out_frame->dlc         = 6U;

    /* Pack Holding Registers 0..2 into little-endian CAN payload */
    out_frame->data[0] = (uint8_t)(bank->holding_regs[0] & 0xFFU);
    out_frame->data[1] = (uint8_t)((bank->holding_regs[0] >> 8U) & 0xFFU);
    out_frame->data[2] = (uint8_t)(bank->holding_regs[1] & 0xFFU);
    out_frame->data[3] = (uint8_t)((bank->holding_regs[1] >> 8U) & 0xFFU);
    out_frame->data[4] = (uint8_t)(bank->holding_regs[2] & 0xFFU);
    out_frame->data[5] = (uint8_t)((bank->holding_regs[2] >> 8U) & 0xFFU);
}

bool can_protocol_nodes_online(const gateway_register_bank_t *bank,
                               uint32_t current_time_ms,
                               uint32_t timeout_ms)
{
    if (bank == NULL) {
        return false;
    }

    uint32_t inv_age = current_time_ms - bank->last_inv_heartbeat_ms;
    uint32_t bms_age = current_time_ms - bank->last_bms_heartbeat_ms;

    return (inv_age <= timeout_ms) && (bms_age <= timeout_ms);
}
