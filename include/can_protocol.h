/**
 * @file    can_protocol.h
 * @author  Yigit Cakirdogan
 * @brief   CAN 2.0B frame signal packing/unpacking and node timeout monitoring.
 */

#ifndef CAN_PROTOCOL_H
#define CAN_PROTOCOL_H

#ifdef __cplusplus
extern "C" {
#endif

#include "gateway_types.h"

/**
 * @brief Decode an incoming CAN 2.0B telemetry frame into the shared register bank.
 * @return true if the CAN ID was recognized and decoded, false otherwise.
 */
bool can_protocol_ingest_frame(gateway_register_bank_t *bank, const can_frame_t *frame);

/**
 * @brief Encode current Holding Register setpoints into a CAN 2.0B command frame (0x200).
 */
void can_protocol_build_cmd_frame(const gateway_register_bank_t *bank, can_frame_t *out_frame);

/**
 * @brief Check whether critical CAN nodes have sent heartbeats within @p timeout_ms.
 */
bool can_protocol_nodes_online(const gateway_register_bank_t *bank,
                               uint32_t current_time_ms,
                               uint32_t timeout_ms);

#ifdef __cplusplus
}
#endif

#endif /* CAN_PROTOCOL_H */
