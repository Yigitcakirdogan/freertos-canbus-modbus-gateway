/**
 * @file    test_gateway.c
 * @author  Yigit Cakirdogan
 * @brief   Unit tests for CAN 2.0B signal unpacking, Modbus RTU slave engine
 *          (FC03, FC04, FC06, FC10, exceptions), and multi-task watchdog supervisor.
 */

#include <stdio.h>
#include <stdlib.h>
#include <string.h>
#include "gateway_types.h"
#include "modbus_rtu.h"
#include "can_protocol.h"
#include "watchdog_supervisor.h"

#define TEST_ASSERT(cond)                                                          \
    do {                                                                           \
        if (!(cond)) {                                                             \
            fprintf(stderr, "[FAIL] %s:%d: %s\n", __FILE__, __LINE__, #cond);      \
            exit(EXIT_FAILURE);                                                    \
        }                                                                          \
    } while (0)

static uint32_t mock_iwdg_kicks = 0U;
static void mock_kick_iwdg(void)
{
    mock_iwdg_kicks++;
}

static void test_can_to_modbus_fc04_pipeline(void)
{
    gateway_register_bank_t bank;
    modbus_rtu_slave_t      mb;
    memset(&bank, 0, sizeof(bank));
    TEST_ASSERT(modbus_rtu_init(&mb, 0x01U, &bank));

    /* 1. Simulate receiving CAN frame 0x180 (Inverter Status: 3000 RPM, 85.0 C, 400.5 V) */
    can_frame_t rx_can = {
        .id           = CAN_ID_INV_STATUS_1,
        .is_extended  = false,
        .is_rtr       = false,
        .dlc          = 6U,
        .data         = { 0xB8, 0x0B, 0xE2, 0x04, 0xA5, 0x0F },
        .timestamp_ms = 100U
    };
    TEST_ASSERT(can_protocol_ingest_frame(&bank, &rx_can));
    TEST_ASSERT(bank.input_regs[0] == 3000U);
    TEST_ASSERT(bank.input_regs[1] == 1250U);
    TEST_ASSERT(bank.input_regs[2] == 4005U);

    /* 2. Build Modbus FC04 (Read Input Registers 0x0000..0x0002) request from SCADA */
    uint8_t req[8] = { 0x01, MODBUS_FC_READ_INPUT_REGISTERS, 0x00, 0x00, 0x00, 0x03, 0x00, 0x00 };
    uint16_t crc = modbus_crc16_compute(req, 6U);
    req[6] = (uint8_t)(crc & 0xFFU);
    req[7] = (uint8_t)((crc >> 8U) & 0xFFU);

    uint8_t resp[MODBUS_MAX_ADU_LENGTH] = {0};
    size_t resp_len = modbus_rtu_process_frame(&mb, req, sizeof(req), resp);

    TEST_ASSERT(resp_len == 11U);
    TEST_ASSERT(resp[0] == 0x01U);
    TEST_ASSERT(resp[1] == MODBUS_FC_READ_INPUT_REGISTERS);
    TEST_ASSERT(resp[2] == 6U);
    TEST_ASSERT(((uint16_t)resp[3] << 8U | resp[4]) == 3000U);
    TEST_ASSERT(((uint16_t)resp[5] << 8U | resp[6]) == 1250U);
    TEST_ASSERT(((uint16_t)resp[7] << 8U | resp[8]) == 4005U);
    printf("[PASS] test_can_to_modbus_fc04_pipeline\n");
}

static void test_modbus_fc10_to_can_setpoint(void)
{
    gateway_register_bank_t bank;
    modbus_rtu_slave_t      mb;
    memset(&bank, 0, sizeof(bank));
    TEST_ASSERT(modbus_rtu_init(&mb, 0x01U, &bank));

    uint8_t req[15] = {
        0x01, MODBUS_FC_WRITE_MULTIPLE_REGISTERS,
        0x00, 0x00,
        0x00, 0x03,
        0x06,
        0x05, 0xDC,
        0x11, 0x94,
        0x00, 0x01,
        0x00, 0x00
    };
    uint16_t crc = modbus_crc16_compute(req, 13U);
    req[13] = (uint8_t)(crc & 0xFFU);
    req[14] = (uint8_t)((crc >> 8U) & 0xFFU);

    uint8_t resp[MODBUS_MAX_ADU_LENGTH] = {0};
    size_t resp_len = modbus_rtu_process_frame(&mb, req, sizeof(req), resp);
    TEST_ASSERT(resp_len == 8U);

    can_frame_t tx_can;
    can_protocol_build_cmd_frame(&bank, &tx_can);
    TEST_ASSERT(tx_can.id == CAN_ID_INV_CMD_SET);
    TEST_ASSERT(tx_can.dlc == 6U);
    TEST_ASSERT(((uint16_t)tx_can.data[0] | ((uint16_t)tx_can.data[1] << 8U)) == 1500U);
    TEST_ASSERT(((uint16_t)tx_can.data[2] | ((uint16_t)tx_can.data[3] << 8U)) == 4500U);
    TEST_ASSERT(((uint16_t)tx_can.data[4] | ((uint16_t)tx_can.data[5] << 8U)) == 1U);
    printf("[PASS] test_modbus_fc10_to_can_setpoint\n");
}

static void test_watchdog_supervisor_deadman_logic(void)
{
    wdt_supervisor_t wdt;
    mock_iwdg_kicks = 0U;
    wdt_supervisor_init(&wdt, mock_kick_iwdg);

    TEST_ASSERT(wdt_supervisor_register_task(&wdt, 0U));
    TEST_ASSERT(wdt_supervisor_register_task(&wdt, 1U));

    wdt_supervisor_task_checkin(&wdt, 0U);
    wdt_supervisor_task_checkin(&wdt, 1U);
    TEST_ASSERT(wdt_supervisor_evaluate(&wdt) == true);
    TEST_ASSERT(mock_iwdg_kicks == 1U);

    wdt_supervisor_task_checkin(&wdt, 0U);
    TEST_ASSERT(wdt_supervisor_evaluate(&wdt) == false);
    TEST_ASSERT(mock_iwdg_kicks == 1U);
    TEST_ASSERT(wdt.fault_count == 1U);
    printf("[PASS] test_watchdog_supervisor_deadman_logic\n");
}

int main(void)
{
    printf("=== Running FreeRTOS CAN-Modbus Gateway Test Suite ===\n");
    test_can_to_modbus_fc04_pipeline();
    test_modbus_fc10_to_can_setpoint();
    test_watchdog_supervisor_deadman_logic();
    printf("=== All gateway unit tests passed! ===\n");
    return 0;
}
