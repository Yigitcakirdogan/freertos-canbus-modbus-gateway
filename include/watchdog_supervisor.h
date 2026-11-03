/**
 * @file    watchdog_supervisor.h
 * @author  Yigit Cakirdogan
 * @brief   Multi-task Deadman Watchdog Supervisor gating hardware IWDG refresh.
 */

#ifndef WATCHDOG_SUPERVISOR_H
#define WATCHDOG_SUPERVISOR_H

#ifdef __cplusplus
extern "C" {
#endif

#include <stdbool.h>
#include <stdint.h>

#define WDT_MAX_MONITORED_TASKS 8U

typedef struct {
    uint32_t registered_mask;
    uint32_t checkin_mask;
    uint32_t refresh_count;
    uint32_t fault_count;
    void   (*hw_kick_iwdg)(void);
} wdt_supervisor_t;

/**
 * @brief Initialize the watchdog supervisor.
 */
void wdt_supervisor_init(wdt_supervisor_t *wdt, void (*hw_kick_fn)(void));

/**
 * @brief Register a critical RTOS task bit ID (0..7) to be monitored.
 */
bool wdt_supervisor_register_task(wdt_supervisor_t *wdt, uint8_t task_id);

/**
 * @brief Report that a monitored RTOS task completed its control loop iteration.
 */
void wdt_supervisor_task_checkin(wdt_supervisor_t *wdt, uint8_t task_id);

/**
 * @brief Evaluate all registered tasks; kicks hardware IWDG only if every task checked in.
 * @return true if all tasks checked in and hardware watchdog was refreshed.
 */
bool wdt_supervisor_evaluate(wdt_supervisor_t *wdt);

#ifdef __cplusplus
}
#endif

#endif /* WATCHDOG_SUPERVISOR_H */
