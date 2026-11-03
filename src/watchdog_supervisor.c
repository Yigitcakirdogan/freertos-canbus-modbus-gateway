/**
 * @file    watchdog_supervisor.c
 * @author  Yigit Cakirdogan
 * @brief   Implementation of multi-task Deadman Watchdog Supervisor.
 */

#include "watchdog_supervisor.h"
#include <stddef.h>
#include <string.h>

void wdt_supervisor_init(wdt_supervisor_t *wdt, void (*hw_kick_fn)(void))
{
    if (wdt == NULL) {
        return;
    }
    memset(wdt, 0, sizeof(*wdt));
    wdt->hw_kick_iwdg = hw_kick_fn;
}

bool wdt_supervisor_register_task(wdt_supervisor_t *wdt, uint8_t task_id)
{
    if ((wdt == NULL) || (task_id >= WDT_MAX_MONITORED_TASKS)) {
        return false;
    }
    wdt->registered_mask |= (1UL << task_id);
    return true;
}

void wdt_supervisor_task_checkin(wdt_supervisor_t *wdt, uint8_t task_id)
{
    if ((wdt == NULL) || (task_id >= WDT_MAX_MONITORED_TASKS)) {
        return;
    }
    wdt->checkin_mask |= (1UL << task_id);
}

bool wdt_supervisor_evaluate(wdt_supervisor_t *wdt)
{
    if ((wdt == NULL) || (wdt->registered_mask == 0U)) {
        return false;
    }

    if ((wdt->checkin_mask & wdt->registered_mask) == wdt->registered_mask) {
        wdt->checkin_mask = 0U;
        wdt->refresh_count++;
        if (wdt->hw_kick_iwdg != NULL) {
            wdt->hw_kick_iwdg();
        }
        return true;
    }

    wdt->fault_count++;
    return false;
}
