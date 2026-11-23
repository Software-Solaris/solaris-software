#include "app.h"

void app_main(void)
{
    // 1. Initialize the HAL (Hardware Abstraction Layer)
    APP_initHal();

    // 2. Initialize the FSM (Finite State Machine)
    APP_initFsm();

    // 3. Start the FSM loop (executes the state machine indefinitely)
    APP_fsmLoop();
}
