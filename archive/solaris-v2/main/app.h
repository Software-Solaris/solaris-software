#ifndef APP_H
#define APP_H

/* ----------------------------------------------------------------
 * PUBLIC FUNCTIONS
 * ---------------------------------------------------------------- */
/**
 * @brief Initialize the HAL (Hardware Abstraction Layer).
 */
void APP_initHal(void);

/**
 * @brief Initialize the FSM (Finite State Machine).
 */
void APP_initFsm(void);

/**
 * @brief Main loop for the FSM (executes the state machine indefinitely).
 */
void APP_fsmLoop(void);

#endif /* APP_H */
