/**
 * @file    relay_test.h
 * @brief   Hardware test routine for verifying relay digital outputs.
 */
#ifndef RELAY_TEST_H
#define RELAY_TEST_H

/**
 * @brief Run the odd-numbered relay test routine.
 * Energizes odd relays (Relay 1, 3, 5, 7, 9, 11, 13, 15) and loops with heartbeat.
 */
void relay_test_run(void);

#endif /* RELAY_TEST_H */
