#ifndef ARDUINO_TIMING_ADAPTER_H
#define ARDUINO_TIMING_ADAPTER_H

#include <stdint.h>

/**
 * @brief Arduino timing functions adapter
 * 
 * The Basicmicro library uses Arduino's micros() function to implement timeouts.
 * These functions need to be provided for non-Arduino platforms.
 */

#ifdef __cplusplus
extern "C" {
#endif

/**
 * @brief Initialize the microsecond timer
 * 
 * This should be called early in the program to set up the timer
 * used by micros() and millis().
 */
void initMicrosTimer();

/**
 * @brief Get the time in microseconds
 * 
 * Returns the number of microseconds since the program started.
 * This is used for timeout calculations in the Basicmicro library.
 * 
 * @return Microseconds since program start (will overflow after ~71 minutes)
 */
uint32_t micros();

/**
 * @brief Get the time in milliseconds
 * 
 * Returns the number of milliseconds since the program started.
 * 
 * @return Milliseconds since program start (will overflow after ~49 days)
 */
uint32_t millis();

/**
 * @brief Delay for a specified number of milliseconds
 * 
 * Pauses execution for the specified number of milliseconds.
 * 
 * @param ms Milliseconds to delay
 */
void delay(uint32_t ms);

/**
 * @brief Delay for a specified number of microseconds
 * 
 * Pauses execution for the specified number of microseconds.
 * 
 * @param us Microseconds to delay
 */
void delayMicroseconds(uint32_t us);

#ifdef __cplusplus
}
#endif

#endif // ARDUINO_TIMING_ADAPTER_H
