# Basicmicro Controller Adapter Documentation

## Overview

This document describes how to use the Basicmicro motor controller library with non-Arduino microcontrollers. The adapter pattern implemented here allows you to use the original Basicmicro library without modifications while providing the necessary interfaces for your specific microcontroller.

## Table of Contents

1. [Architecture](#architecture)
2. [Components](#components)
3. [Implementation Guide](#implementation-guide)
4. [Usage Examples](#usage-examples)
5. [Advanced Features](#advanced-features)
6. [Troubleshooting](#troubleshooting)

## Architecture

The Basicmicro library was designed for Arduino platforms and depends on several Arduino-specific interfaces and functions:

- The `Stream` class for basic I/O operations
- The `HardwareSerial` class for UART communication
- Arduino timing functions like `micros()` and `delay()`

Our adapter architecture provides implementations of these interfaces that can be customized for any microcontroller:

```
Your Application
      │
      ▼
BasicmicroAdapter
      │
      ▼
Basicmicro Library
      │
      ├───────────────┬───────────────┐
      ▼               ▼               ▼
HardwareSerial    Arduino Timing   SoftwareSerial
Implementation    Implementation   (Not used for most MCUs)
      │               │
      ▼               ▼
Microcontroller   Microcontroller
UART Hardware     Timer Hardware
```

## Components

### 1. BasicmicroAdapter

This is the main interface class that wraps the Basicmicro library hardwareserial functions and exposes a Basicmicro Class object. It handles:

- Creating and managing the Basicmicro controller instance(via the BasicmicroAdapter Class)
- abstract HardwareSerial class
- abstract Stream class

### 2. Stream and HardwareSerial Classes

These abstract base classes define the interface that your microcontroller must implement:

- `available()` - Check if data is available to read
- `read()` - Read a byte from the serial port
- `peek()` - Look at the next byte without removing it
- `write()` - Write a byte to the serial port
- `flush()` - Wait for transmission to complete
- `begin()` - Initialize the serial port with a specific baud rate

### 3. Arduino Timing Adapter

These functions provide the timing capabilities expected by the Basicmicro library:

- `micros()` - Get the number of microseconds since program start
- `millis()` - Get the number of milliseconds since program start
- `delay()` - Pause execution for a number of milliseconds
- `delayMicroseconds()` - Pause execution for a number of microseconds

## Implementation Guide

### Step 1: Implement the HardwareSerial Class

Create a subclass of `HardwareSerial` for your specific microcontroller:

```cpp
class MyMCUSerial : public HardwareSerial {
private:
    // Your microcontroller-specific variables
    
public:
    // Implement all required methods
    void begin(long baud_rate) override;
    int available() override;
    int read() override;
    int peek() override;
    size_t write(uint8_t byte) override;
    void flush() override;
};
```

### Step 2: Implement Arduino Timing Functions

Implement the timing functions required by the Basicmicro library:

```cpp
// Initialize the microsecond timer
void initMicrosTimer() {
    // Configure your microcontroller's timer hardware
}

// Get the time in microseconds
uint32_t micros() {
    // Return the timer value in microseconds
}

// Get the time in milliseconds
uint32_t millis() {
    // Return microseconds / 1000
}

// Delay for milliseconds
void delay(uint32_t ms) {
    // Implement a delay function
}

// Delay for microseconds
void delayMicroseconds(uint32_t us) {
    // Implement a microsecond delay
}
```

### Step 3: Include in Your Project

Include the necessary files in your project:

```cpp
#include "Basicmicro.h"         // Original library
#include "BasicmicroAdapter.h"   // Our adapter class
#include "ArduinoTimingAdapter.h" // Timing functions
```

### Step 4: Create and Use the Adapter

```cpp
// Create your serial implementation
MyMCUSerial serial;

// Create the motor controller adapter
BasicmicroAdapter mc(&serial);

// Initialize with desired baud rate
mc.controller.begin(115200);

// Now you can use the motor controller
mc.controller.setMotor1Duty(16384); // 50% forward
```

## Usage Examples

### Basic Motor Control

```cpp
// Set motor speeds
mc.controller.setMotor1Duty(8192);   // 25% forward
mc.controller.setMotor2Duty(-16384); // 50% reverse
mc.controller.setBothMotorsDuty(8192, -8192); // Both motors

// Read encoder values
uint8_t status;
uint32_t position = mc.controller.readEncoder1(&status);

// Reset encoders to zero
mc.controller.resetEncoders();

// Read battery voltage (in tenths of volts, e.g., 124 = 12.4V)
uint16_t voltage = mc.controller.readMainBatteryVoltage();

// Check for errors
uint32_t errors = mc.controller.readError();
if (errors != 0) {
    // Handle error condition
}
```

### Handling Communication Timeouts

The Basicmicro library implements timeout handling using the `micros()` function. Make sure your implementation is accurate:

```cpp
// Read a byte with timeout
int read_timeout(uint32_t timeout_us) {
    uint32_t start = micros();
    while (!available()) {
        if ((micros() - start) >= timeout_us) {
            return -1; // Timeout
        }
    }
    return read();
}
```

## Troubleshooting

### Communication Errors

If you're experiencing communication errors:

1. **Check Baud Rate**: Ensure your UART is configured for the correct baud rate
2. **Timing Accuracy**: Verify that your `micros()` implementation is accurate
3. **Buffer Sizes**: Make sure your receive buffer is large enough

### Timing Issues

If operations are timing out:

1. **Timer Resolution**: Ensure your timer has microsecond resolution
2. **Interrupt Latency**: Check if interrupts are causing delays
3. **Timeout Values**: You may need to increase timeout values for slower processors

### Common Error Codes

The Basicmicro library defines error codes in the `Basicmicro.h` file:

- `ERROR_ESTOP` (0x00000001): E-Stop active
- `ERROR_TEMP` (0x00000002): Temperature Sensor 1 >= 100°C
- `ERROR_LBATHIGH` (0x00000010): Logic Battery High Voltage
- `ERROR_LBATLOW` (0x00000020): Logic Battery Low Voltage
- `ERROR_SPEED1` (0x00000100): Motor 1 Speed Error Limit
- `ERROR_SPEED2` (0x00000200): Motor 2 Speed Error Limit

Check the returned error value against these constants to determine specific issues.

## Conclusion

With the adapter pattern presented in this document, you can use the Basicmicro motor controller library with virtually any microcontroller. By implementing the required interfaces for your specific platform, you maintain the benefits of the original library while adapting it to your needs.

For further assistance or to report issues, please contact the adapter maintainer.
