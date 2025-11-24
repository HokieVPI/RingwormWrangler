# DL-TDoA Test Setup

This folder contains a simplified DL-TDoA (Downlink Time Difference of Arrival) test setup using 2 Portenta C33 anchors and 1 Portenta C33 tag.

## Overview

This is a simplified test configuration that displays measurement data without full position calculation. It's designed for testing and debugging DL-TDoA functionality with a minimal 2-anchor setup.

## Hardware Requirements

- 2x Arduino Portenta C33 boards with UWB Shield (for anchors)
- 1x Arduino Portenta C33 board with UWB Shield (for tag)
- USB cables for programming and power

## Software Requirements

- Arduino IDE with Portenta C33 board support
- PortentaUWBShield library installed

## Configuration

### Anchor Positions

- **Anchor 1 (Initiator)**: (0, 0) meters
- **Anchor 2 (Responder)**: (1.8, 0) meters

### MAC Addresses

- **Initiator**: 0xEE, 0xEE
- **Responder**: 0xFF, 0xFF
- **Tag**: 0x11, 0x11

### Session ID

All devices use session ID: **0x87654321**

## Setup Instructions

1. **Upload Initiator Script**
   - Open `DL_TDoA_Test_Anchor_Initiator/DL_TDoA_Test_Anchor_Initiator.ino`
   - Select "Arduino Portenta C33" as the board
   - Upload to the first Portenta C33 with UWB Shield
   - The red LED (LEDR) will blink when running

2. **Upload Responder Script**
   - Open `DL_TDoA_Test_Anchor_Responder/DL_TDoA_Test_Anchor_Responder.ino`
   - Select "Arduino Portenta C33" as the board
   - Upload to the second Portenta C33 with UWB Shield
   - The green LED (LEDG) will blink when running

3. **Upload Tag Script**
   - Open `DL_TDoA_Test_Tag/DL_TDoA_Test_Tag.ino`
   - Select "Arduino Portenta C33" as the board
   - Upload to the third Portenta C33 with UWB Shield
   - The blue LED (LEDB) will blink when running

## Usage

1. Power on all three devices
2. Wait for UWB stack initialization (about 1-2 seconds)
3. The anchors will begin ranging with each other
4. The tag will passively listen and display measurement data via Serial Monitor

## Serial Monitor Output

### Anchor Initiator
- Displays anchor position and configuration
- Shows UWB initialization status
- Confirms when ranging starts

### Anchor Responder
- Displays anchor position and configuration
- Shows UWB initialization status
- Confirms when ranging starts

### Tag
- Displays received DL-TDoA measurements including:
  - Timestamps (RX and TX)
  - Time-of-flight estimates
  - Distance estimates
  - Angle of Arrival (AoA) - Azimuth and Elevation
  - Anchor-to-anchor time-of-flight
  - Measurement status and indices

## Measurement Data

The tag displays the following information for each measurement:

- **Anchor identification** (MAC address and position)
- **RX Timestamp** (reception time at tag)
- **TX Timestamp** (transmission time from anchor)
- **Time-of-Flight** (estimated from timestamps)
- **Distance Estimate** (calculated from ToF)
- **AoA Azimuth** (angle in degrees)
- **AoA Elevation** (angle in degrees)
- **Figure of Merit (FOM)** (quality indicator for AoA)
- **Anchor-to-Anchor ToF** (time-of-flight between anchors)
- **Status** (validity of measurement)
- **Block and Round Indices** (ranging session information)

## Notes

- This is a **test setup** that displays measurements without full 2D position calculation
- With only 2 anchors, full 2D positioning is not possible (would require 3+ anchors or AoA assistance)
- The session ID (0x87654321) is different from the main DL-TDoA setup to avoid conflicts
- MAC addresses are unique to avoid conflicts with other UWB devices
- All devices must be powered on and within UWB range for measurements to be received

## Troubleshooting

- **No measurements received**: Ensure all devices are powered on and within range
- **UWB initialization timeout**: Check UWB Shield connections and power supply
- **Session initialization failed**: Verify session ID matches on all devices
- **Unknown anchor MAC**: Check that anchor MAC addresses match the tag's expected addresses

## Differences from Main DL-TDoA Setup

- Simplified 2-anchor configuration (vs. 3 anchors)
- Different session ID (0x87654321 vs. 0x12345678)
- Different MAC addresses to avoid conflicts
- Measurement display only (no position calculation)
- Test-focused configuration

