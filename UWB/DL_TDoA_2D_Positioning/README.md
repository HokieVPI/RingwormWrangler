# DL-TDoA 2D Positioning System

This example demonstrates a 2D positioning system using Downlink Time Difference of Arrival (DL-TDoA) with the PortentaUWB and StellaUWB libraries.

## Overview

The system consists of:
- **3 Anchors** with known positions that perform ranging sessions with each other
  - 1 Initiator Anchor (Anchor 1)
  - 2 Responder Anchors (Anchor 2 and Anchor 3)
- **1 Tag** that listens to the anchor ranging sessions and calculates its own position

## How DL-TDoA Works

1. The anchors perform ranging sessions with each other (initiator to responders)
2. The tag listens to these ranging sessions passively
3. The tag receives timestamps (TX and RX) from each anchor
4. Using the time differences of arrival and known anchor positions, the tag calculates its 2D position
5. The tag also receives Angle of Arrival (AoA) information from each anchor

## Setup Instructions

### Hardware Requirements
- 3x Arduino Portenta boards with UWB Shields (for anchors)
- 1x Arduino Stella board (for tag) OR 1x Arduino Portenta with UWB Shield
- USB cables for programming and power

### Software Requirements
- Arduino IDE with Portenta board support
- PortentaUWBShield library installed (for anchors)
- StellaUWB-main library installed (for Stella tag)
- ArduinoBLE-master library (optional, if BLE configuration is needed)

### Configuration

1. **Set Anchor Positions**: Edit the anchor position constants in each anchor script:
   - `DL_TDoA_Anchor_Initiator.ino`: Set `ANCHOR_X`, `ANCHOR_Y`, `ANCHOR_Z`
   - `DL_TDoA_Anchor_Responder1.ino`: Set `ANCHOR_X`, `ANCHOR_Y`, `ANCHOR_Z`
   - `DL_TDoA_Anchor_Responder2.ino`: Set `ANCHOR_X`, `ANCHOR_Y`, `ANCHOR_Z`

2. **Set MAC Addresses**: Ensure each device has a unique MAC address:
   - Initiator: `0xAA, 0xAA`
   - Responder 1: `0xBB, 0xBB`
   - Responder 2: `0xCC, 0xCC`
   - Tag: `0xDD, 0xDD`

3. **Set Session ID**: All devices must use the same session ID (`0x12345678`)

### Recommended Anchor Layout

For best results, place anchors in a non-collinear arrangement. Example:
- Anchor 1 (Initiator): (0, 0) meters
- Anchor 2 (Responder 1): (5, 0) meters
- Anchor 3 (Responder 2): (2.5, 4.33) meters (forms equilateral triangle)

## Uploading and Running

1. **Upload Anchor Scripts**:
   - Upload `DL_TDoA_Anchor_Initiator.ino` to the first Portenta board
   - Upload `DL_TDoA_Anchor_Responder1.ino` to the second Portenta board
   - Upload `DL_TDoA_Anchor_Responder2.ino` to the third Portenta board

2. **Upload Tag Script**:
   - For Portenta: Upload `DL_TDoA_Tag.ino` to the fourth Portenta board
   - For Stella: Upload `DL_TDoA_Tag_Stella/DL_TDoA_Tag_Stella.ino` to an Arduino Stella board
   - **Note**: If using Stella, you may need to enable DL-TDoA measurement access in the StellaUWB library (see DL_TDoA_Helper.h for instructions)

3. **Power On**:
   - Power on all anchors first and wait for them to initialize
   - Then power on the tag

4. **Monitor Output**:
   - Open Serial Monitor (115200 baud) on each device to see status and measurements
   - The tag will display calculated position and AoA information

## Understanding the Output

### Anchor Output
- Initialization status
- Session start confirmation
- LED blinking indicates active ranging

### Tag Output
- Received measurement count
- AoA (Angle of Arrival) from each anchor:
  - Azimuth angle (horizontal)
  - Elevation angle (vertical)
  - Figure of Merit (FOM) - quality indicator
- Calculated 2D position (x, y) in meters
- Distance to each anchor
- Comparison of measured vs calculated distances

## Position Calculation Algorithm

The tag uses trilateration to calculate its position:

1. **Time Difference Calculation**: Calculates time differences between signal arrivals from different anchors
2. **Distance Estimation**: Converts time differences to distance differences
3. **Trilateration**: Solves the system of equations:
   - (x - x₁)² + (y - y₁)² = d₁²
   - (x - x₂)² + (y - y₂)² = d₂²
   - (x - x₃)² + (y - y₃)² = d₃²

Where (x, y) is the tag position, (xᵢ, yᵢ) are anchor positions, and dᵢ are distances.

## Angle of Arrival (AoA)

The AoA information includes:
- **Azimuth**: Horizontal angle (0-360°)
- **Elevation**: Vertical angle (typically 0° for 2D positioning)
- **FOM (Figure of Merit)**: Quality indicator (lower is better)

The tag compares measured AoA with expected AoA based on calculated position.

## Troubleshooting

1. **No measurements received**: 
   - Check that all anchors are powered and initialized
   - Verify session IDs match
   - Ensure anchors are within range

2. **Position calculation fails**:
   - Verify anchor positions are correctly set
   - Ensure at least 3 valid measurements are received
   - Check that anchors are not collinear

3. **Inaccurate position**:
   - Verify anchor positions are accurate
   - Check for obstacles between tag and anchors
   - Ensure anchors are synchronized properly

## Notes

- The coordinate system uses meters. For centimeter precision, multiply values by 100.
- The system assumes 2D positioning (z = 0). For 3D, modify the position calculation algorithm.
- Timestamp units: 2^(-7) of 499.2 MHz chipping period ≈ 15.65 ps
- Speed of light: 299,792,458 m/s

## License

This example is provided as-is for educational and development purposes.

