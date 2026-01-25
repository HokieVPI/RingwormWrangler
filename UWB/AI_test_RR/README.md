## AI_test_RR (Stella tag/controller + 3 Portenta anchors)

### Goal
Use **one Arduino Stella** as the **UWB ranging controller + tag** and **three Portenta C33 + UWB Shield** devices as **stationary anchors**. The Stella ranges all three anchors (one-to-many) and computes the tag’s **(x,y)** position from the three measured distances.  
The **only Serial output from the Stella** is a CSV line:

`x,y`

### Fixed configuration (per your request)
- **Tag (Stella) MAC**: `{0x11,0x11}`
- **Anchor MACs**:
  - Anchor 1: `{0x22,0x22}`
  - Anchor 2: `{0x33,0x33}`
  - Anchor 3: `{0x44,0x44}`
- **Anchor coordinates (meters)**:
  - Anchor 1: (0,0)
  - Anchor 2: (4,0)
  - Anchor 3: (2,2)
- **Session ID**: `0x11223344` (must match on all devices)

### Trilateration math used
Anchors:
- A1 = (0,0), radius r1
- A2 = (4,0), radius r2
- A3 = (2,2), radius r3

Solve:
- \(x = (r1^2 - r2^2 + 16)/8\)
- From A3-A1: \(x + y = (8 - r3^2 + r1^2)/4\)
- \(y = (8 - r3^2 + r1^2)/4 - x\)

### Distance units
The library’s `twr[j].distance` is a `uint16_t`. This project assumes it is **millimeters** and converts to meters with `0.001f`.  
If you observe obviously-scaled results (e.g., ~0.4 instead of ~4.0), change `DISTANCE_TO_METERS` in the Stella sketch to `0.01f` (centimeters) or your correct scale.

### Sketches
- `Stella_Controller_Tag/Stella_Controller_Tag.ino`
  - Runs `UWBRangingOneToMany` and prints only `x,y`.
- `Anchor1/Anchor1.ino`, `Anchor2/Anchor2.ino`, `Anchor3/Anchor3.ino`
  - Each runs `UWBRangingControlee` (responder) with destination MAC set to the Stella tag.
