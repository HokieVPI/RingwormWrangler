# UWB Library Compatibility Check Report

## Executive Summary

This report documents compatibility issues between the DL-TDoA Arduino sketches and the PortentaUWBShield/StellaUWB libraries, with focus on errors that would prevent anchors from starting ranging.

**Critical Issues Found:** 3  
**Warnings:** 4  
**Total Issues:** 7

---

## Critical Issues (Prevent Ranging from Starting)

### 1. Double Initialization Problem ⚠️ CRITICAL

**Location:**
- `DL_TDoA_Anchor_Initiator.ino` lines 111, 117
- `DL_TDoA_Anchor_Responder1.ino` lines 101, 107
- `DL_TDoA_Anchor_Responder2.ino` lines 105, 111

**Issue:**
The `UWBDltdoaInitiator` and `UWBDltdoaResponder` constructors call `init()` internally (line 69 in both `UWBDltdoaInitiator.hpp` and `UWBDltdoaResponder.hpp`), but the sketches also call `init()` again after object creation.

**Root Cause:**
- Constructor calls `init()` which calls `UWBHAL.sessionInit()` and sets `sessionHdl`
- Sketch calls `init()` again, which attempts to initialize the same session ID again
- This may cause `SESSION_ACTIVE` or `INVALID_PHASE` errors

**Impact:**
- Session initialization may fail on the second `init()` call
- Even if it succeeds, the session handle may be inconsistent
- Ranging may fail to start due to session state issues

**Fix:**
Remove the redundant `init()` calls from all three anchor sketches. The constructor already initializes the session.

```cpp
// REMOVE these lines from all anchor sketches:
uwb::Status status = initiator.init();  // or responder.init()
if (status != uwb::Status::SUCCESS) {
  // error handling
}
```

---

### 2. Session Manager Creates New Session Object ⚠️ CRITICAL

**Location:**
- `DL_TDoA_Anchor_Initiator.ino` line 114
- `DL_TDoA_Anchor_Responder1.ino` line 104
- `DL_TDoA_Anchor_Responder2.ino` line 108

**Issue:**
`UWBSessionManager.addSession()` creates a **new** `UWBSession` object and only copies `sessionID` and `sessionType` (see `UWBSessionManager.cpp` lines 29-31). The original session object (with all configuration, session handle, app params, ranging params) is NOT stored.

**Root Cause:**
```cpp
// From UWBSessionManager.cpp line 29-31:
UWBSession *newSess= new UWBSession();  // Creates NEW empty session
newSess->sessionID(sess.sessionID());   // Only copies ID
newSess->sessionType(sess.sessionType()); // Only copies type
// All other configuration is LOST!
```

**Impact:**
- Session handle from constructor's `init()` is lost
- All app parameters (channel, preamble, frame config, etc.) are lost
- All ranging parameters (device role, device type, etc.) are lost
- When `start()` is called on the original object, it may work, but the session manager has a different object
- This creates inconsistency and potential memory/resource leaks

**Fix:**
The session manager should store a pointer to the original session object, not create a new one. However, since this is a library limitation, the workaround is to:
1. Call `init()` and `start()` on the original session object (not through session manager)
2. Only use session manager for tracking, not for session operations

**Note:** The current code pattern (calling `init()` and `start()` on the original object) may work, but the session manager's stored object is incomplete and could cause issues if accessed later.

---

### 3. Anchor Coordinates Not Transmitted to UWB Stack ⚠️ CRITICAL

**Location:**
- `UWBDltdoaInitiator.hpp` lines 60-64
- `UWBDltdoaResponder.hpp` lines 60-64

**Issue:**
The constructors check if coordinates are available but **never actually transmit them** to the UWB stack. The coordinates are stored in `UWBAnchorCoordinates` object but there's no code to set them via vendor params or HAL calls.

**Root Cause:**
```cpp
// From UWBDltdoaInitiator.hpp lines 60-64:
if(anchorCoordinates.areCoordinatesAvailable()) {
    // Coordinates may need to be set via vendor params or a different method
    // For now, we'll set basic parameters and coordinates may need manual configuration
    // NO ACTUAL CODE TO SET COORDINATES!
}
```

**Impact:**
- Anchors do not broadcast their positions
- Tags cannot calculate their position using trilateration
- The `anchor_location` field in DL-TDoA measurements will be empty or invalid

**Fix:**
Coordinates need to be transmitted via vendor parameters. The `UWBAnchorCoordinates` object has a `data[13]` array that should be sent as a vendor parameter. However, the exact vendor parameter ID for `DLTDOA_ANCHOR_LOCATION` is not clearly defined in the library headers.

**Workaround:**
This may require:
1. Finding the correct vendor parameter ID for anchor location
2. Adding code to set coordinates via `vendorParams.addOrUpdateParam()`
3. Or using a HAL-level API if available

---

## Warnings (May Cause Issues)

### 4. Empty UWBActiveRounds Configuration

**Location:**
- `DL_TDoA_Anchor_Initiator.ino` line 102
- `DL_TDoA_Anchor_Responder1.ino` line 98
- `DL_TDoA_Anchor_Responder2.ino` line 102

**Issue:**
`UWBActiveRounds rounds(1)` is created with capacity 1 but **no configs are added** via `addConfig()`. The `getSize()` returns 0.

**Root Cause:**
The constructor creates an empty array. The code that would use `rounds.getConfigs()` is commented out in the constructor (lines 72-79 in both initiator/responder headers).

**Impact:**
- Active ranging rounds may not be properly configured
- The commented-out `updateActiveRoundsAnchor()` call suggests this functionality may not be available in the HAL
- This may be acceptable if the UWB stack uses default round configuration

**Status:** May be acceptable - depends on whether the UWB stack requires explicit round configuration or uses defaults.

---

### 5. Tag Missing Device Role Configuration

**Location:**
- `DL_TDoA_Tag_Stella.ino` line 356

**Issue:**
The tag sketch comments out `deviceRole(uwb::DeviceRole::DL_TDOA_TAG)` and only sets `deviceType(uwb::DeviceType::CONTROLEE)`.

**Root Cause:**
StellaUWB library doesn't have a dedicated `UWBDltdoaTag` class like PortentaUWBShield does. The tag manually configures a `UWBSession`.

**Impact:**
- Tag may not be properly identified as a DL-TDoA tag
- UWB stack may not configure the tag correctly for passive DL-TDoA operation
- However, since tags are passive listeners, this may still work

**Fix:**
Uncomment and set the device role:
```cpp
tagSession.rangingParams.deviceRole(uwb::DeviceRole::DL_TDOA_TAG);
```

**Note:** Verify that `DeviceRole::DL_TDOA_TAG` exists in StellaUWB library (it should, as it's defined in `uwb_types.hpp`).

---

### 6. Frame Configuration Enum Usage

**Location:**
- Anchors: `UWBDltdoaInitiator.hpp` line 38 uses `uwb::RfFrameConfig::SP1`
- Tag: `DL_TDoA_Tag_Stella.ino` line 365 uses `uwb::RfFrameConfig::Sfd_Sts`

**Issue:**
Different enum values are used, though they have the same numeric value (1).

**Status:** **COMPATIBLE** - Both `SP1` and `Sfd_Sts` are defined as value 1 in `uwb_types.hpp`:
```cpp
Sfd_Sts = 1,
SP1 = Sfd_Sts,  // Same value
```

**Impact:** None - they are equivalent.

---

### 7. MAC Address List Not Used for Peer Configuration

**Location:**
- `UWBDltdoaInitiator.hpp` lines 46-49
- `UWBDltdoaResponder.hpp` lines 46-49

**Issue:**
Destination MAC addresses are added to `UWBMacAddressList`, but the constructor has an empty loop with a comment: "Note: PeerAddress typically supports single address, may need multiple calls or use a different method if multiple addresses are needed".

**Impact:**
- Destination addresses may not be properly configured
- Initiator may not know which responders to range with
- Responders may not know the initiator's address

**Status:** Unclear if this is a problem - the UWB stack may use the session ID and MAC addresses from ranging parameters instead.

---

## Configuration Parameter Verification

### ✅ Session ID
- All devices use `0x12345678` - **CONSISTENT**

### ✅ Channel
- All devices use channel `9` - **CONSISTENT**

### ✅ Preamble Code Index
- All devices use `10` - **CONSISTENT**

### ✅ SFD ID
- All devices use `0` - **CONSISTENT**

### ✅ Frame Configuration
- Anchors: `SP1` (value 1)
- Tag: `Sfd_Sts` (value 1)
- **COMPATIBLE** (same value)

### ✅ STS Configuration
- All devices use `StaticSts` - **CONSISTENT**

---

## API Compatibility Check

### ✅ All Required APIs Exist

1. **PortentaUWBShield:**
   - `UWBDltdoaInitiator` - ✅ Exists
   - `UWBDltdoaResponder` - ✅ Exists
   - `UWBAnchorCoordinates` - ✅ Exists
   - `UWBActiveRounds` - ✅ Exists
   - `UWBMacAddressList` - ✅ Exists
   - `buildScalar()` - ✅ Exists

2. **StellaUWB:**
   - `UWBSession` - ✅ Exists
   - `UWBRangingData` - ✅ Exists
   - `dlTdoaMeasure()` - ✅ Exists
   - `registerRangingCallback()` - ✅ Exists
   - `buildScalar()` - ✅ Exists
   - `DeviceRole::DL_TDOA_TAG` - ✅ Exists in `uwb_types.hpp`

### ✅ Enum Values Match

- `DeviceRole::DL_TDOA_ANCHOR` = 7 in both libraries
- `DeviceRole::DL_TDOA_TAG` = 8 in both libraries
- `RfFrameConfig::SP1` = `RfFrameConfig::Sfd_Sts` = 1 in both libraries

---

## Recommendations

### Immediate Fixes (Critical)

1. **Remove redundant `init()` calls** from all anchor sketches
2. **Fix session manager usage** - either fix the library or work around it
3. **Implement coordinate transmission** via vendor parameters

### Recommended Fixes (Warnings)

4. **Set device role** in tag sketch to `DL_TDOA_TAG`
5. **Verify active rounds configuration** - check if empty rounds cause issues
6. **Verify MAC address configuration** - ensure peer addresses are properly set

### Testing Recommendations

1. Test each anchor individually to verify initialization
2. Check serial output for error codes during `init()` and `start()`
3. Verify session handles are consistent
4. Test tag reception of anchor signals
5. Verify coordinate data in DL-TDoA measurements

---

## Summary of Errors That Prevent Ranging

### Primary Blockers:
1. **Double initialization** - May cause session state errors
2. **Session manager object mismatch** - Configuration lost, session handle inconsistent
3. **Missing coordinates** - Anchors don't broadcast position (prevents position calculation, but not ranging itself)

### Secondary Issues:
4. Empty active rounds (may be acceptable)
5. Missing device role in tag (may still work for passive listening)
6. MAC address list not used (unclear impact)

---

## File Locations Reference

### Anchor Sketches:
- `UWB/DL_TDoA_2D_Positioning/DL_TDoA_Anchor_Initiator/DL_TDoA_Anchor_Initiator.ino`
- `UWB/DL_TDoA_2D_Positioning/DL_TDoA_Anchor_Responder1/DL_TDoA_Anchor_Responder1.ino`
- `UWB/DL_TDoA_2D_Positioning/DL_TDoA_Anchor_Responder2/DL_TDoA_Anchor_Responder2.ino`

### Tag Sketch:
- `UWB/DL_TDoA_2D_Positioning/DL_TDoA_Tag_Stella/DL_TDoA_Tag_Stella.ino`

### Library Files:
- `UWB/Libraries/PortentaUWBShield/src/uwbapps/UWBDltdoaInitiator.hpp`
- `UWB/Libraries/PortentaUWBShield/src/uwbapps/UWBDltdoaResponder.hpp`
- `UWB/Libraries/PortentaUWBShield/src/uwbapps/UWBSessionManager.cpp`
- `UWB/Libraries/PortentaUWBShield/src/uwbapps/UWBSession.cpp`

---

**Report Generated:** Based on library analysis and code review  
**Libraries Analyzed:** PortentaUWBShield, StellaUWB-main  
**Status:** Ready for fixes

