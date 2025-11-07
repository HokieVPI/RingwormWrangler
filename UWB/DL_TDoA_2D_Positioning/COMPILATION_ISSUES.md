# DL-TDoA Compilation Issues

## Problem

The DL-TDoA classes in the PortentaUWBShield library (`UWBDltdoaInitiator`, `UWBDltdoaResponder`, `UWBDltdoaTag`) appear to have API mismatches with the actual library implementation.

## Issues Found

1. **phActiveRoundsConfig_t type not defined**: This type is referenced in `UWBActiveRounds.hpp` but is not defined in any visible header files. It may be defined in the HAL implementation.

2. **Method name mismatches in DL-TDoA classes**:
   - `setDeviceRole()` should be `deviceRole()`
   - `setDeviceType()` should be `deviceType()`
   - `setMultiNodeMode()` should be `multiNodeMode()`
   - `setRangingRoundUsage()` should be `rangingRoundUsage()`
   - `setScheduleMode()` should be `scheduledMode()`
   - `setDeviceMacAddr()` should be `deviceMacAddr()`

3. **Type/Namespace issues**:
   - `AppConfigId` should be `uwb::AppConfigId`
   - `AppParamType` should be `uwb::AppParamType`
   - `RangingRoundUsage` should be `RangingMethod`
   - `ScheduleMode` should be `ScheduledMode`
   - `StsConfig::STATIC` should be `StsConfig::StaticSts`
   - `MacAddressMode::EXTENDED` should be `MacAddressMode::LONG`

4. **Method name mismatches**:
   - `UWBMacAddressList::getSize()` should be `size()`
   - `UWBMacAddressList::getTypeSize()` should be `macTypeSize()`

5. **Missing HAL methods**:
   - `UWBHAL.updateActiveRoundsAnchor()` doesn't exist in the HAL interface

## Solutions

### Option 1: Fix the Library Classes (Recommended)

You need to fix the DL-TDoA classes in the library to match the actual API:

**File: `src/uwbapps/UWBDltdoaInitiator.hpp`**
- Change all `set*()` methods to the correct method names
- Add `uwb::` namespace to types
- Fix enum values
- Fix method calls on `UWBMacAddressList`

**File: `src/uwbapps/UWBDltdoaResponder.hpp`**
- Same fixes as above

**File: `src/uwbapps/UWBDltdoaTag.hpp`**
- Same fixes as above

### Option 2: Use Manual Session Configuration

Instead of using the DL-TDoA classes, manually configure a `UWBSession` with the correct parameters. This is more work but avoids the library class issues.

### Option 3: Contact Library Maintainers

The DL-TDoA functionality may be incomplete or use a different API. Contact the library maintainers for the correct way to use DL-TDoA.

## Current Status

The example scripts have been updated to remove direct use of `phActiveRoundsConfig_t`, but the library classes themselves still need to be fixed for the code to compile.

