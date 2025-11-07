/**
 * Helper header to access DL-TDoA measurements in StellaUWB
 * 
 * This header provides a workaround to access DL-TDoA measurement data
 * since the dlTdoaMeasure() function is commented out in the StellaUWB library.
 * 
 * To use this, you may need to:
 * 1. Uncomment the dltdoa array in uwb_types.hpp RangingMeasurements union
 * 2. Uncomment the dlTdoaMeasure() function in UWBRangingData.hpp and .cpp
 * 
 * Alternatively, this provides a way to access the data if you can get a pointer
 * to the RangingResult structure.
 */

#ifndef DL_TDOA_HELPER_H
#define DL_TDOA_HELPER_H

#include "uwbapps/UWBRangingData.hpp"
#include "hal/uwb_types.hpp"

// Helper function to get DL-TDoA measurements
// Note: This requires access to the private 'result' member of UWBRangingData
// You may need to make it a friend function or modify the library

// Alternative: Create a derived class or modify UWBRangingData to add this method
// For now, we'll provide a template that shows how to access the data

// If you can modify UWBRangingData, add this public method:
/*
const RangingMesrDlTdoas UWBRangingData::dlTdoaMeasure() const {
    return result.measurements.dltdoa;
}
*/

// And uncomment in UWBRangingData.hpp:
/*
const RangingMesrDlTdoas dlTdoaMeasure() const;
*/

// Also need to uncomment in hal/uwb_types.hpp RangingMeasurements union:
/*
dltdoa_mesr dltdoa[MAX_TDOA_MEASURES];
*/

#endif // DL_TDOA_HELPER_H

