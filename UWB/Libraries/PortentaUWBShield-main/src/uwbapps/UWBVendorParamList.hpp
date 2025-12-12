// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBVENDORPARAMLIST_HPP
#define UWBVENDORPARAMLIST_HPP

#include "UWBAppParamsList.hpp"
#include "hal/uwb_types.hpp"
#include "hal/uwb_hal.hpp"

class UWBVendorParamList: public UWBAppParamsList<uwb::VendorAppConfig, 
                                                 uwb::VendorAppConfigId,
                                                 uwb::AppParamType,
                                                 uwb::AppParamValue> 
{
public:
    // Constructor
    UWBVendorParamList() : UWBAppParamsList<uwb::VendorAppConfig, 
                                                 uwb::VendorAppConfigId,
                                                 uwb::AppParamType,
                                                 uwb::AppParamValue>() 
    
    {
        
    }
    
};

#endif /* UWBVENDORPARAMLIST */
