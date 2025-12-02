// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBDLTDOARESPONDER
#define UWBDLTDOARESPONDER

#define DLTDOA_ANCHOR2_TIMEBASE_64BIT 0x02


#include "UWB.hpp"
#include "UWBSession.hpp"
#include "UWBAnchorCoordinates.hpp"
#include "UWBActiveRounds.hpp"
#include "UWBMacAddressList.hpp"

class UWBDltdoaResponder : public UWBSession {
public:     
    UWBDltdoaResponder(uint32_t session_ID, UWBMacAddress srcAddr, 
                       UWBAnchorCoordinates coords, UWBMacAddressList dstAddrs, 
                       UWBActiveRounds rounds)
        : anchorCoordinates(coords), destAddrs(dstAddrs) 
    {
        sessionID(session_ID);
        sessionType(uwb::SessionType::RANGING);
        
        // Set ranging parameters using HAL types
        rangingParams.setDeviceRole(uwb::DeviceRole::DL_TDOA_ANCHOR);
        rangingParams.setDeviceType(uwb::DeviceType::CONTROLLER);
        rangingParams.setMultiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
        rangingParams.setRangingRoundUsage(uwb::RangingRoundUsage::DL_TDOA);
        rangingParams.setScheduleMode(uwb::ScheduleMode::TIME_SCHEDULED);
        rangingParams.setDeviceMacAddr(srcAddr);
        
        // Add app parameters using HAL parameter IDs
        appParams.addOrUpdateParam(AppConfigId::RANGING_DURATION, AppParamType::U32, 200);
        appParams.addOrUpdateParam(AppConfigId::SLOTS_PER_RR, AppParamType::U32, 10);
        appParams.addOrUpdateParam(AppConfigId::SLOT_DURATION, AppParamType::U32, 1200);
        appParams.addOrUpdateParam(AppConfigId::RFRAME_CONFIG, AppParamType::U32, 
                                    static_cast<uint32_t>(uwb::RFrameConfig::SP1));
        appParams.addOrUpdateParam(AppConfigId::IN_BAND_TERMINATION_ATTEMPT_COUNT, 
                                    AppParamType::U32, 0);
        appParams.addOrUpdateParam(AppConfigId::CHANNEL_NUMBER, AppParamType::U32, 9);
        appParams.addOrUpdateParam(AppConfigId::DLTDOA_ANCHOR_CFO, AppParamType::U32, 1);

        // Handle coordinates
        if(anchorCoordinates.areCoordinatesAvailable()) {
            if(anchorCoordinates.isWGS84()) {
                appParams.addOrUpdateParam(AppConfigId::DLTDOA_ANCHOR_LOCATION, 
                                           AppParamType::ARRAY_U8,
                                           anchorCoordinates.data, 13);
            } else {
                appParams.addOrUpdateParam(AppConfigId::DLTDOA_ANCHOR_LOCATION,
                                           AppParamType::ARRAY_U8,
                                           anchorCoordinates.data, 11);
            }
        } else {
            appParams.addOrUpdateParam(AppConfigId::DLTDOA_ANCHOR_LOCATION,
                                       AppParamType::ARRAY_U8,
                                       anchorCoordinates.data, 1);
        }

        appParams.addOrUpdateParam(AppConfigId::DLTDOA_TX_ACTIVE_RANGING_ROUNDS, 
                                    AppParamType::U32, 1);
        appParams.addOrUpdateParam(AppConfigId::NO_OF_CONTROLEES, 
                                    AppParamType::U32, dstAddrs.getSize());
        appParams.addOrUpdateParam(AppConfigId::STS_CONFIG, AppParamType::U32,
                                    static_cast<uint32_t>(uwb::StsConfig::STATIC));
        appParams.addOrUpdateParam(AppConfigId::MULTI_NODE_MODE, AppParamType::U32,
                                    static_cast<uint32_t>(uwb::MultiNodeMode::ONE_TO_MANY));
        appParams.addOrUpdateParam(AppConfigId::DL_TDOA_HOP_COUNT, AppParamType::U32, 1);
        
        // Set destination addresses
        appParams.addOrUpdateParam(AppConfigId::DST_MAC_ADDRESS, AppParamType::ARRAY_U8,
                                    dstAddrs.getAllData(),
                                    dstAddrs.getTypeSize() * dstAddrs.getSize());
                                    
        appParams.addOrUpdateParam(AppConfigId::DLTDOA_TX_TIMESTAMP_CONF, 
                                    AppParamType::U32, DLTDOA_ANCHOR2_TIMEBASE_64BIT);

        uwb::Status status = init();
        if(status == uwb::Status::SUCCESS) {
            status = UWBHAL.updateActiveRoundsAnchor(sessionHdl, rounds.getSize(),
                                                   dstAddrs.getTypeSize() == 2 ? 
                                                   uwb::MacAddressMode::SHORT :
                                                   uwb::MacAddressMode::EXTENDED,
                                                   rounds.getConfigs(),
                                                   notActivatedRounds);
        }
    }

private:
    UWBAnchorCoordinates anchorCoordinates;
    UWBMacAddressList destAddrs;
    uint8_t* notActivatedRounds;
    uint8_t* roundsIndexList;
    uint8_t roundsIndexListSize;
};

#endif /* UWBDLTDOARESPONDER */
