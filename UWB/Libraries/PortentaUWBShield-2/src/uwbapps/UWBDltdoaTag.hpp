// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBDLTDOATAG_HPP
#define UWBDLTDOATAG_HPP


#include "UWB.hpp"
#include "UWBSession.hpp"

class UWBDltdoaTag : public UWBSession {
public:     
    UWBDltdoaTag(uint32_t session_ID, UWBMacAddress srcAddr, 
                 uint8_t rangingroundIndexList[], uint8_t rangingroundIndexListSize)
        : roundsIndexList(rangingroundIndexList),
          roundsIndexListSize(rangingroundIndexListSize) 
    {
        sessionID(session_ID);
        sessionType(uwb::SessionType::RANGING);
        
        // Set ranging parameters using HAL types
        rangingParams.setDeviceRole(uwb::DeviceRole::DL_TDOA_TAG);
        rangingParams.setDeviceType(uwb::DeviceType::CONTROLEE);
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
        appParams.addOrUpdateParam(AppConfigId::NO_OF_CONTROLEES, AppParamType::U32, 1);

        uwb::Status status = init();
        if(status != uwb::Status::SUCCESS) {
            status = updateActiveRoundsReceiver();
        }
    }

protected:
    uwb::Status updateActiveRoundsReceiver() {
        if(roundsIndexListSize > 0 && roundsIndexList != nullptr) {
            return UWBHAL.updateActiveRoundsReceiver(sessionHdl,
                                                   roundsIndexListSize,
                                                   roundsIndexList,
                                                   &notActivatedRounds);
        }
        return uwb::Status::FAILED;
    }

private:
    uint8_t notActivatedRounds;  // Changed from phNotActivatedRounds_t
    uint8_t* roundsIndexList;
    uint8_t roundsIndexListSize;
};

#endif /* UWBDLTDOATAG_HPP */
