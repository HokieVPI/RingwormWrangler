// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBDLTDOATAG_HPP
#define UWBDLTDOATAG_HPP


#include "UWB.hpp"
#include "UWBSession.hpp"
#include "UWBAppParamList.hpp"

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
        rangingParams.deviceRole(uwb::DeviceRole::DL_TDOA_TAG);
        rangingParams.deviceType(uwb::DeviceType::CONTROLEE);
        rangingParams.multiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
        rangingParams.rangingRoundUsage(uwb::RangingMethod::DL_TDOA);
        rangingParams.scheduledMode(uwb::ScheduledMode::TIME_SCHEDULED);
        rangingParams.deviceMacAddr(srcAddr);
        
        // Add app parameters using HAL parameter IDs - use buildScalar helper
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::RangingDuration, 200));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SlotsPerRound, 10));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SlotDuration, 1200));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::RFrameConfig, 
                                    static_cast<uint32_t>(uwb::RfFrameConfig::SP1)));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::Channel, 9));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::NumControlees, 1));

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
