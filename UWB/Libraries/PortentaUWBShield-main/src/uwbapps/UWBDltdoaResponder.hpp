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
#include "UWBAppParamList.hpp"

class UWBDltdoaResponder : public UWBSession {
public:     
    UWBDltdoaResponder(uint32_t session_ID, UWBMacAddress srcAddr, 
                       UWBAnchorCoordinates coords, UWBMacAddressList dstAddrs, 
                       UWBActiveRounds rounds)
        : anchorCoordinates(coords), destAddrs(dstAddrs), initStatus(uwb::Status::NOT_INITIALIZED) 
    {
        sessionID(session_ID);
        sessionType(uwb::SessionType::RANGING);
        
        // Set ranging parameters using HAL types
        rangingParams.deviceRole(uwb::DeviceRole::DL_TDOA_ANCHOR);
        rangingParams.deviceType(uwb::DeviceType::CONTROLLER);
        rangingParams.multiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
        rangingParams.rangingRoundUsage(uwb::RangingMethod::DL_TDOA);
        rangingParams.scheduledMode(uwb::ScheduledMode::TIME_SCHEDULED);
        rangingParams.deviceMacAddr(srcAddr);
        
        // Add app parameters using HAL parameter IDs - use buildScalar/buildArray helpers
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::RangingDuration, 200));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SlotsPerRound, 10));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SlotDuration, 1200));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::RFrameConfig, 
                                    static_cast<uint32_t>(uwb::RfFrameConfig::SP1)));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::Channel, 9));

        // Handle coordinates - using the enum value we added
        if(anchorCoordinates.areCoordinatesAvailable()) {
            if(anchorCoordinates.isWGS84()) {
                appParams.addOrUpdateParam(buildArray(uwb::AppConfigId::DLTDOA_ANCHOR_LOCATION, 
                                                      anchorCoordinates.data, 13));
            } else {
                appParams.addOrUpdateParam(buildArray(uwb::AppConfigId::DLTDOA_ANCHOR_LOCATION,
                                                      anchorCoordinates.data, 11));
            }
        } else {
            appParams.addOrUpdateParam(buildArray(uwb::AppConfigId::DLTDOA_ANCHOR_LOCATION,
                                                  anchorCoordinates.data, 1));
        }

        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::NumControlees, dstAddrs.size()));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::StsConfig,
                                    static_cast<uint32_t>(uwb::StsConfig::StaticSts)));
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::MultiNode,
                                    static_cast<uint32_t>(uwb::MultiNodeMode::ONE_TO_MANY)));
        
        // Enable session info notifications (notification type 9 - SESSION_DATA)
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SessionInfoNtf, 1));
        
        // Set destination addresses
        appParams.addOrUpdateParam(buildArray(uwb::AppConfigId::PeerAddress,
                                    dstAddrs.getAllData(),
                                              dstAddrs.macTypeSize() * dstAddrs.size()));
                                    
        // Note: DL-TDoA specific parameters (DLTDOA_ANCHOR_CFO, DLTDOA_TX_ACTIVE_RANGING_ROUNDS,
        // DLTDOA_TX_TIMESTAMP_CONF, DL_TDOA_HOP_COUNT) are not in the AppConfigId enum.
        // They may be vendor-specific parameters or need to be added to the enum.
        // The coordinate transmission (DLTDOA_ANCHOR_LOCATION) is handled above.
                                    
        // Note: updateActiveRoundsAnchor() method doesn't exist in HAL, so this code is commented out
        // Active rounds configuration may not be needed for basic DL-TDoA operation
        initStatus = init();
        if (initStatus != uwb::Status::SUCCESS) {
            UWBHAL.Log_E("Session init failed in constructor: %d", (int)initStatus);
        }
    }

    /**
     * @brief Get the initialization status from the constructor
     * @return uwb::Status The status returned by init()
     */
    uwb::Status getInitStatus() const {
        return initStatus;
    }

private:
    UWBAnchorCoordinates anchorCoordinates;
    UWBMacAddressList destAddrs;
    uint8_t* notActivatedRounds;
    uint8_t* roundsIndexList;
    uint8_t roundsIndexListSize;
    uwb::Status initStatus;  // Store initialization status for error checking
};

#endif /* UWBDLTDOARESPONDER */
