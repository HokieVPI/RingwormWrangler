// SPDX-License-Identifier: MIT
// Copyright (c) 2025 Truesense Srl

#ifndef UWBULTDOASYNCANCHOR_HPP
#define UWBULTDOASYNCANCHOR_HPP

#include "UWB.hpp"
#include "UWBSession.hpp"
#include "UWBMacAddress.hpp"
#include "hal/uwb_types.hpp"

class UWBUltdoaSyncAnchor : public UWBSession {
public:     
    UWBUltdoaSyncAnchor(uint32_t session_ID, UWBMacAddress srcAddr)
    {
        sessionID(session_ID);
        sessionType(uwb::SessionType::RANGING);
        rangingParams.deviceRole(uwb::DeviceRole::UT_SYNC_ANCHOR);
        rangingParams.deviceType(uwb::DeviceType::CONTROLEE);
        rangingParams.multiNodeMode(uwb::MultiNodeMode::ONE_TO_MANY);
        rangingParams.rangingRoundUsage(uwb::RangingMethod::TDOA);
        rangingParams.scheduledMode(uwb::ScheduledMode::TIME_SCHEDULED);
        rangingParams.deviceMacAddr(srcAddr);

        appParams.frameConfig(uwb::RfFrameConfig::Sfd_Sts);
        appParams.stsConfig(uwb::StsConfig::StaticSts);
        appParams.addOrUpdateParam(buildScalar(uwb::AppConfigId::SessionInfoNtf, 1));
        appParams.sfdId(0);
        appParams.channel(9);
        appParams.preambleCodeIndex(10);
        appParams.macFcsType(0);
        appParams.noOfControlees(1);
        appParams.uplinkTdoaTimestamp(2);


    };
};

#endif /* UWBULTDOASYNCANCHOR */
