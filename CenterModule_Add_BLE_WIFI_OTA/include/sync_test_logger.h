#ifndef SYNC_TEST_LOGGER_H
#define SYNC_TEST_LOGGER_H

#include <Arduino.h>
#include "imu_frame.h"

void initTimeSyncTestLogger();

void processTimeSyncTestRecord(
    const IMU_Node_Frame &frame,
    uint32_t centerReceiveTimeMs
);

void handleTimeSyncTestLogger();

#endif