#pragma once
#include <Arduino.h>
#include "SensorData.h"

// =============================================================================
//  RowFormatter.h / .cpp
//
//  Single source of truth for the CSV row format.
//  Both DataLogger (writes to PSRAM buffer) and DebugSerial (writes to Serial)
//  call these functions — no duplicated format strings anywhere.
// =============================================================================
class RowFormatter {
public:
    static size_t formatCSVRow(char* buf, size_t bufSize,
                               uint32_t timestampMs,
                               const APPSData&             apps,
                               const BSPData&              bsp,
                               const SuspensionData&       susp,
                               const WheelSpeedData&       ws,
                               const RadiatorData&         rad,
                               const BrakeTempData&        brakeTemp,
                               const IMUData&              imu,
                               const BMSData&              bms,
                               const MotorControllerData&  mc,
                               const PaceGuideData&        pace);   // ← NEW

    static void printHumanReadable(Stream& out,
                                   uint32_t timestampMs,
                                   const APPSData&             apps,
                                   const BSPData&              bsp,
                                   const SuspensionData&       susp,
                                   const WheelSpeedData&       ws,
                                   const RadiatorData&         rad,
                                   const BrakeTempData&        brakeTemp,
                                   const IMUData&              imu,
                                   const BMSData&              bms,
                                   const MotorControllerData&  mc,
                                   const PaceGuideData&        pace);  // ← NEW

    static void printCSVHeader(Stream& out);
};
