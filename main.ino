// =============================================================================
//  main.ino — Formula SAE DAQ System
//  Teensy 4.1
//
//  Architecture:
//    SensorData.h       — all data structs
//    AnalogSensors      — APPS, BSP, linear pots, radiator thermistors
//    WheelSpeed         — 4x hall effect (interrupt-driven)
//    IMUSensor          — ISM330DHCX + MMC5983MA via I2C
//    BrakeTempSensor    — 4x MLX90614 via I2C
//    CANInterface       — BMS + MC CAN frames
//    DataLogger         — SD card wide-format CSV
//    DashOutput         — Nextion display + nRF24 TX
//    PaceGuide          — 16x WS2812B LED pace bar (2 sticks in series)
//
//  Sampling tiers:
//    FAST    (1  ms)  — APPS, linear pots
//    MEDIUM  (10 ms)  — BSP, IMU accel/gyro, wheel speed compute, PACE GUIDE
//    SLOW    (100 ms) — brake temps, radiator, IMU mag, display data cache
//    LOG     (50 ms)  — write one CSV row with all latest values
//    DISPLAY (200 ms) — Nextion + nRF24 push
//    CAN              — async, drained every loop iteration
//    WHEEL SPEED      — async ISR pulse count, speed computed at MEDIUM
// =============================================================================

// To enable debug serial output, uncomment the line below.
// When commented out, all debug calls compile to nothing — zero overhead.
//#define DEBUG_SERIAL

#include "SensorData.h"
#include "AnalogSensors.h"
#include "WheelSpeed.h"
#include "IMUSensor.h"
#include "BrakeTempSensor.h"
#include "CANInterface.h"
#include "DataLogger.h"
#include "DashOutput.h"
#include "DebugSerial.h"
#include "PaceGuide.h"      // ← NEW

// =============================================================================
//  Global data structs — one instance of each, passed by reference everywhere
// =============================================================================
APPSData            apps;
BSPData             bsp;
SuspensionData      susp;
WheelSpeedData      wheelSpeed;
RadiatorData        radiator;
BrakeTempData       brakeTemp;
IMUData             imu;
BMSData             bms;
MotorControllerData mc;
PaceGuideData       paceData;       // ← NEW

// =============================================================================
//  Module instances
// =============================================================================
AnalogSensors   analogSensors;
WheelSpeed      wheelSpeedSensor;
IMUSensor       imuSensor;
BrakeTempSensor brakeTempSensor;
CANInterface    canInterface;
DataLogger      dataLogger;
DashOutput      dashOutput;
DebugSerial     debugSerial;
PaceGuide       paceGuide;              // ← NEW

// =============================================================================
//  Sampling timers
// =============================================================================
SampleTimer timerFast    = { 1   };   //   1 ms — APPS, suspension
SampleTimer timerMedium  = { 10  };   //  10 ms — BSP, IMU accel/gyro, wheel speed, pace guide
SampleTimer timerSlow    = { 100 };   // 100 ms — brake temps, radiator, IMU mag
SampleTimer timerLog     = { 10  };   //  10 ms — CSV row write (100 Hz)
SampleTimer timerDisplay = { 200 };   // 200 ms — Nextion + nRF24

uint32_t startTimeMs = 0;

// =============================================================================
//  setup()
// =============================================================================
void setup() {
    Serial.begin(115200);
    delay(500); // give Serial time to enumerate before first prints

    Serial.println("=== DAQ System Init ===");

    // analogSensors.begin();
    // Serial.println("[Analog] OK");

    // wheelSpeedSensor.begin();
    // WheelSpeed::setData(&wheelSpeed);
    // Serial.println("[WheelSpeed] Interrupts attached");

    bool imuOk = imuSensor.begin();
    Serial.println(imuOk ? "[IMU] OK" : "[IMU] WARNING — check wiring");

    // bool btOk = brakeTempSensor.begin();
    // Serial.println(btOk ? "[BrakeTemp] OK" : "[BrakeTemp] WARNING — check I2C addresses");

    //canInterface.begin();

    dataLogger.begin();
    dashOutput.begin();
    Serial.println("[Dash] OK");

    debugSerial.begin();

    // ── Pace Guide ────────────────────────────────────────────────────────────
    // begin() reads bms.packSOC to set the energy budget, so ideally CAN has
    // had a moment to populate the BMS struct before this runs.
    // If CAN isn't connected yet during bench testing, it defaults to 100% SOC.
    paceGuide.begin(bms);
    Serial.println("[PaceGuide] OK");
    // ─────────────────────────────────────────────────────────────────────────

    startTimeMs = millis();
    Serial.println("=== Init complete ===");
}

// =============================================================================
//  loop()
// =============================================================================
void loop() {
    uint32_t now = millis();

    // -------------------------------------------------------------------------
    //  DataLogger state machine — must run every iteration to catch switch edges
    // -------------------------------------------------------------------------
    dataLogger.tick();
    dashOutput.updateLogIndicator(dataLogger.state());

    // -------------------------------------------------------------------------
    //  ASYNC — CAN frames (drain every iteration, no rate limiting)
    // -------------------------------------------------------------------------
    //canInterface.processFrames(bms, mc);

    // -------------------------------------------------------------------------
    //  FAST tier — 1 ms
    // -------------------------------------------------------------------------
    // if (timerFast.ready(now)) {
    //     analogSensors.updateAPPS(apps);
    //     analogSensors.updateSuspension(susp);
    // }

    // -------------------------------------------------------------------------
    //  MEDIUM tier — 10 ms
    // -------------------------------------------------------------------------
    if (timerMedium.ready(now)) {
        //analogSensors.updateBSP(bsp);
        imuSensor.updateAccelGyro(imu);
        //wheelSpeedSensor.update(wheelSpeed);

        // ── Pace Guide update ─────────────────────────────────────────────────
        // Runs at 10 ms — fast enough to integrate speed/energy accurately,
        // slow enough not to thrash the LED strip.
        paceGuide.update(bms, mc);
        paceGuide.updateData(paceData);  // populate struct for datalogging
        // ─────────────────────────────────────────────────────────────────────
    }

    // -------------------------------------------------------------------------
    //  SLOW tier — 100 ms
    // -------------------------------------------------------------------------
    if (timerSlow.ready(now)) {
        //analogSensors.updateRadiator(radiator);
        //brakeTempSensor.update(brakeTemp);
        imuSensor.updateMag(imu);
    }

    // -------------------------------------------------------------------------
    //  LOG tier — 10 ms — write one wide CSV row
    // -------------------------------------------------------------------------
    if (timerLog.ready(now)) {
        dataLogger.writeRow(
            now - startTimeMs,
            apps, bsp, susp, wheelSpeed,
            radiator, brakeTemp, imu, bms, mc, paceData
        );
        debugSerial.update(
            now - startTimeMs,
            apps, bsp, susp, wheelSpeed,
            radiator, brakeTemp, imu, bms, mc, paceData
        );
    }

    // -------------------------------------------------------------------------
    //  DISPLAY tier — 200 ms — Nextion + nRF24
    // -------------------------------------------------------------------------
    if (timerDisplay.ready(now)) {
        dashOutput.update(mc, bms, bsp);
    }
}
