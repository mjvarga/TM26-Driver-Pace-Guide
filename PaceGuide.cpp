#include "PaceGuide.h"
#include <math.h>

// =============================================================================
//  PaceGuide.cpp
// =============================================================================

// -----------------------------------------------------------------------------
//  begin() — call once from setup()
// -----------------------------------------------------------------------------
void PaceGuide::begin(const BMSData& bms) {
    FastLED.addLeds<WS2812B, PACE_LED_PIN, GRB>(_leds, PACE_NUM_LEDS);
    FastLED.setBrightness(PACE_BRIGHTNESS);

    fill_solid(_leds, PACE_NUM_LEDS, CRGB::Black);
    FastLED.show();

    _raceStartMs = millis(); // initialise before reset() so elapsed time is valid
    reset(bms);

    Serial.print("[PaceGuide] Init OK. Energy budget: ");
    Serial.print(_totalEnergyJ / 1000000.0f, 2);
    Serial.println(" MJ");
}

// -----------------------------------------------------------------------------
//  reset() — call at race start (or restart)
//  Re-reads SOC so the budget always reflects the actual charge state.
// -----------------------------------------------------------------------------
void PaceGuide::reset(const BMSData& bms) {
    // Scale full-charge energy by actual starting SOC.
    // e.g. 95% SOC → budget = 0.95 × 22 MJ = 20.9 MJ
    float startSOC = constrain(bms.packSOC, 0.0f, 100.0f);
    _totalEnergyJ = (startSOC / 100.0f) * PACE_FULL_CHARGE_J;

    _cumDistM        = 0.0f;
    _cumEnergyJ      = 0.0f;
    _ratioRaw        = 1.0f;    // start at "on pace" — avoids false alarm at line
    _ratioSmoothed   = 1.0f;
    _ledLevel        = 0;
    _lastUpdateMs    = millis();
    _lastLedUpdateMs = millis();
    _raceStartMs     = millis(); // start elapsed-time clock for power-target ratio

    fill_solid(_leds, PACE_NUM_LEDS, CRGB::Black);
    FastLED.show();

    Serial.print("[PaceGuide] Reset. SOC=");
    Serial.print(startSOC, 1);
    Serial.print("% → Budget=");
    Serial.print(_totalEnergyJ / 1000000.0f, 2);
    Serial.println(" MJ");
}

// -----------------------------------------------------------------------------
//  update() — call at MEDIUM tier (every 10 ms)
//
//  Integrates distance and energy every call for accuracy.
//  LEDs only update every PACE_LED_UPDATE_MS (100 ms) for visual stability.
//  Checks for stale CAN data and flashes magenta if timeout exceeded.
// -----------------------------------------------------------------------------
void PaceGuide::update(const BMSData& bms, const MotorControllerData& mc) {
    uint32_t nowMs = millis();

    // ── Stale CAN data check ──────────────────────────────────────────────────
    // If BMS or MC haven't sent a frame recently, alert the driver.
    bool bmsStale = (nowMs - bms.lastUpdateMs)  > PACE_CAN_TIMEOUT_MS;
    bool mcStale  = (nowMs - mc.lastUpdateMs)   > PACE_CAN_TIMEOUT_MS;

    if (bmsStale || mcStale) {
        _canStale = true;
        // Flash magenta every 250 ms to signal stale data
        if ((nowMs / 250) % 2 == 0) {
            fill_solid(_leds, PACE_NUM_LEDS, CRGB(80, 0, 80)); // magenta
        } else {
            fill_solid(_leds, PACE_NUM_LEDS, CRGB::Black);
        }
        FastLED.show();
        _lastUpdateMs = nowMs; // keep timestamp current so dt is valid on recovery
        return;
    }

    // Guard: skip first call (no valid dt yet)
    if (_lastUpdateMs == 0) {
        _lastUpdateMs = nowMs;
        return;
    }

    float dtS = (nowMs - _lastUpdateMs) / 1000.0f;
    _lastUpdateMs = nowMs;
    _canStale = false; // data is fresh

    // ── 1. Integrate distance ─────────────────────────────────────────────────
    float speedMs = erpmToSpeedMs(mc.erpmRaw);
    _cumDistM += speedMs * dtS;

    // ── 2. Integrate energy ───────────────────────────────────────────────────
    // packVoltage raw * 0.1 = V,  packCurrent raw * 0.1 = A
    // Power (W) = V × A
    // fabsf on current: regen (negative current) also costs energy budget
    float voltageV = bms.packVoltage * 0.1f;
    float currentA = bms.packCurrent * 0.1f;
    float powerW   = voltageV * fabsf(currentA);
    _cumEnergyJ   += powerW * dtS;

    // ── 3. Compute pace ratio ─────────────────────────────────────────────────
    // ratio = target_energy_so_far / actual_energy_so_far
    //
    // target_energy = _targetPowerW × elapsed_seconds
    //   (i.e. how many joules a perfectly-paced car would have spent by now)
    //
    // ratio > 1.0 → spending LESS than target rate → energy surplus → push harder
    // ratio = 1.0 → exactly on pace
    // ratio < 1.0 → spending MORE than target rate → energy deficit → slow down
    //
    // This approach only requires a clock — no GPS or wheel speed needed for
    // the pace judgement itself. Distance integration is still logged for
    // reference. We guard on _cumEnergyJ > 0 to avoid divide-by-zero at
    // the very start of the run.
    float elapsedS = (nowMs - _raceStartMs) / 1000.0f;

    if (_cumEnergyJ > 0.0f && elapsedS > 1.0f) {
        float targetEnergy = _targetPowerW * elapsedS;

        _ratioRaw = targetEnergy / _cumEnergyJ;

        // ── 4. Exponential moving average smoothing ───────────────────────────
        // Prevents LED flickering from current spikes, regen, and throttle blips.
        // _ratioSmoothed = α × old + (1-α) × new
        _ratioSmoothed = PACE_EMA_ALPHA * _ratioSmoothed +
                         (1.0f - PACE_EMA_ALPHA) * _ratioRaw;
    }

    // ── 5. Update LEDs at display rate (not every 10 ms) ─────────────────────
    if (nowMs - _lastLedUpdateMs >= PACE_LED_UPDATE_MS) {
        _lastLedUpdateMs = nowMs;
        _ledLevel = ratioToLedLevel(_ratioSmoothed);
        updateStrip();
    }
}

// -----------------------------------------------------------------------------
//  erpmToSpeedMs()
//  Converts raw ERPM from the VESC CAN frame to wheel speed in m/s.
// -----------------------------------------------------------------------------
float PaceGuide::erpmToSpeedMs(int32_t erpmRaw) const {
    if (erpmRaw == 0) return 0.0f;

    // ERPM → mechanical RPM (divide by pole pairs)
    float rpm = (float)abs(erpmRaw) / (float)PACE_MOTOR_POLE_PAIRS;

    // Shaft RPM → wheel RPM (divide by gear ratio)
    float wheelRpm = rpm / PACE_GEAR_RATIO;

    // Wheel RPM → m/s:  v = ω × r = (rpm × 2π / 60) × r
    return (wheelRpm * 2.0f * PI * PACE_WHEEL_RADIUS_M) / 60.0f;
}

// -----------------------------------------------------------------------------
//  ratioToLedLevel()
//  Maps smoothed ratio to number of LEDs lit (0–16).
//
//  ratio > 1.0 = energy surplus  → more LEDs lit → push harder
//  ratio = 1.0 = on pace         → green zone (centre of strip)
//  ratio < 1.0 = energy deficit  → fewer LEDs lit → slow down
//
//  LED layout:
//    LEDs  0– 3  orange  critical deficit  (slow down urgently)
//    LEDs  4– 7  yellow  deficit warning   (ease off)
//    LEDs  8–11  green   on pace           (hold steady)
//    LEDs 12–15  blue    surplus           (push harder)
// -----------------------------------------------------------------------------
int PaceGuide::ratioToLedLevel(float ratio) const {
    // Before race start (insufficient elapsed time) — show nothing
    float elapsedS = (millis() - _raceStartMs) / 1000.0f;
    if (elapsedS <= 1.0f || _cumEnergyJ <= 0.0f) return 0;

    // Deficit side: fewer LEDs = more urgent to slow down
    if (ratio <= PACE_DEFICIT_CRITICAL)  return 1;   // 1 orange  — critical
    if (ratio <= PACE_DEFICIT_WARNING)   return 4;   // orange full
    if (ratio <= PACE_DEFICIT_MILD)      return 7;   // into yellow
    if (ratio <= 1.00f)                  return 9;   // mild deficit, low green

    // Surplus side: more LEDs = more energy to spend
    if (ratio <= PACE_SURPLUS_MILD)      return 11;  // mild surplus, upper green
    if (ratio <= PACE_SURPLUS_WARNING)   return 13;  // into blue
    if (ratio <= PACE_SURPLUS_HIGH)      return 15;  // most of blue
    return 16;                                        // all lit — large surplus
}

// -----------------------------------------------------------------------------
//  ledColor()
//  Returns the colour for a given LED index using the dark outdoor palette.
// -----------------------------------------------------------------------------
CRGB PaceGuide::ledColor(int index) const {
    if (index <= 3)  return CRGB(122, 58,  5);    // dark orange — deficit
    if (index <= 7)  return CRGB(110, 82,  12);   // dark yellow — warning
    if (index <= 11) return CRGB(30,  56,  9);    // dark green  — on pace
    return               CRGB(12,  49,  86);       // dark blue   — surplus
}

// -----------------------------------------------------------------------------
//  updateStrip()
//  Lights the first _ledLevel LEDs in their zone colours, turns the rest off.
// -----------------------------------------------------------------------------
void PaceGuide::updateStrip() {
    for (int i = 0; i < PACE_NUM_LEDS; i++) {
        _leds[i] = (i < _ledLevel) ? ledColor(i) : CRGB::Black;
    }
    FastLED.show();
}

// -----------------------------------------------------------------------------
//  updateData() — populate a PaceGuideData struct for datalogging
//  Call this after update() in the main loop, then pass to dataLogger.writeRow()
// -----------------------------------------------------------------------------
void PaceGuide::updateData(PaceGuideData& data) const {
    data.ratioRaw      = _ratioRaw;
    data.ratioSmoothed = _ratioSmoothed;
    data.cumDistM      = _cumDistM;
    data.cumEnergyJ    = _cumEnergyJ;
    data.ledLevel      = _ledLevel;
    data.canStale      = _canStale;
    data.lastUpdateMs  = _lastUpdateMs;
}
