#pragma once
#include <Arduino.h>
#include <FastLED.h>
#include "SensorData.h"

// =============================================================================
//  PaceGuide.h / .cpp
//
//  Drives two WS2812B LED sticks (8 LEDs each, 16 total) in series to give
//  the driver a real-time energy pace indication during the 22 km endurance run.
//
//  Logic:
//    - At race start, read actual pack SOC and compute available energy budget.
//      This means the guide adapts to whatever charge state the car starts in,
//      rather than blindly assuming a perfect full charge.
//
//    - Each loop, integrate speed → cumulative distance and
//      voltage × current → cumulative energy spent.
//
//    - Compute ratio = energy_remaining / distance_remaining
//      normalised so that 1.0 = exactly on pace:
//
//        ratio = (energy_remaining / total_energy) /
//                (distance_remaining / total_distance)
//
//        ratio > 1.0 → more energy left than distance remaining → surplus → push harder
//        ratio = 1.0 → exactly on pace
//        ratio < 1.0 → less energy left than distance remaining → deficit → slow down
//
//    - Ratio is smoothed with an exponential moving average to prevent LED
//      flickering caused by current spikes, regen transitions, or throttle
//      modulation.
//
//    - Map smoothed ratio to 0–16 LEDs:
//        LEDs  0– 3  → orange  (critical deficit  — slow down urgently)
//        LEDs  4– 7  → yellow  (deficit warning   — ease off)
//        LEDs  8–11  → green   (on pace            — hold steady)
//        LEDs 12–15  → blue    (surplus            — push harder)
//
//  Wiring:
//    Teensy PACE_LED_PIN → 300Ω resistor → Stick 1 IN
//    Stick 1 OUT → Stick 2 IN
//    5V supply → both sticks VCC (parallel)
//    5V supply GND → both sticks GND + Teensy GND (shared ground)
//
// =============================================================================

// ── Hardware config ───────────────────────────────────────────────────────────
#define PACE_LED_PIN        6       // Teensy data pin to first stick IN
#define PACE_NUM_LEDS       16      // 2 sticks × 8 LEDs
#define PACE_BRIGHTNESS     255     // full brightness for outdoor visibility

// ── LED update rate ───────────────────────────────────────────────────────────
// Energy integration runs at the MEDIUM tier (10 ms) for accuracy.
// LEDs only update at this slower interval — fast enough for the driver,
// slow enough to feel stable and not distracting.
#define PACE_LED_UPDATE_MS  100     // ms between LED strip updates

// ── Race constants ────────────────────────────────────────────────────────────
// FSAE endurance = 22 km (confirmed)
#define PACE_TOTAL_DISTANCE_M   22000.0f    // metres

// Full charge energy confirmed from datalog53.xlsx analysis.
// Scaled at startup by actual SOC so budget is always correct.
#define PACE_FULL_CHARGE_J      22000000.0f // 22 MJ

// ── Drivetrain constants ──────────────────────────────────────────────────────
// All confirmed from datalog53.xlsx and team lead
#define PACE_GEAR_RATIO         3.81f       // confirmed from Excel speed formula
#define PACE_WHEEL_RADIUS_M     0.203f      // confirmed from Excel speed formula
#define PACE_MOTOR_POLE_PAIRS   5           // confirmed by team lead

// ── Smoothing ─────────────────────────────────────────────────────────────────
// Exponential moving average weight for ratio smoothing.
// 0.9 = heavily smoothed (slow response), 0.5 = more responsive.
// Prevents flickering from current spikes, regen, and throttle modulation.
// Tune on track: if LEDs react too slowly increase toward 0.5,
//                if LEDs flicker too much increase toward 0.95.
#define PACE_EMA_ALPHA          0.9f

// ── Ratio → LED thresholds ────────────────────────────────────────────────────
// ratio = (energy_remaining_fraction) / (distance_remaining_fraction)
// ratio > 1.0 = surplus energy  → push harder  → more LEDs lit (into blue)
// ratio = 1.0 = exactly on pace → green zone
// ratio < 1.0 = energy deficit  → slow down    → fewer LEDs lit (orange)
//
// Tune these breakpoints after on-track testing.
#define PACE_DEFICIT_CRITICAL   0.70f   // below → 1–4 LEDs orange (urgent)
#define PACE_DEFICIT_WARNING    0.85f   // below → some yellow
#define PACE_DEFICIT_MILD       0.95f   // below → mild deficit zone
#define PACE_SURPLUS_MILD       1.05f   // above → mild surplus zone
#define PACE_SURPLUS_WARNING    1.15f   // above → into blue
#define PACE_SURPLUS_HIGH       1.30f   // above → all 16 LEDs lit

// ── CAN stale data timeout ────────────────────────────────────────────────────
// If BMS or MC data hasn't been updated in this many ms, flash magenta
// to alert the driver that the pace guide is operating on stale data.
#define PACE_CAN_TIMEOUT_MS     500

// =============================================================================
class PaceGuide {
public:
    // ── Lifecycle ─────────────────────────────────────────────────────────────

    // Call from setup() — initialises FastLED, reads starting SOC to set
    // energy budget, zeros accumulators.
    void begin(const BMSData& bms);

    // Call from loop() at MEDIUM tier (10 ms) — integrates distance & energy,
    // recomputes smoothed ratio. LEDs update at PACE_LED_UPDATE_MS rate.
    void update(const BMSData& bms, const MotorControllerData& mc);

    // Call at race start to zero accumulators and re-read SOC.
    // Wire a push button to a Teensy pin and call this on press.
    void reset(const BMSData& bms);

    // ── Accessors (useful for datalogging / debug) ─────────────────────────────
    float getRatio()               const { return _ratioSmoothed; }
    float getRatioRaw()            const { return _ratioRaw; }
    float getCumulativeDistanceM() const { return _cumDistM; }
    float getCumulativeEnergyJ()   const { return _cumEnergyJ; }
    int   getLedLevel()            const { return _ledLevel; }
    bool  isCanStale()             const { return _canStale; }

    // Populate a PaceGuideData struct for datalogging — call after update()
    void updateData(PaceGuideData& data) const;

private:
    // ── State ─────────────────────────────────────────────────────────────────
    float    _totalEnergyJ    = PACE_FULL_CHARGE_J; // set from SOC at begin/reset
    float    _cumDistM        = 0.0f;               // metres driven so far
    float    _cumEnergyJ      = 0.0f;               // joules spent so far
    float    _ratioRaw        = 1.0f;               // unsmoothed ratio
    float    _ratioSmoothed   = 1.0f;               // EMA-smoothed ratio
    int      _ledLevel        = 0;                  // 0–16 LEDs currently lit
    bool     _canStale        = false;              // true when magenta is flashing
    uint32_t _lastUpdateMs    = 0;                  // last integration timestamp
    uint32_t _lastLedUpdateMs = 0;                  // last LED push timestamp

    // ── Helpers ───────────────────────────────────────────────────────────────

    // Convert ERPM from VESC CAN frame to wheel speed in m/s
    float erpmToSpeedMs(int32_t erpmRaw) const;

    // Map smoothed ratio to number of LEDs to light (0–16)
    int   ratioToLedLevel(float ratio) const;

    // Return the colour for a given LED index (0–15)
    CRGB  ledColor(int index) const;

    // Push current _ledLevel to the physical strip
    void  updateStrip();

    // Flash magenta to indicate stale CAN data
    void  flashStale();

    // FastLED array
    CRGB _leds[PACE_NUM_LEDS];
};
