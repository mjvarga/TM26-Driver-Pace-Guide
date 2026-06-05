
// -----------------------------------------------------------------------------
//  PaceGuideData — logged values from the driver pace guide LED system
//  Tier: populated at MEDIUM tick (~10 ms) after paceGuide.update()
// -----------------------------------------------------------------------------
struct PaceGuideData {
    float    ratioRaw      = 1.0f;  // raw energy_remaining/dist_remaining ratio
    float    ratioSmoothed = 1.0f;  // EMA-smoothed ratio driving the LEDs
    float    cumDistM      = 0.0f;  // cumulative distance driven (metres)
    float    cumEnergyJ    = 0.0f;  // cumulative energy spent (joules)
    int      ledLevel      = 0;     // 0–16 LEDs lit (number equivalent of lights)
    bool     canStale      = false; // true when magenta is flashing (stale CAN)
    uint32_t lastUpdateMs  = 0;
};
