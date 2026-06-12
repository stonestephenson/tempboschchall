// Recording.h — the run log produced by the simulation and consumed by the
// visualizer. Stores per-vehicle, decimated frames plus run metadata and a
// final per-vehicle summary. Can be saved/loaded so the viewer can replay a run
// without re-simulating.
#pragma once

#include <cstdint>
#include <string>
#include <vector>

namespace cps {

// One recorded sample for one vehicle. e_y is the lateral error; the actual
// world position is reconstructed by the viewer as trajectory.pointAt(refStep)
// + e_y * trajectory.normalAt(refStep).
struct Frame {
    float    t            = 0.0f;  // seconds
    uint32_t refStep      = 0;     // wrapped trajectory index for geometry lookup
    float    e_y_real     = 0.0f;
    float    e_y_est      = 0.0f;
    float    act          = 0.0f;  // applied steering command
    float    vel          = 0.0f;
    float    rolling_real = 0.0f;
    float    average_real = 0.0f;
    // v4: full physical state + aged predictions, so the prediction overlay
    // can be recomputed while scrubbing a replay (PREDICTOR.md). ttv/ttpnr
    // are -1 in recordings loaded from older formats ("no prediction data").
    float    phys[6]      = {};    // [yaw rate, slip, steer angle, steer rate, e_y, e_y_dot]
    float    ttv_ms       = -1.0f;
    float    ttpnr_ms     = -1.0f;
    uint8_t  flags        = 0;     // bit0 soft, bit1 hard, bit2 critical

    enum : uint8_t { kSoft = 1, kHard = 2, kCritical = 4 };
};

struct VehicleSummary {
    double average_real        = 0.0;
    double max_rolling_real    = 0.0;
    int    threshold_cntr_real = 0;
    double soft_violation_pct  = 0.0;  // % of run with |e_y| over soft bound
    int    hard_violations     = 0;    // recorded frames with |e_y| over hard bound
    double max_data_age_ms     = -1.0; // worst-case data age, freshest convention; -1 = none
    double max_data_age_oldest_ms = -1.0;  // oldest-direct (path) convention; -1 = none
    double min_ttpnr_ms        = -1.0; // closest call: run-min of finite TTPNR; -1 = never finite
    long   past_pnr_ticks      = 0;    // ticks spent past the predicted point of no return
};

struct RunRecording {
    // --- metadata ---
    int    profile        = 0;     // Profile enum as int
    int    nVehicles      = 0;
    int    nCores         = 3;
    double baseStep       = 1e-4;
    long   durationSteps  = 0;
    int    decimation     = 100;
    long   missedJobs     = 0;
    std::string schedulerName;
    std::vector<long> startOffsets;        // per vehicle
    // Format version this recording was loaded from (current version when
    // freshly recorded; not serialized). < 4 means frames carry no physical
    // state, so the replay prediction overlay is unavailable.
    int    loadedVersion  = 4;

    // --- data ---
    std::vector<std::vector<Frame>> frames;   // frames[vehicle][sample]
    std::vector<VehicleSummary>     summary;  // per vehicle

    int    frameCount() const { return frames.empty() ? 0 : static_cast<int>(frames[0].size()); }
    double duration()   const { return static_cast<double>(durationSteps) * baseStep; }

    void save(const std::string& path) const;
    static RunRecording load(const std::string& path);
};

}  // namespace cps
