// Predictor.h — harness-side plant model + held-command rollouts.
//
// Answers, for one vehicle: "if the actuator keeps holding its current (stale)
// command, when does |e_y| cross the hard 0.8 m bound (TTV), and when does
// recovery become impossible under an assumed steering limit (TTPNR)?"
// TTPNR <= TTV always; TTPNR is the physically-derived dynamic deadline the
// TimeToUnsafe scheduler ranks on, and both feed the prediction overlay in the
// visualizer. Full definitions, caveats and calibration: PREDICTOR.md.
//
// The plant model is a verbatim port of the FMU's dynamics (see Predictor.cpp
// provenance note) — the FMU itself is never touched. The steering limit
// exists ONLY here: the FMU's commanded steering is amplitude-unbounded, so
// "unrecoverable" is meaningful only relative to PredictParams::deltaMax.
#pragma once

#include <vector>

#include "trace/Trajectory.h"

namespace cps {

struct PredictParams {
    // |delta_des| cap assumed for recovery rollouts (rad). <= 0 means "use
    // defaultDeltaMax(profile)".
    double deltaMax = -1.0;
    long   horizonTicks         = 5000;   // 500 ms lookahead cap
    long   recoveryLatencyTicks = 40;     // fresh command pipeline delay (~4 ms BC chain)
    long   recoveryWindowTicks  = 10000;  // recovery must hold |e_y| < hard for 1 s
    long   pnrSearchStrideTicks = 50;     // TTPNR binary-search resolution (5 ms)
    int    vizStride            = 10;     // e_y polyline decimation (1 ms)
    bool   computePnr           = true;   // false: TTV/polyline only (validation mode)
    // Velocity grid for the shared matrix cache (m/s). The plant matrices are
    // recomputed per velocity change; quantizing to 0.01 m/s makes rollouts
    // ~10x cheaper at a sub-millimeter effect on predicted e_y. 0 = exact
    // (used by --validate-predictor so the fidelity gate checks the true model).
    double velQuantum           = 0.01;
};

// Result of one held-command prediction, relative to `fromStep` (the first
// not-yet-simulated tick; e_y[0] is the current error). ttvTicks/ttpnrTicks
// == horizonTicks means "not within the horizon".
struct Prediction {
    long fromStep    = 0;
    long ttvTicks    = 0;     // first tick with |e_y| >= 0.8 under held command
    long ttpnrTicks  = 0;     // last tick at which recovery can still begin
    bool pastPnr     = false; // recovery already impossible (ttpnrTicks == 0)
    long strideTicks = 10;    // ticks between consecutive e_y samples
    std::vector<float> e_y;   // predicted error every strideTicks ticks
};

// Per-profile default steering limit: max |act_out| over a clean N=1
// --exec worst lap, x1.5 margin. Values recorded in PREDICTOR.md.
double defaultDeltaMax(Profile p);

// Roll the plant forward from state x0 (the FMU state after the last
// simulated tick) holding the applied command `heldCmd`. Rollout step j uses
// the trajectory inputs (ff_ref, velocity) at fromStep + j + trajOffset,
// mirroring the FMU's per-tick input application. Computes TTV, the e_y
// polyline, and TTPNR (binary search over recovery start times; recovery =
// PD-flavored bang-bang steering at +-deltaMax through the model's own
// second-order steering dynamics — a heuristic, not certified reachability).
Prediction predictHold(const double x0[6], double heldCmd, long fromStep,
                       const Trajectory& traj, long trajOffset,
                       const PredictParams& params);

}  // namespace cps
