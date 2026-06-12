// Scheduler.h — the raw scheduler interface (escape hatch / power path).
//
// A Scheduler is called once per base tick and fills the 16 triggers for every
// vehicle. This is the lowest-level contract; most users instead write a
// CorePolicy and use PolicyScheduler (see CorePolicy.h / PolicyScheduler.h),
// but a fully custom or data-driven scheduler (e.g. Challenge Q6) can subclass
// this directly.
#pragma once

#include <vector>

#include "fmu/FmuTypes.h"

namespace cps {

struct TaskSet;  // declared in sched/TaskModel.h

// Static configuration handed to the scheduler once at init.
struct SimConfig {
    int    nVehicles = 1;
    int    nCores    = 3;     // shared cloud cores
    double baseStep  = 1e-4;  // seconds per tick (FMU base step)
};

// Read-only per-vehicle snapshot the scheduler may use to make context-aware
// decisions (Challenge Q2). Rebuilt by the Simulation before each onTick.
//
// Information sets: `*_real` fields are ground truth -- a cloud scheduler that
// reads them is an ORACLE (upper bound only). The information legitimately
// available in-cloud is the estimator-derived set: e_y_est, rolling_remote,
// critical_remote, violated_remote.
struct VehicleView {
    int    id              = 0;
    double velocity        = 0.0;
    double e_y_real        = 0.0;
    double e_y_est         = 0.0;
    double rolling_real    = 0.0;
    double rolling_remote  = 0.0;
    double average_real    = 0.0;
    int    threshold_cntr_real = 0;
    bool   critical        = false;  // ground-truth critical section
    bool   violated_real   = false;
    bool   critical_remote = false;  // cloud's (estimated) view of the above
    bool   violated_remote = false;
    // Held-command predictions (PREDICTOR.md): time until |e_y| crosses the
    // hard bound (TTV) and until recovery becomes impossible (TTPNR), in ms.
    // Values at the prediction horizon (500 ms) mean "not within horizon";
    // ttpnr_ms == 0 means the car is past its (assumed-limit) point of no
    // return. Larger = less urgent, so policies can rank on these directly.
    double ttv_ms          = 500.0;
    double ttpnr_ms        = 500.0;
    // Clearance of the rescue available now: min (0.8 - |e_y|) over the
    // simulated recovery, meters (negative = the rescue fails by that depth;
    // 1e9 = not computed). AdaptiveGuard's tie-break among equal TTPNRs.
    double rescue_clearance_m = 1e9;
    // Recent latch-time command round-trip (ms, ~2 s window; -1 = none yet).
    // Drives the adaptive guard threshold.
    double age_recent_ms   = -1.0;
};

class Scheduler {
public:
    virtual ~Scheduler() = default;

    // Called once before the run. `taskSets[v]` is the task set for vehicle v.
    virtual void init(const SimConfig& /*cfg*/,
                      const std::vector<TaskSet>& /*taskSets*/) {}

    // Called every base tick. Fill out[v] (already sized to nVehicles, cleared)
    // with the triggers vehicle v should fire this tick.
    virtual void onTick(double t, long step,
                        const std::vector<VehicleView>& views,
                        std::vector<VehicleTriggers>& out) = 0;

    virtual const char* name() const { return "Scheduler"; }

    // Total jobs that missed their deadline (were dropped). Optional diagnostic.
    virtual long missedJobs() const { return 0; }

    // Worst-case end-to-end data age observed for `vehicle`, in base ticks; -1
    // if this scheduler does not track it. Optional diagnostic. Two merge
    // conventions are reported (see TaskChainModel): freshest-contributing
    // (reaction latency) and oldest-direct-input (the S->E->B->M->A path age
    // the analytical bound targets).
    virtual long maxDataAgeTicks(int /*vehicle*/) const { return -1; }
    virtual long maxDataAgeOldestTicks(int /*vehicle*/) const { return -1; }
    // Recent latch-time path age (~2 s window) — the live command round-trip
    // estimate consumed by adaptive policies. -1 if untracked / no latch yet.
    virtual long recentLatchAgeTicks(int /*vehicle*/, long /*step*/) const { return -1; }
};

}  // namespace cps
