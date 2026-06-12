// Simulation.h — the co-simulation master. Owns the FMU instances and the
// scheduler, drives the whole system one base tick (0.1 ms) at a time, and
// records decimated frames for the visualizer / metrics.
//
// step() advances exactly one tick, so the visualizer can drive it live by
// calling step() many times per rendered frame; runToCompletion() loops it for
// a headless run.
#pragma once

#include <cstdint>
#include <memory>
#include <vector>

#include "fmu/Fmu.h"
#include "sched/Scheduler.h"
#include "sched/TaskModel.h"
#include "sim/Predictor.h"
#include "sim/Recording.h"
#include "sim/Vehicle.h"
#include "trace/Trajectory.h"

namespace cps {

struct SimParams {
    Profile  profile       = Profile::V10;
    int      nVehicles     = 1;
    int      nCores        = 3;
    long     durationSteps = 0;   // 0 -> one full lap of the chosen profile
    int      decimation    = 100; // record one frame per N ticks (100 == 10 ms)
    std::vector<long> startOffsets;  // empty -> spread vehicles evenly around lap
    ExecMode execMode      = ExecMode::Average;
    OverrunPolicy overrun  = OverrunPolicy::KillAndHold;
    // If >= 0, override BOTH network delays with this fixed value (ms),
    // regardless of execMode (which then governs task execution times only).
    // For the zone-tolerance delay sweeps (ZONE_TOLERANCE.md).
    double   netDelayMs    = -1.0;
    // Predictor fidelity gate: at every actuator latch, predict e_y over the
    // upcoming hold with the ported plant model and compare against the FMU's
    // realized e_y. Prints max |deviation| at the end (PASS < 1e-6 m).
    bool     validatePredictor = false;
    // Assumed steering limit for recoverability predictions (rad);
    // <= 0 uses the calibrated per-profile default (defaultDeltaMax).
    double   deltaMax      = -1.0;
    // TimeToUnsafe policy behavior for past-PNR cars: false (default) =
    // maximum urgency (never give up); true = triage (drop to lowest).
    bool     triage        = false;
    uint64_t seed          = 0;
};

class Simulation {
public:
    Simulation(const SimParams& params, std::unique_ptr<Scheduler> scheduler,
               std::shared_ptr<FmuLibrary> lib = nullptr);

    void start();                 // load trajectory, instantiate FMUs, init scheduler
    bool step();                  // advance one base tick; false once finished
    void runToCompletion(bool verbose = true);

    long   currentStep()   const { return step_; }
    long   durationSteps() const { return durationSteps_; }
    bool   finished()      const { return started_ && step_ >= durationSteps_; }
    double progress()      const { return durationSteps_ ? double(step_) / durationSteps_ : 1.0; }

    const RunRecording&         recording()  const { return rec_; }
    std::shared_ptr<Trajectory> trajectory() const { return traj_; }

    // Latest cached held-command prediction for a vehicle (refreshed every
    // kPredictRefreshTicks; see currentPredTicks for aged TTV/TTPNR values).
    const Prediction& prediction(int vehicle) const { return predCache_[vehicle]; }

private:
    void buildViews();
    void recordFrame(double t);
    void finalizeSummary();

    SimParams                    params_;
    std::unique_ptr<Scheduler>   scheduler_;
    std::shared_ptr<FmuLibrary>  lib_;
    std::shared_ptr<Trajectory>  traj_;
    std::vector<Vehicle>         vehicles_;
    std::vector<long>            offsets_;
    std::vector<VehicleView>     views_;
    std::vector<VehicleTriggers> triggers_;
    RunRecording                 rec_;

    double dt_            = 1e-4;
    long   step_          = 0;
    long   durationSteps_ = 0;
    bool   started_       = false;
    bool   finalized_     = false;

    std::vector<double> maxRolling_;
    std::vector<int>    hardCount_;

    // --- Held-command prediction cache (drives TTU policy, viz, stats).
    //     TTV + polyline refresh every 10 ms (cheap single rollout); the PNR
    //     binary search (the expensive part) every 50 ms; both age in between. ---
    static constexpr long kPredictRefreshTicks = 100;  // 10 ms, like the metrics
    // PNR every 10 ms too: warm-started searches (PredictParams.
    // warmStartTtpnrTicks) make this affordable; gate is >= 10x at 12 veh.
    static constexpr long kPnrRefreshTicks     = 100;  // 10 ms
    void refreshPredictions(bool withPnr);
    // Aged (ttv, ttpnr) in ticks as of the current step_: cached values minus
    // elapsed ticks, clamped at 0; horizon sentinel passes through unchanged.
    void currentPredTicks(int vehicle, long& ttv, long& ttpnr) const;
    PredictParams           predParams_;
    std::vector<Prediction> predCache_;      // latest TTV + polyline
    std::vector<long>       predBaseStep_;   // step_ at which each cache entry was made
    std::vector<long>       ttpnrTicks_;     // latest PNR result (separate cadence)
    std::vector<long>       ttpnrBaseStep_;
    std::vector<double>     minTtpnrMs_;     // run-min of finite aged TTPNR (-1 = none)
    std::vector<long>       pastPnrTicks_;   // ticks spent at aged TTPNR == 0

    // --- Predictor fidelity gate state (params_.validatePredictor) ---
    void validatePredictions();
    struct PendingValidation {
        Prediction pred;
        long       madeAtStep = -1;  // pred.e_y[j] = e_y after tick madeAtStep + j
        bool       active     = false;
    };
    std::vector<PendingValidation> pendingVal_;
    double valMaxDev_   = 0.0;
    long   valHolds_    = 0;
    long   valSamples_  = 0;
    double valMaxAct_   = 0.0;  // max |act_out| seen (delta-max calibration aid)
};

}  // namespace cps
