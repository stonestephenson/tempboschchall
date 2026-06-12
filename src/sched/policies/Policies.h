// Policies.h — factories for the built-in CorePolicy implementations.
//
// Each policy lives in its own translation unit and exposes a factory here, so
// main.cpp can select one without seeing the class definition. To add your own
// scheduling method: create sched/policies/MyPolicy.cpp defining a CorePolicy
// subclass + a makeMyPolicy() factory, declare it here (or in main.cpp), and
// pass it to PolicyScheduler.
#pragma once

#include <cmath>
#include <memory>

#include "sched/CorePolicy.h"

namespace cps {

// Shared comfort-urgency score ("how badly does this vehicle need compute
// right now"), defined once so every policy that ranks on it (ContextAware,
// Hybrid) provably uses the identical rule — A/B comparisons then isolate
// the policy mechanism, never an accidentally divergent score.
inline double comfortUrgencyOracle(const VehicleView& v) {
    return 3.0 * std::fabs(v.e_y_real)      // distance from the path
         + 1.0 * v.rolling_real             // recent quadratic cost
         + (v.critical ? 0.5 : 0.0)         // mid-maneuver
         + (v.violated_real ? 1.0 : 0.0);   // currently over the soft bound
}
inline double comfortUrgencyRemote(const VehicleView& v) {
    return 3.0 * std::fabs(v.e_y_est)
         + 1.0 * v.rolling_remote
         + (v.critical_remote ? 0.5 : 0.0)
         + (v.violated_remote ? 1.0 : 0.0);
}

// Classic rate-monotonic priority (shorter period = higher priority).
// Use as the Challenge Q1 (non-context-aware) baseline.
std::unique_ptr<CorePolicy> makeRateMonotonicPolicy();

// Partitioned rate-monotonic: vehicles statically mapped to cores by
// `vehicle % nCores`, one RM-best job per core, no migration.
std::unique_ptr<CorePolicy> makePartitionedRMPolicy();

// Earliest-deadline-first across all vehicles' ready cloud jobs.
std::unique_ptr<CorePolicy> makeEdfPolicy();

// Context-aware: gives cores first to vehicles whose control is degrading
// (high rolling error / in a critical maneuver). Challenge Q2 demo. One class,
// two information sets (see ContextAware.cpp):
//   oracle — scores on ground-truth `*_real` metrics (upper bound only);
//   honest — same scoring on the estimator-derived remote metrics the cloud
//            legitimately sees.
std::unique_ptr<CorePolicy> makeContextAwarePolicy();        // oracle
std::unique_ptr<CorePolicy> makeContextAwareHonestPolicy();  // honest

// Predictive: ranks vehicles by time-to-point-of-no-return (then
// time-to-violation) from the held-command plant rollouts — the physically-
// derived dynamic deadline (PREDICTOR.md). `triage` drops past-PNR cars to
// the bottom instead of boosting them.
std::unique_ptr<CorePolicy> makeTimeToUnsafePolicy(bool triage);

// Two-tier guarded triage: vehicles with TTPNR below `guardMs` get cores
// unconditionally (TimeToUnsafe's rule); remaining capacity is ranked by the
// shared comfort score (ContextAware's rule). See Hybrid.cpp / PREDICTOR.md.
std::unique_ptr<CorePolicy> makeHybridPolicy(double guardMs, bool triage);

// Hybrid with a self-tuning guard: theta(t) = floorMs + live fleet round-trip
// estimate (clamped), plus rescue-clearance tie-breaking in the emergency
// tier. See AdaptiveGuard.cpp / PREDICTOR.md §5c.
std::unique_ptr<CorePolicy> makeAdaptiveGuardPolicy(double floorMs, bool triage);

}  // namespace cps
