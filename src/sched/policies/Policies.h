// Policies.h — factories for the built-in CorePolicy implementations.
//
// Each policy lives in its own translation unit and exposes a factory here, so
// main.cpp can select one without seeing the class definition. To add your own
// scheduling method: create sched/policies/MyPolicy.cpp defining a CorePolicy
// subclass + a makeMyPolicy() factory, declare it here (or in main.cpp), and
// pass it to PolicyScheduler.
#pragma once

#include <memory>

#include "sched/CorePolicy.h"

namespace cps {

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

}  // namespace cps
