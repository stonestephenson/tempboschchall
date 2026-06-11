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

// Earliest-deadline-first across all vehicles' ready cloud jobs.
std::unique_ptr<CorePolicy> makeEdfPolicy();

// Context-aware: gives cores first to vehicles whose control is degrading
// (high rolling error / in a critical maneuver). Challenge Q2 demo.
std::unique_ptr<CorePolicy> makeContextAwarePolicy();

}  // namespace cps
