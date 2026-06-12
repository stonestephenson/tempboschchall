// CorePolicy.h — the pluggable arbitration policy. This is the small piece a
// user typically writes to define a new scheduling method: given the set of
// cloud jobs that want to run this tick (across all vehicles) and the shared
// core count, pick which ones get a core. May consult the per-vehicle
// VehicleView for context-aware decisions (Challenge Q2).
//
// To add a scheduling method: subclass CorePolicy in one .cpp, then in main.cpp
//   sim.setScheduler(std::make_unique<PolicyScheduler>(std::make_unique<MyPolicy>()));
#pragma once

#include <vector>

#include "sched/Scheduler.h"   // VehicleView
#include "sched/TaskModel.h"   // ReadyJob, TaskKind

namespace cps {

class CorePolicy {
public:
    virtual ~CorePolicy() = default;

    // Choose at most nCores jobs to run this tick. Fill `chosen` with indices
    // into `ready`. ctx[j.vehicle] gives that vehicle's latest metrics.
    virtual void assign(const std::vector<ReadyJob>& ready, int nCores,
                        const std::vector<VehicleView>& ctx,
                        std::vector<int>& chosen) = 0;

    virtual const char* name() const { return "CorePolicy"; }
};

}  // namespace cps
