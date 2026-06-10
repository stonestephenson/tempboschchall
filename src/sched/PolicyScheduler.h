// PolicyScheduler.h — the declarative-layer Scheduler. It owns one
// TaskChainModel per vehicle and delegates the shared-core arbitration to a
// CorePolicy. Swapping scheduling methods is just swapping the CorePolicy.
#pragma once

#include <memory>
#include <string>
#include <vector>

#include "sched/CorePolicy.h"
#include "sched/Scheduler.h"
#include "sched/TaskModel.h"

namespace cps {

class PolicyScheduler : public Scheduler {
public:
    explicit PolicyScheduler(std::unique_ptr<CorePolicy> policy);

    void init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) override;
    void onTick(double t, long step, const std::vector<VehicleView>& views,
                std::vector<VehicleTriggers>& out) override;
    const char* name() const override { return name_.c_str(); }

    long missedJobs() const override;
    long maxDataAgeTicks(int vehicle) const override;
    long maxDataAgeOldestTicks(int vehicle) const override;

private:
    std::unique_ptr<CorePolicy>  policy_;
    std::vector<TaskChainModel>  models_;
    int                          nCores_ = 3;
    std::string                  name_;

    // Per-tick scratch reused to avoid allocations in the hot loop.
    std::vector<ReadyJob> ready_;
    std::vector<int>      chosen_;
};

}  // namespace cps
