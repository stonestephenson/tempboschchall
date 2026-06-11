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

struct ScheduleVizConfig {
    std::string path;
    double      windowMs = 120.0;

    bool enabled() const { return !path.empty(); }
};

class PolicyScheduler : public Scheduler {
public:
    explicit PolicyScheduler(std::unique_ptr<CorePolicy> policy, bool partitioned = false,
                             ScheduleVizConfig scheduleViz = {});
    ~PolicyScheduler() override;

    void init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) override;
    void onTick(double t, long step, const std::vector<VehicleView>& views,
                std::vector<VehicleTriggers>& out) override;
    const char* name() const override { return name_.c_str(); }

    long missedJobs() const override;
    long maxDataAgeTicks(int vehicle) const override;

private:
    std::unique_ptr<CorePolicy>  policy_;
    std::vector<TaskChainModel>  models_;
    int                          nCores_ = 3;
    int                          nVehicles_ = 0;
    bool                         partitioned_ = false;
    std::string                  name_;
    double                       baseStepSeconds_ = 0.0;
    ScheduleVizConfig            scheduleViz_;
    long                         scheduleVizTicks_ = 0;
    bool                         scheduleVizWritten_ = false;

    // Per-tick scratch reused to avoid allocations in the hot loop.
    std::vector<ReadyJob> ready_;
    std::vector<int>      chosen_;
    std::vector<int>      coreToReady_;
    std::vector<ReadyJob> partitionReady_;
    std::vector<int>      partitionReadyToGlobal_;
    std::vector<int>      partitionChosen_;

    struct CoreSample {
        int      vehicle = -1;
        TaskKind kind = TaskKind::Estimator;
    };
    std::vector<std::vector<CoreSample>> scheduleSamples_;

    void recordScheduleTick(long step);
    void writeScheduleViz();
};

}  // namespace cps
