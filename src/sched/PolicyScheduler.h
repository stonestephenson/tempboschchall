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
    double      maxMs = 120.0;

    bool enabled() const { return !path.empty() && maxMs > 0.0; }
};

class PolicyScheduler : public Scheduler {
public:
    explicit PolicyScheduler(std::unique_ptr<CorePolicy> policy,
                             bool partitioned = false,
                             ScheduleVizConfig viz = {});
    ~PolicyScheduler() override;

    void init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) override;
    void onTick(double t, long step, const std::vector<VehicleView>& views,
                std::vector<VehicleTriggers>& out) override;
    const char* name() const override { return name_.c_str(); }

    long missedJobs() const override;
    long maxDataAgeTicks(int vehicle) const override;
    long maxDataAgeOldestTicks(int vehicle) const override;

private:
    struct VizSpan {
        int      core = 0;
        int      vehicle = 0;
        TaskKind kind = TaskKind::Estimator;
        long     startStep = 0;
        long     endStep = 0;
    };

    void recordVizGrant(long step, const ReadyJob& job, int core);
    void writeScheduleViz() const;

    std::unique_ptr<CorePolicy>  policy_;
    std::vector<TaskChainModel>  models_;
    int                          nCores_ = 3;
    double                       baseStepSeconds_ = 1e-4;
    bool                         partitioned_ = false;
    std::string                  name_;
    ScheduleVizConfig            viz_;

    // Per-tick scratch reused to avoid allocations in the hot loop.
    std::vector<ReadyJob> ready_;
    std::vector<int>      chosen_;
    std::vector<ReadyJob> partitionReady_;
    std::vector<int>      partitionMap_;
    std::vector<int>      partitionChosen_;

    std::vector<VizSpan>  vizSpans_;
    std::vector<int>      vizLastSpanByLane_;
    mutable bool          vizWritten_ = false;
};

}  // namespace cps
