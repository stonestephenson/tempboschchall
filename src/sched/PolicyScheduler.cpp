#include "sched/PolicyScheduler.h"

namespace cps {

PolicyScheduler::PolicyScheduler(std::unique_ptr<CorePolicy> policy)
    : policy_(std::move(policy)) {
    name_ = std::string("PolicyScheduler[") +
            (policy_ ? policy_->name() : "null") + "]";
}

void PolicyScheduler::init(const SimConfig& cfg, const std::vector<TaskSet>& taskSets) {
    nCores_ = cfg.nCores;
    models_.clear();
    models_.reserve(taskSets.size());
    for (int v = 0; v < static_cast<int>(taskSets.size()); ++v)
        models_.emplace_back(v, taskSets[v], cfg.baseStep);
}

void PolicyScheduler::onTick(double /*t*/, long step,
                             const std::vector<VehicleView>& views,
                             std::vector<VehicleTriggers>& out) {
    // 1. Release jobs and collect every vehicle's ready cloud jobs.
    ready_.clear();
    for (auto& m : models_) m.beginTick(step, ready_);

    // 2. Let the policy pick which ready jobs get the shared cores this tick.
    chosen_.clear();
    if (!ready_.empty() && nCores_ > 0)
        policy_->assign(ready_, nCores_, views, chosen_);

    // 3. Grant the chosen cores.
    for (int idx : chosen_) {
        if (idx < 0 || idx >= static_cast<int>(ready_.size())) continue;
        const ReadyJob& j = ready_[idx];
        models_[j.vehicle].grantCore(j.kind);
    }

    // 4. Advance every model and emit this tick's triggers.
    for (size_t v = 0; v < models_.size(); ++v)
        models_[v].endTick(step, out[v]);
}

long PolicyScheduler::missedJobs() const {
    long total = 0;
    for (const auto& m : models_) total += m.missedJobs();
    return total;
}

long PolicyScheduler::maxDataAgeTicks(int vehicle) const {
    if (vehicle < 0 || vehicle >= static_cast<int>(models_.size())) return -1;
    return models_[vehicle].maxDataAgeTicks();
}

}  // namespace cps
