// PartitionedRM.cpp — partitioned rate-monotonic core arbitration, for the
// global-vs-partitioned comparison.
//
// Vehicles are statically partitioned onto cores by `vehicle % nCores`; each
// tick, every core runs the single best RM job among the ready jobs of its own
// vehicles (no migration, no work stealing — a core idles if its partition has
// no ready job, even while another partition is overloaded). Same per-job RM
// ordering as RateMonotonic.cpp.
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class PartitionedRMPolicy : public CorePolicy {
public:
    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& /*ctx*/,
                std::vector<int>& chosen) override {
        // bestPerCore_[c] = index into `ready` of the current best job for core c.
        bestPerCore_.assign(static_cast<size_t>(nCores), -1);

        auto better = [&](const ReadyJob& x, const ReadyJob& y) {
            if (x.period_ms != y.period_ms) return x.period_ms < y.period_ms;
            if (x.vehicle != y.vehicle)     return x.vehicle < y.vehicle;
            return x.kind < y.kind;
        };

        for (int i = 0; i < static_cast<int>(ready.size()); ++i) {
            const int core = ready[i].vehicle % nCores;
            const int cur = bestPerCore_[core];
            if (cur < 0 || better(ready[i], ready[cur])) bestPerCore_[core] = i;
        }

        for (int idx : bestPerCore_)
            if (idx >= 0) chosen.push_back(idx);
    }
    const char* name() const override { return "PartitionedRM"; }

private:
    std::vector<int> bestPerCore_;
};

}  // namespace

std::unique_ptr<CorePolicy> makePartitionedRMPolicy() {
    return std::unique_ptr<CorePolicy>(new PartitionedRMPolicy());
}

}  // namespace cps
