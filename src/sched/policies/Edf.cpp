// Edf.cpp — earliest-deadline-first core arbitration across all vehicles'
// ready cloud jobs. Deadline == release + period. Deadline ties are broken by
// the same strict (kind, vehicle) order as RateMonotonic, so the schedule is
// fully deterministic across STL implementations.
#include <algorithm>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class EdfPolicy : public CorePolicy {
public:
    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& /*ctx*/,
                std::vector<int>& chosen) override {
        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const ReadyJob& x = ready[a];
            const ReadyJob& y = ready[b];
            if (x.deadlineStep != y.deadlineStep) return x.deadlineStep < y.deadlineStep;
            if (x.vehicle != y.vehicle)           return x.vehicle < y.vehicle;
            return x.kind < y.kind;
        });

        const int n = std::min<int>(nCores, static_cast<int>(order_.size()));
        chosen.assign(order_.begin(), order_.begin() + n);
    }
    const char* name() const override { return "EDF"; }

private:
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeEdfPolicy() {
    return std::unique_ptr<CorePolicy>(new EdfPolicy());
}

}  // namespace cps
