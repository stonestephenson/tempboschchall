// RateMonotonic.cpp — classic rate-monotonic core arbitration (Challenge Q1
// baseline). Priorities are the strict total order (period, vehicle, kind):
// shorter period first, ties by vehicle id (matching the Challenge's Q1
// exemplar — each additional vehicle at lower priority, so overload degrades
// per vehicle instead of starving a whole stage class), then chain order
// (Controller < Feedforward < Merger) within a vehicle. Each vehicle has at
// most one ready job per kind, so this key is unique — the schedule is a pure
// static per-task fixed-priority order, exactly the model the response-time
// analysis in BOUND.md §7 assumes, and identical on every STL implementation.
#include <algorithm>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class RateMonotonicPolicy : public CorePolicy {
public:
    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& /*ctx*/,
                std::vector<int>& chosen) override {
        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const ReadyJob& x = ready[a];
            const ReadyJob& y = ready[b];
            if (x.period_ms != y.period_ms) return x.period_ms < y.period_ms;
            if (x.vehicle != y.vehicle)     return x.vehicle < y.vehicle;
            return x.kind < y.kind;
        });

        const int n = std::min<int>(nCores, static_cast<int>(order_.size()));
        chosen.assign(order_.begin(), order_.begin() + n);
    }
    const char* name() const override { return "RateMonotonic"; }

private:
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeRateMonotonicPolicy() {
    return std::unique_ptr<CorePolicy>(new RateMonotonicPolicy());
}

}  // namespace cps
