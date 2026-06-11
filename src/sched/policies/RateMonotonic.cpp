// RateMonotonic.cpp — classic rate-monotonic core arbitration (Challenge Q1
// baseline). Shorter period == higher priority; among equal periods, prefer
// jobs already running (less preemption churn), then lower vehicle id.
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
            if (x.started != y.started)     return x.started;  // keep running jobs
            return x.vehicle < y.vehicle;
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
