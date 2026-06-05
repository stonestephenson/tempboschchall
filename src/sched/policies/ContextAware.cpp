// ContextAware.cpp — adaptive, context-aware core arbitration (Challenge Q2).
//
// Instead of a fixed priority, this ranks ready jobs by how much their vehicle's
// control is currently suffering: vehicles with larger lateral error, higher
// rolling cost, or in a critical maneuver get cores first. Ties fall back to
// rate-monotonic. This demonstrates using runtime control metrics (VehicleView)
// to drive scheduling — swap it for RateMonotonic to see the difference.
#include <algorithm>
#include <cmath>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class ContextAwarePolicy : public CorePolicy {
public:
    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& ctx,
                std::vector<int>& chosen) override {
        auto urgency = [&](const ReadyJob& j) -> double {
            if (j.vehicle < 0 || j.vehicle >= static_cast<int>(ctx.size())) return 0.0;
            const VehicleView& v = ctx[j.vehicle];
            // Weighted "how badly does this vehicle need compute right now".
            return 3.0 * std::fabs(v.e_y_real)     // distance from the path
                 + 1.0 * v.rolling_real            // recent quadratic cost
                 + (v.critical ? 0.5 : 0.0)         // mid-maneuver
                 + (v.violated_real ? 1.0 : 0.0);   // currently over the soft bound
        };

        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const double ua = urgency(ready[a]);
            const double ub = urgency(ready[b]);
            if (ua != ub) return ua > ub;  // most-suffering vehicle first
            // Tie-break: rate-monotonic, then running jobs, then vehicle id.
            if (ready[a].period_ms != ready[b].period_ms)
                return ready[a].period_ms < ready[b].period_ms;
            if (ready[a].started != ready[b].started) return ready[a].started;
            return ready[a].vehicle < ready[b].vehicle;
        });

        const int n = std::min<int>(nCores, static_cast<int>(order_.size()));
        chosen.assign(order_.begin(), order_.begin() + n);
    }
    const char* name() const override { return "ContextAware"; }

private:
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeContextAwarePolicy() {
    return std::unique_ptr<CorePolicy>(new ContextAwarePolicy());
}

}  // namespace cps
