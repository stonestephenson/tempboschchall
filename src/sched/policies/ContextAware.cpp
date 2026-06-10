// ContextAware.cpp — adaptive, context-aware core arbitration (Challenge Q2).
//
// One decision rule, two information sets:
//   * oracle ("context"): scores on the ground-truth *_real metrics. A real
//     cloud scheduler can never see these (the true state is in the vehicle),
//     so this is an upper bound on what context-awareness could achieve.
//   * honest ("honest"): the identical scoring fed the estimator-derived
//     remote metrics the cloud legitimately has (e_y_est, rolling_remote,
//     critical_remote, violated_remote).
// Both variants live in this one class so the oracle-vs-honest A/B compares
// information sets only — the scoring rule cannot drift apart.
//
// Vehicles with larger lateral error, higher rolling cost, or in a critical
// maneuver get cores first; ties fall back to rate-monotonic.
#include <algorithm>
#include <cmath>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

enum class InfoSet { Oracle, Remote };

class ContextAwarePolicy : public CorePolicy {
public:
    explicit ContextAwarePolicy(InfoSet info) : info_(info) {}

    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& ctx,
                std::vector<int>& chosen) override {
        auto urgency = [&](const ReadyJob& j) -> double {
            if (j.vehicle < 0 || j.vehicle >= static_cast<int>(ctx.size())) return 0.0;
            const VehicleView& v = ctx[j.vehicle];
            const bool oracle = info_ == InfoSet::Oracle;
            const double e_y     = oracle ? v.e_y_real      : v.e_y_est;
            const double rolling = oracle ? v.rolling_real  : v.rolling_remote;
            const bool   crit    = oracle ? v.critical      : v.critical_remote;
            const bool   viol    = oracle ? v.violated_real : v.violated_remote;
            // Weighted "how badly does this vehicle need compute right now".
            return 3.0 * std::fabs(e_y)     // distance from the path
                 + 1.0 * rolling            // recent quadratic cost
                 + (crit ? 0.5 : 0.0)       // mid-maneuver
                 + (viol ? 1.0 : 0.0);      // currently over the soft bound
        };

        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const double ua = urgency(ready[a]);
            const double ub = urgency(ready[b]);
            if (ua != ub) return ua > ub;  // most-suffering vehicle first
            // Tie-break: the same strict (period, vehicle, kind) order as RM.
            if (ready[a].period_ms != ready[b].period_ms)
                return ready[a].period_ms < ready[b].period_ms;
            if (ready[a].vehicle != ready[b].vehicle)
                return ready[a].vehicle < ready[b].vehicle;
            return ready[a].kind < ready[b].kind;
        });

        const int n = std::min<int>(nCores, static_cast<int>(order_.size()));
        chosen.assign(order_.begin(), order_.begin() + n);
    }
    const char* name() const override {
        return info_ == InfoSet::Oracle ? "ContextAware[oracle]" : "ContextAware[honest]";
    }

private:
    InfoSet info_;
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeContextAwarePolicy() {
    return std::unique_ptr<CorePolicy>(new ContextAwarePolicy(InfoSet::Oracle));
}

std::unique_ptr<CorePolicy> makeContextAwareHonestPolicy() {
    return std::unique_ptr<CorePolicy>(new ContextAwarePolicy(InfoSet::Remote));
}

}  // namespace cps
