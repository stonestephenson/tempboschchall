// TimeToUnsafe.cpp — predictive core arbitration on physical deadlines
// (--scheduler ttu). The Route B scheduler.
//
// Ranks vehicles by how soon scheduling stops mattering: primary key is
// time-to-point-of-no-return (TTPNR — after this instant, even a fresh
// command under the assumed steering limit cannot keep |e_y| < 0.8),
// tie-broken by time-to-violation (TTV — when |e_y| crosses 0.8 under the
// held command), then by the strict (period, vehicle, kind) order. Both
// predictions come from the harness-side plant rollouts (PREDICTOR.md) via
// VehicleView; values at the horizon mean "relaxed".
//
// Past-PNR cars (ttpnr_ms == 0): by default maximally urgent — the real
// plant's steering is unbounded, so the controller may still recover them,
// and deprioritizing would manufacture crashes. With triage=true they drop
// to the bottom instead (rescue-vs-triage experiment; --triage flag).
#include <algorithm>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class TimeToUnsafePolicy : public CorePolicy {
public:
    explicit TimeToUnsafePolicy(bool triage) : triage_(triage) {}

    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& ctx,
                std::vector<int>& chosen) override {
        auto key = [&](const ReadyJob& j) -> double {
            if (j.vehicle < 0 || j.vehicle >= static_cast<int>(ctx.size()))
                return 1e18;
            const VehicleView& v = ctx[j.vehicle];
            if (triage_ && v.ttpnr_ms <= 0.0) return 1e18;  // unsavable: stand aside
            return v.ttpnr_ms;  // smaller = closer to the point of no return
        };

        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const double ka = key(ready[a]);
            const double kb = key(ready[b]);
            if (ka != kb) return ka < kb;            // soonest deadline first
            const double ta = ctx[ready[a].vehicle].ttv_ms;
            const double tb = ctx[ready[b].vehicle].ttv_ms;
            if (ta != tb) return ta < tb;            // then soonest violation
            // Then the strict static order shared by all FP policies.
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
        return triage_ ? "TimeToUnsafe[triage]" : "TimeToUnsafe";
    }

private:
    bool triage_;
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeTimeToUnsafePolicy(bool triage) {
    return std::unique_ptr<CorePolicy>(new TimeToUnsafePolicy(triage));
}

}  // namespace cps
