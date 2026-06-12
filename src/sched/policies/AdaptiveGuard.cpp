// AdaptiveGuard.cpp — Hybrid's guarded triage with a self-tuning guard
// (--scheduler aguard). PREDICTOR.md §5b showed the fixed guard must grow
// with load because its purpose is to cover the command round-trip: a rescue
// decided when TTPNR hits the guard still needs one chain traversal before it
// acts, so achieved floor ≈ guard − round-trip. This policy closes that loop
// online:
//
//   θ(t) = floorTarget + A(t)
//
// where A(t) is the fleet-max *measured recent latch-time age*
// (VehicleView::age_recent_ms — the live round-trip estimate, which inflates
// under contention and thereby also covers queueing implicitly). θ is clamped
// to [floorTarget + 60 ms, 450 ms]: the lower clamp keeps a sane guard before
// any latch is observed; the upper clamp means extreme overload degrades
// gracefully into pure TimeToUnsafe (θ ≥ horizon ⇒ everyone with a finite
// TTPNR is in the emergency tier).
//
// Emergency tier ordering: (TTPNR, rescue clearance, TTV, strict static
// order). Clearance — min(0.8 − |e_y|) over the simulated rescue — breaks
// ties between cars at the same TTPNR grid point (including both at 0): the
// car whose best rescue grazes the wall goes first. This is the legitimate
// form of "use the saved rescue computation when two cars collide at the
// point of no return": the cached rescue informs WHO is served, never WHAT
// the chain computes (the FMU owns all data; see PREDICTOR.md §5c).
#include <algorithm>
#include <string>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class AdaptiveGuardPolicy : public CorePolicy {
public:
    AdaptiveGuardPolicy(double floorMs, bool triage)
        : floorMs_(floorMs), triage_(triage) {
        name_ = "AdaptiveGuard[floor=" + std::to_string(static_cast<int>(floorMs)) +
                "ms" + (triage ? ",triage]" : "]");
    }

    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& ctx,
                std::vector<int>& chosen) override {
        // Adaptive threshold from the live fleet-max round-trip estimate.
        double recentAge = 0.0;
        for (const VehicleView& v : ctx) recentAge = std::max(recentAge, v.age_recent_ms);
        const double theta =
            std::min(450.0, floorMs_ + std::max(60.0, recentAge));

        struct Key { int tier; double k1, k2, k3; };
        auto key = [&](const ReadyJob& j) -> Key {
            if (j.vehicle < 0 || j.vehicle >= static_cast<int>(ctx.size()))
                return {1, 0.0, 0.0, 0.0};
            const VehicleView& v = ctx[j.vehicle];
            if (triage_ && v.ttpnr_ms <= 0.0) return {2, 0.0, 0.0, 0.0};
            if (v.ttpnr_ms < theta)
                return {0, v.ttpnr_ms, v.rescue_clearance_m, v.ttv_ms};
            return {1, -comfortUrgencyOracle(v), 0.0, 0.0};
        };

        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const Key ka = key(ready[a]);
            const Key kb = key(ready[b]);
            if (ka.tier != kb.tier) return ka.tier < kb.tier;
            if (ka.k1 != kb.k1)     return ka.k1 < kb.k1;
            if (ka.k2 != kb.k2)     return ka.k2 < kb.k2;
            if (ka.k3 != kb.k3)     return ka.k3 < kb.k3;
            if (ready[a].period_ms != ready[b].period_ms)
                return ready[a].period_ms < ready[b].period_ms;
            if (ready[a].vehicle != ready[b].vehicle)
                return ready[a].vehicle < ready[b].vehicle;
            return ready[a].kind < ready[b].kind;
        });

        const int n = std::min<int>(nCores, static_cast<int>(order_.size()));
        chosen.assign(order_.begin(), order_.begin() + n);
    }
    const char* name() const override { return name_.c_str(); }

private:
    double floorMs_;
    bool triage_;
    std::string name_;
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeAdaptiveGuardPolicy(double floorMs, bool triage) {
    return std::unique_ptr<CorePolicy>(new AdaptiveGuardPolicy(floorMs, triage));
}

}  // namespace cps
