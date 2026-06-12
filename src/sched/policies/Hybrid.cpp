// Hybrid.cpp — two-tier "guarded triage" arbitration (--scheduler hybrid):
// the safety guard of TimeToUnsafe wrapped around the comfort optimization of
// ContextAware.
//
// Tier 1 (emergency): vehicles whose time-to-point-of-no-return has fallen
//   below the guard threshold get cores unconditionally, soonest TTPNR first
//   (TTV tie-break) — exactly TimeToUnsafe's rule, applied only where physics
//   demands it.
// Tier 2 (comfort): all remaining capacity goes to the rest, ranked by the
//   shared comfort-urgency score (identical to ContextAware's oracle rule by
//   construction — see Policies.h).
//
// At light load tier 1 stays empty and this IS ContextAware; under overload
// the guard engages per vehicle, per moment, and that slice IS TimeToUnsafe.
// The guard (--guard MS, default 150) is the minimum reaction margin promised
// to every car; it must comfortably exceed the chain's command round-trip so
// a rescue command can still arrive before the PNR (see PREDICTOR.md).
//
// `triage` (--triage) matches TimeToUnsafe's semantics: past-PNR vehicles
// drop below BOTH tiers instead of topping tier 1.
#include <algorithm>
#include <string>
#include <vector>

#include "sched/policies/Policies.h"

namespace cps {
namespace {

class HybridPolicy : public CorePolicy {
public:
    HybridPolicy(double guardMs, bool triage)
        : guardMs_(guardMs), triage_(triage) {
        name_ = "Hybrid[guard=" + std::to_string(static_cast<int>(guardMs)) + "ms" +
                (triage ? ",triage]" : "]");
    }

    void assign(const std::vector<ReadyJob>& ready, int nCores,
                const std::vector<VehicleView>& ctx,
                std::vector<int>& chosen) override {
        // Per-job sort key: (tier, k1, k2), lexicographic, then the strict
        // static order. Lower sorts first.
        //   tier 0: emergency  k1 = ttpnr, k2 = ttv
        //   tier 1: comfort    k1 = -urgency, k2 = 0
        //   tier 2: triaged    (past PNR with --triage)
        struct Key { int tier; double k1, k2; };
        auto key = [&](const ReadyJob& j) -> Key {
            if (j.vehicle < 0 || j.vehicle >= static_cast<int>(ctx.size()))
                return {1, 0.0, 0.0};
            const VehicleView& v = ctx[j.vehicle];
            if (triage_ && v.ttpnr_ms <= 0.0) return {2, 0.0, 0.0};
            if (v.ttpnr_ms < guardMs_) return {0, v.ttpnr_ms, v.ttv_ms};
            return {1, -comfortUrgencyOracle(v), 0.0};
        };

        order_.resize(ready.size());
        for (int i = 0; i < static_cast<int>(ready.size()); ++i) order_[i] = i;

        std::sort(order_.begin(), order_.end(), [&](int a, int b) {
            const Key ka = key(ready[a]);
            const Key kb = key(ready[b]);
            if (ka.tier != kb.tier) return ka.tier < kb.tier;
            if (ka.k1 != kb.k1)     return ka.k1 < kb.k1;
            if (ka.k2 != kb.k2)     return ka.k2 < kb.k2;
            // The strict static order shared by all FP policies.
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
    double guardMs_;
    bool triage_;
    std::string name_;
    std::vector<int> order_;
};

}  // namespace

std::unique_ptr<CorePolicy> makeHybridPolicy(double guardMs, bool triage) {
    return std::unique_ptr<CorePolicy>(new HybridPolicy(guardMs, triage));
}

}  // namespace cps
