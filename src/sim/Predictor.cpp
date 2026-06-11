#include "sim/Predictor.h"

#include <algorithm>
#include <cmath>

namespace cps {
namespace {

constexpr double kHardBound = 0.8;   // |e_y| hard safety bound, m
constexpr double kSoftBound = 0.2;   // comfort bound, used for recovery early-exit
constexpr double kVelTol    = 1e-6;  // FMU's velocity-change tolerance (TOL)

// ---------------------------------------------------------------------------
// Plant model, ported VERBATIM from the FMU source so rollouts are tick-exact
// replicas of the simulated physics (checked by --validate-predictor):
//   * calculate_matrices_from_velocity
//     LateralMotionControl/sources/LateralMotionControl.c:793-880
//     (including the v < 0.1 clamp)
//   * state update of simulate_lateral_motion_control_step (:576-600):
//     x+ = Ad(v) x + Bd(v) u + Fd(v) r, dt = 0.1 ms, noise off.
// Do not "simplify" the coefficient expressions — bit-identical arithmetic is
// what makes the fidelity gate pass at ~1e-12.
// ---------------------------------------------------------------------------
struct Matrices {
    double Ad[6][6];
    double Bd[6];
    double Fd[6][2];
};

void computeMatrices(double v, Matrices& mm) {
    if (v < 0.1) v = 0.1;  // FMU uses a safe minimum velocity

    const double v_2 = v * v;
    const double v_3 = v_2 * v;
    const double v_4 = v_3 * v;

    double (&Ad_out)[6][6] = mm.Ad;
    double (&Bd_out)[6] = mm.Bd;
    double (&Fd_out)[6][2] = mm.Fd;

    Ad_out[0][0] =  (1.0e-39*(9.9999992441078955351940749096684e+38*v_3 - 8.6768465022281584351278202058998e+36*v_2 + 4.2124606929881317331568202355641e+34*v - 164182153467125542443784064573640.0))/v_3;
    Ad_out[0][1] = (5.0e-39*(3.0235683429071821652633911270414e+35*v_2 - 4.2861764753450482293931772526108e+33*v + 32355623984465959638276476642571.0))/v_2;
    Ad_out[0][2] = -(1.0e-40*(- 2.5432883195577070042925349468987e+37*v_2 + 5.3015196214781872653199271128166e+34*v + 184623856390639293241490289862460.0))/v_2;
    Ad_out[0][3] = 0.00000012703796317370639392241424194679 - 0.00000000017671732071593959522513550294105/v;
    Ad_out[0][4] = 0.0;
    Ad_out[0][5] = 0.0;

    Ad_out[1][0] = (1.0e-39*(- 9.9999997480359650758140411931185e+34*v_4 + 1.4175887167900519739149878262197e+33*v_3 + 5.9277746027191436000576660869639e+36*v_2 - 8.4031617740317372385689576648245e+34*v + 634340522853345862857055293912660.0))/v_4;
    Ad_out[1][1] = (5.0e-38*(1.9999998488215791070388149819337e+37*v_3 - 3.9349851380969314496870614105301e+35*v_2 + 3.960642809890990213329831082234e+33*v - 26822037073326284723020598414963.0))/v_3;
    Ad_out[1][2] = -(2.0e-39*(63582235738255002825961481518341.0*v_3 - 3.7917752618272791612083441804878e+36*v_2 + 3.3532442397341284527102767132334e+34*v - 214678405839896777788328425575140.0))/v_3;
    Ad_out[1][3] = -(1.0e-44*(423881571588366777573335346883740.0*v_2 - 3.788004105039474223633882976614e+37*v + 2.2354961598227519865958923852818e+35))/v_2;
    Ad_out[1][4] = 0.0;
    Ad_out[1][5] = 0.0;

    Ad_out[2][0] = 0.0;
    Ad_out[2][1] = 0.0;
    Ad_out[2][2] =  0.99999876752102034860314461184316;
    Ad_out[2][3] = 0.00009985088133244983822200263601232;
    Ad_out[2][4] = 0.0;
    Ad_out[2][5] = 0.0;

    Ad_out[3][0] = 0.0;
    Ad_out[3][1] = 0.0;
    Ad_out[3][2] = -0.024637332671482679857799524825168;
    Ad_out[3][3] = 0.99701869799747722122873483385774;
    Ad_out[3][4] = 0.0;
    Ad_out[3][5] = 0.0;

    Ad_out[4][0] = 0.000000000014461412392027089250753046900418 - 0.0000000050000000000000001046128041506424*v;
    Ad_out[4][1] = -0.0000000000025196403492418600449568564397402*v;
    Ad_out[4][2] = -0.0000000000042388157158836677757333534688374*v;
    Ad_out[4][3] = 0.0;
    Ad_out[4][4] = 1.0;
    Ad_out[4][5] = 0.0001;

    Ad_out[5][0] = -(2.0e-41*(4.9999998740179825379070205965592e+36*v_2 - 2.1692118588040635977685314118163e+34*v + 70207678216468862219280337259401.0))/v;
    Ad_out[5][1] = 0.00000000071436274589084149591834829590696 - 0.000000075589210477255787642064449247115*v;
    Ad_out[5][2] = 0.00000000017671732071593959522513550294105 - 0.00000012716447147651000565192296303668*v;
    Ad_out[5][3] = -0.0000000000042388157158836677757333534688374*v;
    Ad_out[5][4] = 0.0;
    Ad_out[5][5] = 1.0;

    Bd_out[0] = 0.000000001045110377136694002085510438356 - 0.000000000001090086377103607923645469856602/v;
    Bd_out[1] = -(2.0e-46*(130736343450025143032735776971930.0*v_2 - 1.5581493515446014309924985743344e+37*v + 6.894864351772649721929665510057e+34))/v_2;
    Bd_out[2] = 0.0000012324796416882199907643843928007;
    Bd_out[3] = 0.0246373326592935516787807870287;
    Bd_out[4] = 0.0;
    Bd_out[5] = -0.000000000000026147268690005028606547155394386*v;

    Fd_out[0][0] = 0.0;
    Fd_out[0][1] = 0.0;
    Fd_out[1][0] = 0.0;
    Fd_out[1][1] = 0.0;
    Fd_out[2][0] = 0.0;
    Fd_out[2][1] = 0.0;
    Fd_out[3][0] = 0.0;
    Fd_out[3][1] = 0.0;
    Fd_out[4][0] = 0.000000005*v_2;
    Fd_out[4][1] = 0.000000001*v_2;
    Fd_out[5][0] = 0.0001*v_2;
    Fd_out[5][1] = 0.00002*v_2;
}

// Velocity-cached plant. refresh() mirrors the FMU's fmi2SetReal velocity
// guard (update only when the change exceeds TOL) + per-doStep matrix update.
struct Model {
    Matrices m{};
    double cachedV = -1e9;

    void refresh(double v) {
        if (std::fabs(v - cachedV) > kVelTol) {
            computeMatrices(v, m);
            cachedV = v;
        }
    }

    // x+ = Ad x + Bd u + Fd r  (mirrors simulate_lateral_motion_control_step)
    void step(double x[6], double u, double r0, double r1) const {
        double nx[6];
        for (int i = 0; i < 6; ++i) {
            double s = 0.0;
            for (int j = 0; j < 6; ++j) s += m.Ad[i][j] * x[j];
            nx[i] = s + m.Bd[i] * u + m.Fd[i][0] * r0 + m.Fd[i][1] * r1;
        }
        for (int i = 0; i < 6; ++i) x[i] = nx[i];
    }
};

// Which steering sign reduces a positive lateral error. Determined once by
// probing the model (apply +0.1 rad from rest at v = 10 for 0.5 s and observe
// the e_y response) so the recovery law never depends on a hand-assumed sign
// convention.
double steerSignForReducingError() {
    Model probe;
    probe.refresh(10.0);
    double x[6] = {0, 0, 0, 0, 0, 0};
    for (int k = 0; k < 5000; ++k) probe.step(x, 0.1, 0.0, 0.0);
    // If +steer pushed e_y negative, +steer reduces a positive error.
    return x[4] < 0.0 ? 1.0 : -1.0;
}

// PD-flavored bang-bang recovery command: full assumed steering authority,
// direction chosen to oppose (e_y + tau * e_y_dot). The model's second-order
// steering dynamics rate-shape it; tau damps oscillation around the path.
double recoveryCommand(const double x[6], double deltaMax, double steerSign) {
    constexpr double kTau = 0.3;  // s of lookahead on the error
    const double pd = x[4] + kTau * x[5];
    return steerSign * (pd > 0.0 ? 1.0 : -1.0) * deltaMax;
}

// Can the car still avoid |e_y| >= 0.8 if recovery steering starts from state
// xStart at absolute trajectory step `stepAt`? Heuristic: latency ticks of
// continued hold, then bang-bang recovery for up to recoveryWindowTicks.
bool recoverable(const double xStart[6], double heldCmd, long stepAt,
                 const Trajectory& traj, long trajOffset,
                 const PredictParams& p, double deltaMax, double steerSign) {
    Model model;
    double x[6];
    for (int i = 0; i < 6; ++i) x[i] = xStart[i];

    long s = stepAt;
    for (long k = 0; k < p.recoveryLatencyTicks; ++k, ++s) {
        const Trajectory::Inputs in = traj.inputsAt(s + trajOffset);
        model.refresh(in.vel);
        model.step(x, heldCmd, in.ff0, in.ff1);
        if (std::fabs(x[4]) >= kHardBound) return false;
    }
    for (long k = 0; k < p.recoveryWindowTicks; ++k, ++s) {
        const Trajectory::Inputs in = traj.inputsAt(s + trajOffset);
        model.refresh(in.vel);
        model.step(x, recoveryCommand(x, deltaMax, steerSign), in.ff0, in.ff1);
        if (std::fabs(x[4]) >= kHardBound) return false;
        // Settled well inside the comfort band -> call it recovered.
        if (k > 1000 && std::fabs(x[4]) < kSoftBound && std::fabs(x[5]) < 0.05)
            return true;
    }
    return true;  // survived the whole window without breaching
}

}  // namespace

double defaultDeltaMax(Profile p) {
    // Calibrated as max |act_out| over a clean N=1 --exec worst lap, x1.5
    // margin (measured 2026-06-11 via --validate-predictor: 0.1903 / 0.3561 /
    // 0.2793 rad; see PREDICTOR.md for rationale).
    switch (p) {
        case Profile::V10:   return 0.285;
        case Profile::V12_5: return 0.534;
        case Profile::V15:   return 0.419;
    }
    return 0.285;
}

Prediction predictHold(const double x0[6], double heldCmd, long fromStep,
                       const Trajectory& traj, long trajOffset,
                       const PredictParams& params) {
    static const double kSteerSign = steerSignForReducingError();
    const double deltaMax = params.deltaMax > 0.0
                                ? params.deltaMax
                                : defaultDeltaMax(traj.profile());

    Prediction out;
    out.fromStep = fromStep;
    out.ttvTicks = params.horizonTicks;

    Model model;
    double x[6];
    for (int i = 0; i < 6; ++i) x[i] = x0[i];

    // --- Held-command rollout: e_y polyline, TTV, and snapshots for the PNR
    //     search (so recovery rollouts never re-simulate the hold prefix). ---
    out.e_y.reserve(static_cast<size_t>(params.horizonTicks / params.vizStride) + 1);
    out.e_y.push_back(static_cast<float>(x[4]));

    const long stride = std::max<long>(1, params.pnrSearchStrideTicks);
    std::vector<double> snaps;  // 6 doubles per snapshot, at ticks 0, stride, 2*stride, ...
    snaps.reserve(static_cast<size_t>(params.horizonTicks / stride + 2) * 6);
    for (int i = 0; i < 6; ++i) snaps.push_back(x[i]);

    for (long k = 1; k <= params.horizonTicks; ++k) {
        const Trajectory::Inputs in = traj.inputsAt(fromStep + (k - 1) + trajOffset);
        model.refresh(in.vel);
        model.step(x, heldCmd, in.ff0, in.ff1);

        if (k % params.vizStride == 0) out.e_y.push_back(static_cast<float>(x[4]));
        if (k % stride == 0)
            for (int i = 0; i < 6; ++i) snaps.push_back(x[i]);
        if (out.ttvTicks == params.horizonTicks && std::fabs(x[4]) >= kHardBound)
            out.ttvTicks = k;  // keep rolling: the full polyline is still useful
    }

    if (!params.computePnr) {
        out.ttpnrTicks = params.horizonTicks;
        return out;
    }

    // --- TTPNR: latest snapshot from which recovery still succeeds. Assumes
    //     recoverability is monotone in the hold time (heuristic; see
    //     PREDICTOR.md), so binary search over the snapshot grid applies. ---
    auto recoverableAt = [&](long snapIdx) {
        return recoverable(&snaps[static_cast<size_t>(snapIdx) * 6], heldCmd,
                           fromStep + snapIdx * stride, traj, trajOffset,
                           params, deltaMax, kSteerSign);
    };

    const long lastSnap = static_cast<long>(snaps.size() / 6) - 1;
    // Only hold times strictly before the violation can be recovery starts.
    const long capTicks = std::min(out.ttvTicks, params.horizonTicks);
    long hi = std::min(lastSnap, capTicks / stride);
    while (hi > 0 && hi * stride >= capTicks) --hi;

    if (!recoverableAt(0)) {
        out.ttpnrTicks = 0;
        out.pastPnr = true;
        return out;
    }
    if (recoverableAt(hi)) {
        // Recoverable right up to the cap: if no violation was even found in
        // the horizon, report "ttpnr >= horizon"; else the last grid point.
        out.ttpnrTicks = (out.ttvTicks >= params.horizonTicks) ? params.horizonTicks
                                                               : hi * stride;
        return out;
    }
    long lo = 0;  // invariant: recoverableAt(lo) && !recoverableAt(hi)
    while (hi - lo > 1) {
        const long mid = lo + (hi - lo) / 2;
        if (recoverableAt(mid)) lo = mid; else hi = mid;
    }
    out.ttpnrTicks = lo * stride;
    return out;
}

}  // namespace cps
