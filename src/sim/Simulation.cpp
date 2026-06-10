#include "sim/Simulation.h"

#include <chrono>
#include <cmath>
#include <cstdio>
#include <string>

#include "fmu/FmuVariables.h"

namespace cps {

Simulation::Simulation(const SimParams& params, std::unique_ptr<Scheduler> scheduler,
                       std::shared_ptr<FmuLibrary> lib)
    : params_(params), scheduler_(std::move(scheduler)), lib_(std::move(lib)) {
    dt_ = vr::kBaseStepSeconds;
}

void Simulation::start() {
    if (started_) return;

    traj_ = Trajectory::load(params_.profile);
    durationSteps_ =
        params_.durationSteps > 0 ? params_.durationSteps : traj_->lapSteps();
    if (!lib_) lib_ = std::make_shared<FmuLibrary>();

    const int n = params_.nVehicles;
    offsets_ = params_.startOffsets;
    if (static_cast<int>(offsets_.size()) != n) {
        offsets_.resize(n);
        for (int v = 0; v < n; ++v)  // spread evenly around the lap
            offsets_[v] = static_cast<long>(static_cast<double>(v) * traj_->lapSteps() / n);
    }

    vehicles_.resize(n);
    std::vector<TaskSet> taskSets(n);
    for (int v = 0; v < n; ++v) {
        Vehicle& veh = vehicles_[v];
        veh.traj = traj_;
        veh.startOffset = offsets_[v];
        veh.taskSet = TaskSet::challengeDefault();
        veh.taskSet.execMode = params_.execMode;
        veh.taskSet.overrun  = params_.overrun;
        veh.taskSet.seed = params_.seed;
        if (params_.netDelayMs >= 0.0) {
            const ExecTimes fixed{params_.netDelayMs, params_.netDelayMs, params_.netDelayMs};
            veh.taskSet.netSC.delay = fixed;
            veh.taskSet.netCA.delay = fixed;
        }
        taskSets[v] = veh.taskSet;

        veh.fmu = lib_->instantiate("veh" + std::to_string(v));
        const double v0 = traj_->inputsAt(offsets_[v]).vel;
        veh.fmu->initialize(0.0, static_cast<double>(durationSteps_) * dt_, v0);
    }

    SimConfig cfg{n, params_.nCores, dt_};
    scheduler_->init(cfg, taskSets);

    rec_.profile       = static_cast<int>(params_.profile);
    rec_.nVehicles     = n;
    rec_.nCores        = params_.nCores;
    rec_.baseStep      = dt_;
    rec_.durationSteps = durationSteps_;
    rec_.decimation    = params_.decimation;
    rec_.schedulerName = scheduler_->name();
    rec_.startOffsets  = offsets_;
    rec_.frames.assign(n, {});
    rec_.summary.assign(n, {});

    views_.resize(n);
    triggers_.resize(n);
    maxRolling_.assign(n, 0.0);
    hardCount_.assign(n, 0);

    step_ = 0;
    finalized_ = false;
    started_ = true;
}

void Simulation::buildViews() {
    for (size_t v = 0; v < vehicles_.size(); ++v) {
        const VehicleOutputs& o = vehicles_[v].out;
        views_[v] = VehicleView{static_cast<int>(v), vehicles_[v].curVel,
                                o.e_y_real, o.e_y_est, o.rolling_real, o.rolling_remote,
                                o.average_real, o.threshold_cntr_real,
                                o.critical_real, o.violated_real,
                                o.critical_remote, o.violated_remote};
    }
}

bool Simulation::step() {
    if (!started_) start();
    if (step_ >= durationSteps_) {
        finalizeSummary();
        return false;
    }
    const double t = static_cast<double>(step_) * dt_;

    // 1. Apply this tick's reference inputs and snapshot the latest state.
    for (size_t v = 0; v < vehicles_.size(); ++v) {
        const Trajectory::Inputs in = vehicles_[v].traj->inputsAt(step_ + offsets_[v]);
        vehicles_[v].curVel = in.vel;
        vehicles_[v].fmu->setInputs(in.ff0, in.ff1, in.vel);
    }
    buildViews();

    // 2. Scheduler decides the triggers for every vehicle.
    scheduler_->onTick(t, step_, views_, triggers_);

    // 3. Apply triggers, advance the FMUs, read outputs.
    for (size_t v = 0; v < vehicles_.size(); ++v) {
        vehicles_[v].fmu->applyTriggers(triggers_[v]);
        vehicles_[v].fmu->doStep(t, dt_);
        vehicles_[v].out = vehicles_[v].fmu->readOutputs();
    }

    // 4. Record a decimated frame.
    if (step_ % params_.decimation == 0) recordFrame(t);

    ++step_;
    if (step_ >= durationSteps_) finalizeSummary();
    return true;
}

void Simulation::recordFrame(double t) {
    for (size_t v = 0; v < vehicles_.size(); ++v) {
        const VehicleOutputs& o = vehicles_[v].out;
        Frame f;
        f.t            = static_cast<float>(t);
        f.refStep      = static_cast<uint32_t>(vehicles_[v].traj->wrap(step_ + offsets_[v]));
        f.e_y_real     = static_cast<float>(o.e_y_real);
        f.e_y_est      = static_cast<float>(o.e_y_est);
        f.act          = static_cast<float>(o.act_out);
        f.vel          = static_cast<float>(vehicles_[v].curVel);
        f.rolling_real = static_cast<float>(o.rolling_real);
        f.average_real = static_cast<float>(o.average_real);

        const double absEy = std::fabs(o.e_y_real);
        uint8_t flags = 0;
        if (o.violated_real || absEy > vr::kSoftBound) flags |= Frame::kSoft;
        if (absEy > vr::kHardBound) { flags |= Frame::kHard; ++hardCount_[v]; }
        if (o.critical_real) flags |= Frame::kCritical;
        f.flags = flags;

        rec_.frames[v].push_back(f);
        if (o.rolling_real > maxRolling_[v]) maxRolling_[v] = o.rolling_real;
    }
}

void Simulation::finalizeSummary() {
    if (finalized_) return;
    const double duration = static_cast<double>(durationSteps_) * dt_;
    for (size_t v = 0; v < vehicles_.size(); ++v) {
        VehicleSummary& s = rec_.summary[v];
        s.average_real        = vehicles_[v].out.average_real;
        s.max_rolling_real    = maxRolling_[v];
        s.threshold_cntr_real = vehicles_[v].out.threshold_cntr_real;
        s.soft_violation_pct  = duration > 0.0
            ? 100.0 * (s.threshold_cntr_real * vr::kMetricPeriodSeconds) / duration
            : 0.0;
        s.hard_violations     = hardCount_[v];
        const long ageTicks   = scheduler_->maxDataAgeTicks(static_cast<int>(v));
        s.max_data_age_ms     = ageTicks < 0 ? -1.0 : ageTicks * dt_ * 1000.0;
        const long ageOldTicks = scheduler_->maxDataAgeOldestTicks(static_cast<int>(v));
        s.max_data_age_oldest_ms = ageOldTicks < 0 ? -1.0 : ageOldTicks * dt_ * 1000.0;
    }
    rec_.missedJobs = scheduler_->missedJobs();
    finalized_ = true;
}

void Simulation::runToCompletion(bool verbose) {
    if (!started_) start();
    const auto t0 = std::chrono::steady_clock::now();
    const long report = durationSteps_ > 20 ? durationSteps_ / 20 : 1;
    while (step()) {
        if (verbose && step_ % report == 0)
            std::printf("\r  simulating... %3.0f%%", 100.0 * progress());
    }
    if (verbose) {
        const auto t1 = std::chrono::steady_clock::now();
        const double secs = std::chrono::duration<double>(t1 - t0).count();
        std::printf("\r  simulated %.1f s of driving in %.2f s wall (%.0fx)\n",
                    rec_.duration(), secs, secs > 0 ? rec_.duration() / secs : 0.0);
        std::printf("  scheduler: %s   missed jobs: %ld\n",
                    rec_.schedulerName.c_str(), rec_.missedJobs);
        std::printf("  %-4s %12s %12s %10s %10s %13s %13s\n", "veh", "avg_perf", "max_roll",
                    "soft%", "hard", "age_fresh(ms)", "age_path(ms)");
        double worstAgeMs = -1.0, worstAgeOldMs = -1.0;
        auto fmtAge = [](char* buf, size_t n, double v) {
            if (v < 0.0) std::snprintf(buf, n, "%13s", "n/a");
            else         std::snprintf(buf, n, "%13.2f", v);
        };
        for (int v = 0; v < rec_.nVehicles; ++v) {
            const VehicleSummary& s = rec_.summary[v];
            if (s.max_data_age_ms > worstAgeMs) worstAgeMs = s.max_data_age_ms;
            if (s.max_data_age_oldest_ms > worstAgeOldMs) worstAgeOldMs = s.max_data_age_oldest_ms;
            char ageBuf[24], ageOldBuf[24];
            fmtAge(ageBuf, sizeof ageBuf, s.max_data_age_ms);
            fmtAge(ageOldBuf, sizeof ageOldBuf, s.max_data_age_oldest_ms);
            std::printf("  %-4d %12.5f %12.5f %9.2f%% %10d %s %s\n", v, s.average_real,
                        s.max_rolling_real, s.soft_violation_pct, s.hard_violations,
                        ageBuf, ageOldBuf);
        }
        if (worstAgeMs >= 0.0)
            std::printf("  worst-case data age: %.2f ms (freshest) / %.2f ms (path)\n",
                        worstAgeMs, worstAgeOldMs);
    }
}

}  // namespace cps
