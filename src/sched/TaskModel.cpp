#include "sched/TaskModel.h"

#include <algorithm>
#include <cmath>
#include <limits>

namespace cps {

const char* taskKindName(TaskKind k) {
    switch (k) {
        case TaskKind::Sensor:      return "Sensor";
        case TaskKind::Estimator:   return "Estimator";
        case TaskKind::Controller:  return "Controller";
        case TaskKind::Feedforward: return "Feedforward";
        case TaskKind::Merger:      return "Merger";
        case TaskKind::Actuator:    return "Actuator";
    }
    return "?";
}

bool isCloudTask(TaskKind k) {
    return k == TaskKind::Estimator || k == TaskKind::Controller ||
           k == TaskKind::Feedforward || k == TaskKind::Merger;
}

const TaskParams& TaskSet::task(TaskKind k) const {
    switch (k) {
        case TaskKind::Sensor:      return sensor;
        case TaskKind::Estimator:   return estimator;
        case TaskKind::Controller:  return controller;
        case TaskKind::Feedforward: return feedforward;
        case TaskKind::Merger:      return merger;
        case TaskKind::Actuator:    return actuator;
    }
    return sensor;
}

TaskSet TaskSet::challengeDefault() {
    // Values from examples/parameters.md (period / BCET / AvET / WCET, ms).
    TaskSet ts;
    ts.sensor      = {TaskKind::Sensor,       5.0, 0.0, {0.4, 0.6, 1.0}};
    ts.estimator   = {TaskKind::Estimator,   10.0, 0.0, {0.7, 0.9, 1.1}};
    ts.controller  = {TaskKind::Controller,  20.0, 0.0, {0.2, 0.3, 0.5}};
    ts.feedforward = {TaskKind::Feedforward, 20.0, 0.0, {0.2, 1.2, 2.5}};
    ts.merger      = {TaskKind::Merger,      20.0, 0.0, {0.2, 0.3, 0.5}};
    ts.actuator    = {TaskKind::Actuator,    30.0, 0.0, {0.2, 0.5, 0.7}};
    ts.netSC = {10.0, {1.0, 8.0, 16.0}};  // sensor -> cloud
    ts.netCA = {10.0, {1.0, 8.0, 16.0}};  // cloud  -> actuator
    ts.execMode = ExecMode::Average;
    ts.seed = 0;
    return ts;
}

// ----------------------------------------------------------------------------
// TaskChainModel
// ----------------------------------------------------------------------------
TaskChainModel::TaskChainModel(int vehicleId, const TaskSet& ts, double baseStepSeconds)
    : vehicleId_(vehicleId),
      dtMs_(baseStepSeconds * 1000.0),
      execMode_(ts.execMode),
      rng_(ts.seed ^ (static_cast<uint64_t>(vehicleId) * 0x9E3779B97F4A7C15ull)),
      netSC_(ts.netSC),
      netCA_(ts.netCA) {
    auto initJob = [&](Job& j, const TaskParams& p) {
        j.params = p;
        j.period = msToTicks(p.period_ms);
        j.offset = msToTicks(p.offset_ms);
        j.nextRelease = j.offset;
    };
    initJob(sensor_, ts.sensor);
    initJob(estimator_, ts.estimator);
    initJob(controller_, ts.controller);
    initJob(feedforward_, ts.feedforward);
    initJob(merger_, ts.merger);
    initJob(actuator_, ts.actuator);
}

long TaskChainModel::msToTicks(double ms) const {
    return std::lround(ms / dtMs_);
}

long TaskChainModel::sampleExecTicks(const ExecTimes& e) {
    double ms = e.avet_ms;
    switch (execMode_) {
        case ExecMode::Best:    ms = e.bcet_ms; break;
        case ExecMode::Average: ms = e.avet_ms; break;
        case ExecMode::Worst:   ms = e.wcet_ms; break;
        case ExecMode::Pert: {
            const double a = e.bcet_ms, m = e.avet_ms, b = e.wcet_ms;
            if (b <= a) { ms = m; break; }
            const double alpha = 1.0 + 4.0 * (m - a) / (b - a);
            const double beta  = 1.0 + 4.0 * (b - m) / (b - a);
            std::gamma_distribution<double> ga(alpha, 1.0), gb(beta, 1.0);
            const double x = ga(rng_), y = gb(rng_);
            ms = a + (x / (x + y)) * (b - a);
            break;
        }
    }
    return std::max<long>(2, msToTicks(ms));  // >=2 so finished is strictly after activated
}

long TaskChainModel::sampleDelayTicks(const ExecTimes& e) {
    double ms = e.avet_ms;
    switch (execMode_) {
        case ExecMode::Best:    ms = e.bcet_ms; break;
        case ExecMode::Average: ms = e.avet_ms; break;
        case ExecMode::Worst:   ms = e.wcet_ms; break;
        case ExecMode::Pert: {
            const double a = e.bcet_ms, m = e.avet_ms, b = e.wcet_ms;
            if (b <= a) { ms = m; break; }
            const double alpha = 1.0 + 4.0 * (m - a) / (b - a);
            const double beta  = 1.0 + 4.0 * (b - m) / (b - a);
            std::gamma_distribution<double> ga(alpha, 1.0), gb(beta, 1.0);
            const double x = ga(rng_), y = gb(rng_);
            ms = a + (x / (x + y)) * (b - a);
            break;
        }
    }
    return std::max<long>(1, msToTicks(ms));
}

TaskChainModel::Job& TaskChainModel::job(TaskKind k) {
    switch (k) {
        case TaskKind::Sensor:      return sensor_;
        case TaskKind::Estimator:   return estimator_;
        case TaskKind::Controller:  return controller_;
        case TaskKind::Feedforward: return feedforward_;
        case TaskKind::Merger:      return merger_;
        case TaskKind::Actuator:    return actuator_;
    }
    return sensor_;
}

void TaskChainModel::releaseIfDue(Job& j, long step) {
    if (step >= j.nextRelease) {
        if (j.active) ++missed_;  // previous job never finished -> deadline miss, dropped
        j.active = true;
        j.started = false;
        j.ranTicks = 0;
        j.execTicks = sampleExecTicks(j.params.exec);
        j.nextRelease += j.period;
    }
}

void TaskChainModel::beginTick(long step, std::vector<ReadyJob>& readyOut) {
    Job* all[] = {&sensor_, &estimator_, &controller_, &feedforward_, &merger_, &actuator_};
    for (Job* j : all) {
        j->grantedThisTick = false;
        releaseIfDue(*j, step);
    }
    // Only cloud tasks compete for shared cores.
    Job* cloud[] = {&estimator_, &controller_, &feedforward_, &merger_};
    for (Job* j : cloud) {
        if (!j->active) continue;
        const long release = j->nextRelease - j->period;
        readyOut.push_back(ReadyJob{vehicleId_, j->params.kind, j->params.period_ms,
                                    release, j->nextRelease,
                                    j->execTicks - j->ranTicks, j->started});
    }
}

void TaskChainModel::grantCore(TaskKind kind) {
    job(kind).grantedThisTick = true;
}

namespace {
// Advance one running job by a tick; emit activated on start, finished on
// completion. Returns true on the tick the job finishes.
bool advance(TaskChainModel::Job& j, bool& actFlag, bool& finFlag) {
    if (!j.active) return false;
    if (!j.started) {
        j.started = true;
        actFlag = true;
        j.ranTicks = 1;
    } else {
        ++j.ranTicks;
    }
    if (j.ranTicks >= j.execTicks) {
        finFlag = true;
        j.active = false;
        return true;
    }
    return false;
}

// Deliver at most one due packet (earliest scheduled arrival <= step).
bool fireDue(std::vector<long>& q, long step) {
    int best = -1;
    long bestv = std::numeric_limits<long>::max();
    for (int i = 0; i < static_cast<int>(q.size()); ++i)
        if (q[i] <= step && q[i] < bestv) { bestv = q[i]; best = i; }
    if (best >= 0) { q.erase(q.begin() + best); return true; }
    return false;
}
}  // namespace

void TaskChainModel::endTick(long step, VehicleTriggers& out) {
    out.clear();

    // In-vehicle tasks run as soon as released (their own per-vehicle resource).
    if (advance(sensor_, out.sensor_act, out.sensor_fin)) {
        out.net_sc_sent = true;
        scReceiveAt_.push_back(step + sampleDelayTicks(netSC_.delay));
    }

    // Cloud tasks advance only if the policy granted them a core this tick.
    if (estimator_.grantedThisTick)   advance(estimator_,   out.est_act,   out.est_fin);
    if (controller_.grantedThisTick)  advance(controller_,  out.ctrl_act,  out.ctrl_fin);
    if (feedforward_.grantedThisTick) advance(feedforward_, out.ff_act,    out.ff_fin);
    bool mergerFinished = false;
    if (merger_.grantedThisTick)
        mergerFinished = advance(merger_, out.merge_act, out.merge_fin);
    if (mergerFinished) {
        out.net_ca_sent = true;
        caReceiveAt_.push_back(step + sampleDelayTicks(netCA_.delay));
    }

    advance(actuator_, out.act_act, out.act_fin);

    // Deliver any due network packets.
    out.net_sc_recv = fireDue(scReceiveAt_, step);
    out.net_ca_recv = fireDue(caReceiveAt_, step);
}

}  // namespace cps
