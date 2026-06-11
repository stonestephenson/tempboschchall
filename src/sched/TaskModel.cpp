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

// Deliver at most one due packet (earliest scheduled arrival <= step). On
// delivery, also returns the data-age stamp that packet was carrying.
bool fireDue(std::vector<NetPacket>& q, long step, long& stampOut) {
    int best = -1;
    long bestv = std::numeric_limits<long>::max();
    for (int i = 0; i < static_cast<int>(q.size()); ++i)
        if (q[i].arriveStep <= step && q[i].arriveStep < bestv) { bestv = q[i].arriveStep; best = i; }
    if (best >= 0) { stampOut = q[best].stamp; q.erase(q.begin() + best); return true; }
    return false;
}

// Freshest (most recent) of two data-age stamps. Valid stamps are >= 0, so a
// plain max() ignores a -1 ("no data") unless both inputs are -1.
long freshest(long a, long b) { return a > b ? a : b; }
}  // namespace

void TaskChainModel::endTick(long step, VehicleTriggers& out) {
    out.clear();

    // ------------------------------------------------------------------ //
    // Data-age provenance tracking.
    //
    // Alongside the trigger generation below we carry a parallel "stamp" for
    // every data buffer the FMU owns (see process_trigger_events in
    // LateralMotionControl.c). A stamp is the sim tick at which the SENSOR
    // SAMPLE underlying that buffer's current data was taken. We do not
    // evaluate any latency formula -- the stamp rides the exact trigger events
    // this scheduler emits, so it accrues the run's real core contention and
    // sampled network delays, and we read the age off it at actuation.
    //
    // Convention (this IS the quantity the proven bound is defined against):
    //   * data age = current tick - tick the contributing sensor sample fired;
    //   * FEEDFORWARD is excluded: it carries the reference trajectory, not
    //     sensor data, so it propagates no stamp;
    //   * at the MERGER, FRESHEST-WINS among the sensor-derived inputs (the
    //     controller-feedback path and the merger's own direct estimator read).
    //
    // The FMU performs every receive/publish (its "finish" pass) before every
    // sample/compute/send (its "activate" pass) within one doStep, so we apply
    // stamp updates in that same order. Network receives are therefore done
    // first -- which is also behaviour-preserving for the triggers: a packet
    // pushed this tick arrives >= 1 tick later, so it can never be due this
    // tick, and pulling fireDue earlier delivers exactly the same packets.
    // ------------------------------------------------------------------ //

    // ---- Finish/receive pass: deliver due network packets first. ----
    long recvStamp = -1;
    if (fireDue(scReceiveAt_, step, recvStamp)) { out.net_sc_recv = true; scRecStamp_ = recvStamp; }
    if (fireDue(caReceiveAt_, step, recvStamp)) { out.net_ca_recv = true; caRecStamp_ = recvStamp; }

    // ---- In-vehicle Sensor: runs as soon as released. ----
    const bool sensorFinished = advance(sensor_, out.sensor_act, out.sensor_fin);
    if (out.sensor_act) sensCompStamp_ = step;            // sensor samples physical state now
    if (out.sensor_fin) sensOutStamp_  = sensCompStamp_;  // publish the sampled value
    if (sensorFinished) {
        out.net_sc_sent = true;
        scReceiveAt_.push_back(NetPacket{step + sampleDelayTicks(netSC_.delay), sensOutStamp_});
    }

    // ---- Cloud tasks: advance only if the policy granted a core this tick. ----
    if (estimator_.grantedThisTick) {
        advance(estimator_, out.est_act, out.est_fin);
        if (out.est_act) estCompStamp_ = scRecStamp_;     // reads received sensor data
        if (out.est_fin) estOutStamp_  = estCompStamp_;
    }
    if (controller_.grantedThisTick) {
        advance(controller_, out.ctrl_act, out.ctrl_fin);
        if (out.ctrl_act) fbCompStamp_ = estOutStamp_;    // reads estimator output
        if (out.ctrl_fin) fbOutStamp_  = fbCompStamp_;
    }
    if (feedforward_.grantedThisTick) {
        advance(feedforward_, out.ff_act, out.ff_fin);
        // Feedforward carries the reference, not sensor data -> no stamp (excluded).
    }
    bool mergerFinished = false;
    if (merger_.grantedThisTick) {
        mergerFinished = advance(merger_, out.merge_act, out.merge_fin);
        if (out.merge_act) aggCompStamp_ = freshest(fbOutStamp_, estOutStamp_);  // freshest-wins
        if (out.merge_fin) aggOutStamp_  = aggCompStamp_;
    }
    if (mergerFinished) {
        out.net_ca_sent = true;
        caReceiveAt_.push_back(NetPacket{step + sampleDelayTicks(netCA_.delay), aggOutStamp_});
    }

    // ---- In-vehicle Actuator. ----
    advance(actuator_, out.act_act, out.act_fin);
    if (out.act_act) actInStamp_  = caRecStamp_;          // reads received command
    if (out.act_fin) actOutStamp_ = actInStamp_;          // command becomes the applied output

    // ---- Hold-time age sample: age of the CURRENTLY-APPLIED command, taken
    //      every tick (not only when a fresh command latches) and maxed over
    //      the whole run, so a command held stale between updates is counted. ----
    if (actOutStamp_ >= 0) {
        const long age = step - actOutStamp_;
        if (age > maxAgeTicks_) maxAgeTicks_ = age;
    }
}

}  // namespace cps
