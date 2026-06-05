// TaskModel.h — declarative description of one vehicle's control task chain and
// a discrete-event model (TaskChainModel) that turns periodic job releases +
// per-tick core grants into the exact FMU trigger pulses.
//
// Tasks:  Sensor → [net SC] → Estimator → {Controller, Feedforward} → Merger
//         → [net CA] → Actuator.
// Sensor and Actuator are in-vehicle (each vehicle's own resource — they run as
// soon as released). Estimator/Controller/Feedforward/Merger are "cloud" tasks
// that contend for the shared N_cores via a CorePolicy. Networks are pure
// transmission delays (no CPU), event-driven by the upstream task finishing.
#pragma once

#include <cstdint>
#include <random>
#include <vector>

#include "fmu/FmuTypes.h"

namespace cps {

enum class TaskKind { Sensor, Estimator, Controller, Feedforward, Merger, Actuator };
constexpr int kNumTaskKinds = 6;

const char* taskKindName(TaskKind k);
// Cloud tasks contend for shared cores; in-vehicle tasks (Sensor/Actuator) do not.
bool isCloudTask(TaskKind k);

struct ExecTimes { double bcet_ms = 0, avet_ms = 0, wcet_ms = 0; };

enum class ExecMode { Best, Average, Worst, Pert };

struct TaskParams {
    TaskKind  kind;
    double    period_ms;
    double    offset_ms;
    ExecTimes exec;
};

struct NetworkParams {
    double    period_ms;  // transmission periodicity (informational for v1)
    ExecTimes delay;      // bc/av/wc transmission delay in ms
};

// Full task set for one vehicle. challengeDefault() uses the example values from
// examples/parameters.md.
struct TaskSet {
    TaskParams    sensor, estimator, controller, feedforward, merger, actuator;
    NetworkParams netSC;  // sensor -> cloud
    NetworkParams netCA;  // cloud  -> actuator
    ExecMode      execMode = ExecMode::Average;
    uint64_t      seed     = 0;

    const TaskParams& task(TaskKind k) const;
    static TaskSet challengeDefault();
};

// A cloud job that wants a core this tick (handed to the CorePolicy).
struct ReadyJob {
    int      vehicle;
    TaskKind kind;
    double   period_ms;
    long     releaseStep;
    long     deadlineStep;    // releaseStep + period
    long     remainingTicks;  // execution ticks still to run
    bool     started;         // already activated (running) vs just released
};

// Per-vehicle discrete-event chain model. Driven each tick in three phases:
//   beginTick()  -> release jobs, auto-run in-vehicle tasks & networks, list
//                   this vehicle's ready cloud jobs.
//   grantCore()  -> the policy marks which cloud kinds run this tick.
//   endTick()    -> advance granted cloud jobs and emit all trigger pulses.
class TaskChainModel {
public:
    TaskChainModel(int vehicleId, const TaskSet& ts, double baseStepSeconds);

    void beginTick(long step, std::vector<ReadyJob>& readyOut);
    void grantCore(TaskKind kind);
    void endTick(long step, VehicleTriggers& out);

    int vehicleId() const { return vehicleId_; }
    long missedJobs() const { return missed_; }

    // State machine for a single periodic task (public so the tick-advance
    // helper in TaskModel.cpp can operate on it).
    struct Job {
        TaskParams params;
        long period = 0, offset = 0;  // ticks
        long nextRelease = 0;
        bool active = false;          // released, not yet finished
        bool started = false;         // activated emitted
        long execTicks = 0;           // sampled execution for the current job
        long ranTicks = 0;            // execution ticks accumulated
        bool grantedThisTick = false;
    };

private:
    long msToTicks(double ms) const;
    long sampleExecTicks(const ExecTimes& e);
    long sampleDelayTicks(const ExecTimes& e);
    void releaseIfDue(Job& j, long step);

    int      vehicleId_;
    double   dtMs_;
    ExecMode execMode_;
    std::mt19937_64 rng_;
    long     missed_ = 0;

    Job sensor_, estimator_, controller_, feedforward_, merger_, actuator_;
    NetworkParams netSC_, netCA_;

    // Pending network "receive" events (tick at which the delayed packet lands).
    std::vector<long> scReceiveAt_;
    std::vector<long> caReceiveAt_;

    Job& job(TaskKind k);
};

}  // namespace cps
