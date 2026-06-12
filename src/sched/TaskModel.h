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

// What happens when a job is still unfinished at its next release.
//   KillAndHold: the overrunning job is killed (never publishes); the output
//     register holds its last published value; the next job releases on
//     schedule. This is Cervin's "Kill" overrun strategy + register hold —
//     the "kill and hold" of the Challenge paper (Sec. V.A).
//   SkipNext: the overrunning job keeps its accumulated execution and runs to
//     completion; releases that pass while it runs are skipped (each counted
//     as a miss).
enum class OverrunPolicy { KillAndHold, SkipNext };

// Full task set for one vehicle. challengeDefault() uses the example values from
// examples/parameters.md.
struct TaskSet {
    TaskParams    sensor, estimator, controller, feedforward, merger, actuator;
    NetworkParams netSC;  // sensor -> cloud
    NetworkParams netCA;  // cloud  -> actuator
    ExecMode      execMode = ExecMode::Average;
    OverrunPolicy overrun  = OverrunPolicy::KillAndHold;
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

// A delayed network packet in flight: the tick it will arrive, plus the
// data-age stamps it carries (the sensor-sample tick of the data inside it,
// under both merge conventions — see endTick; they only diverge downstream
// of the merger, so SC packets carry the same value twice).
struct NetPacket {
    long arriveStep;
    long stamp;     // freshest-contributing convention (S→E→M→A shortcut)
    long stampOld;  // oldest-direct-input convention (S→E→B→M→A path)
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
    // Worst-case end-to-end data age observed over the run, in base ticks
    // (-1 if no actuation has propagated yet). See endTick for the definition.
    // Freshest-contributing convention: age of the NEWEST sensor sample in the
    // applied command (multipath reaction latency; S→E→M→A shortcut lineage).
    long maxDataAgeTicks() const { return maxAgeTicks_; }
    // Oldest-direct-input convention: age of the OLDEST sensor-derived direct
    // register input in the applied command's lineage (no recursion through
    // the estimator's filter memory). Under FIFO delivery this equals the
    // S→E→B→M→A path age — the chain the analytical bound is proven for.
    long maxDataAgeOldestTicks() const { return maxAgeOldTicks_; }
    // Recent command round-trip estimate: max LATCH-TIME path age over a
    // ~2 s sliding window (two rotating buckets). Latch-time age — the age of
    // the data the moment a fresh command is applied — is the realized
    // sensor-to-actuation round-trip including scheduling delay, unlike the
    // per-tick max above which also accrues hold time. -1 = no latch yet.
    long recentLatchAgeTicks(long step) const;

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
    OverrunPolicy overrun_ = OverrunPolicy::KillAndHold;
    std::mt19937_64 rng_;
    long     missed_ = 0;

    Job sensor_, estimator_, controller_, feedforward_, merger_, actuator_;
    NetworkParams netSC_, netCA_;

    // Pending network "receive" events (arrival tick + carried data-age stamp).
    std::vector<NetPacket> scReceiveAt_;
    std::vector<NetPacket> caReceiveAt_;

    // --- Data-age provenance stamps (see endTick). Each holds the sim tick of
    //     the sensor sample underlying that buffer's current data; -1 = none.
    //     Up to the controller the two conventions coincide (single-input
    //     stages); they diverge at the merger, so only the merger-to-actuator
    //     buffers carry an extra *Old* twin (oldest-direct-input). ---
    long sensCompStamp_ = -1, sensOutStamp_ = -1;  // sensor sample / published
    long scRecStamp_    = -1;                       // received sensor packet
    long estCompStamp_  = -1, estOutStamp_  = -1;   // estimator
    long fbCompStamp_   = -1, fbOutStamp_   = -1;   // controller feedback
    long aggCompStamp_  = -1, aggOutStamp_  = -1;   // merger, freshest-wins
    long aggCompOldStamp_ = -1, aggOutOldStamp_ = -1;  // merger, oldest-direct
    long caRecStamp_    = -1, caRecOldStamp_  = -1;  // received command packet
    long actInStamp_    = -1, actInOldStamp_  = -1;  // actuator input
    long actOutStamp_   = -1, actOutOldStamp_ = -1;  // applied command
    long maxAgeTicks_    = -1;  // worst-case age over run (freshest)
    long maxAgeOldTicks_ = -1;  // worst-case age over run (oldest-direct)

    // Two-bucket rotating window for recentLatchAgeTicks (bucket = 1 s).
    static constexpr long kLatchAgeBucketTicks = 10000;
    long latchAgeBucketStart_ = 0;
    long latchAgeCur_  = -1;  // max latch age in the current bucket
    long latchAgePrev_ = -1;  // max latch age in the previous bucket

    Job& job(TaskKind k);
};

}  // namespace cps
