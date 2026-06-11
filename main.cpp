// CPS Challenge Visualizer — entry point.
//
// Builds a multi-vehicle co-simulation driven by a chosen scheduling policy and
// either visualizes it live (raylib) or runs it headless and prints metrics.
//
// ============================ Adding a scheduler =============================
// The plug-and-play part: to try a new scheduling method, drop a CorePolicy
// subclass into src/sched/policies/MyPolicy.cpp (CMake auto-globs it), declare
// its factory in sched/policies/Policies.h, and add one line to makePolicy()
// below. Swap which policy main builds and re-run — nothing else changes.
// For full control over the 16 FMU triggers, subclass Scheduler directly and
// pass it to the Simulation instead of a PolicyScheduler.
// ============================================================================
#include <algorithm>
#include <cctype>
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <memory>
#include <string>

#include "fmu/FmuVariables.h"
#include "sched/PolicyScheduler.h"
#include "sched/policies/Policies.h"
#include "sim/Recording.h"
#include "sim/Simulation.h"
#include "trace/Trajectory.h"
#include "viz/Visualizer.h"

using namespace cps;

namespace {

std::string schedulerKey(const std::string& name) {
    std::string key;
    key.reserve(name.size());
    for (unsigned char c : name) {
        if (c == '-' || c == '_') continue;
        key.push_back(static_cast<char>(std::tolower(c)));
    }
    return key;
}

std::unique_ptr<CorePolicy> makePolicy(const std::string& name) {
    if (name == "edf")     return makeEdfPolicy();
    if (name == "context" || name == "ctx" || name == "adaptive")
                           return makeContextAwarePolicy();
    return makeRateMonotonicPolicy();  // "rm" / default
}

struct SchedulerSelection {
    std::unique_ptr<CorePolicy> policy;
    bool partitioned = false;
};

SchedulerSelection makeSchedulerSelection(const std::string& name) {
    std::string key = schedulerKey(name);
    SchedulerSelection selected;
    const std::string partitionedPrefix = "partitioned";
    if (key.compare(0, partitionedPrefix.size(), partitionedPrefix) == 0) {
        selected.partitioned = true;
        key = key.substr(partitionedPrefix.size());
        if (key.empty()) key = "rm";
    }
    selected.policy = makePolicy(key);
    return selected;
}

Profile parseProfile(const std::string& s) {
    if (s == "12.5" || s == "12_5") return Profile::V12_5;
    if (s == "15")                  return Profile::V15;
    return Profile::V10;
}

ExecMode parseExec(const std::string& s) {
    if (s == "worst") return ExecMode::Worst;
    if (s == "best")  return ExecMode::Best;
    if (s == "pert")  return ExecMode::Pert;
    return ExecMode::Average;
}

const char* argValue(int argc, char** argv, const char* key, const char* def) {
    for (int i = 1; i < argc - 1; ++i)
        if (std::strcmp(argv[i], key) == 0) return argv[i + 1];
    return def;
}
bool hasFlag(int argc, char** argv, const char* key) {
    for (int i = 1; i < argc; ++i)
        if (std::strcmp(argv[i], key) == 0) return true;
    return false;
}

void usage() {
    std::printf(
        "CPS Challenge Visualizer\n"
        "  --scheduler rm|edf|context|partitionedRM|partitionedEDF|partitionedContext\n"
        "                               scheduling policy (default rm)\n"
        "  --vehicles N                 number of vehicles (default 1)\n"
        "  --cores N                    shared cloud cores (default 3)\n"
        "  --profile 10|12.5|15         speed profile (default 10)\n"
        "  --duration SEC               sim seconds (default 0 = one lap)\n"
        "  --exec avg|worst|best|pert   execution-time model (default avg)\n"
        "  --seed N                     RNG seed for pert mode (default 0)\n"
        "  --schedule-viz FILE.svg      write a core/job timeline SVG\n"
        "  --schedule-viz-ms MS         schedule timeline window (default 120)\n"
        "  --headless                   run without the GUI, print metrics\n"
        "  --save FILE                  write the run recording to FILE\n"
        "  --replay FILE                visualize a saved recording (no sim)\n"
        "  --help                       this message\n");
}

}  // namespace

int main(int argc, char** argv) {
    if (hasFlag(argc, argv, "--help") || hasFlag(argc, argv, "-h")) {
        usage();
        return 0;
    }

    const std::string replayFile = argValue(argc, argv, "--replay", "");
    const std::string saveFile   = argValue(argc, argv, "--save", "");

    try {
        // --- Replay a saved recording ---
        if (!replayFile.empty()) {
            RunRecording rec = RunRecording::load(replayFile);
            auto traj = Trajectory::load(static_cast<Profile>(rec.profile));
            std::printf("Replaying %s (%s, %d vehicles, %.1f s)\n", replayFile.c_str(),
                        rec.schedulerName.c_str(), rec.nVehicles, rec.duration());
            Visualizer viz(traj);
            viz.replay(rec);
            return 0;
        }

        // --- Build a fresh simulation ---
        SimParams params;
        params.profile       = parseProfile(argValue(argc, argv, "--profile", "10"));
        params.nVehicles     = std::max(1, std::atoi(argValue(argc, argv, "--vehicles", "1")));
        params.nCores        = std::max(1, std::atoi(argValue(argc, argv, "--cores", "3")));
        params.execMode      = parseExec(argValue(argc, argv, "--exec", "avg"));
        params.seed          = static_cast<uint64_t>(std::atoll(argValue(argc, argv, "--seed", "0")));
        const double durSec  = std::atof(argValue(argc, argv, "--duration", "0"));
        if (durSec > 0) params.durationSteps =
            static_cast<long>(durSec / vr::kBaseStepSeconds);

        const std::string schedName = argValue(argc, argv, "--scheduler", "rm");
        auto selected = makeSchedulerSelection(schedName);
        ScheduleVizConfig scheduleViz;
        scheduleViz.path = argValue(argc, argv, "--schedule-viz", "");
        scheduleViz.windowMs = std::max(1.0, std::atof(argValue(argc, argv,
                                                                "--schedule-viz-ms", "120")));
        auto scheduler = std::make_unique<PolicyScheduler>(std::move(selected.policy),
                                                           selected.partitioned,
                                                           scheduleViz);

        Simulation sim(params, std::move(scheduler));

        if (hasFlag(argc, argv, "--headless")) {
            std::printf("Running headless: %s, %d vehicle(s), %d cores, profile %s\n",
                        schedName.c_str(), params.nVehicles, params.nCores,
                        profileName(params.profile));
            sim.runToCompletion(true);
            if (!saveFile.empty()) {
                sim.recording().save(saveFile);
                std::printf("Saved recording to %s\n", saveFile.c_str());
            }
            return 0;
        }

        // --- Live visualization ---
        sim.start();
        std::printf("Live: %s, %d vehicle(s), %d cores, profile %s. Close window to exit.\n",
                    schedName.c_str(), params.nVehicles, params.nCores,
                    profileName(params.profile));
        VizConfig vcfg;
        vcfg.screenshotPath = argValue(argc, argv, "--screenshot", "");
        Visualizer viz(sim.trajectory(), vcfg);
        viz.live(sim);
        if (!saveFile.empty()) {
            sim.recording().save(saveFile);
            std::printf("Saved recording to %s\n", saveFile.c_str());
        }
        return 0;
    } catch (const std::exception& e) {
        std::fprintf(stderr, "Error: %s\n", e.what());
        return 1;
    }
}
