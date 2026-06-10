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

std::unique_ptr<CorePolicy> makePolicy(const std::string& name) {
    if (name == "edf")     return makeEdfPolicy();
    if (name == "context" || name == "ctx" || name == "adaptive")
                           return makeContextAwarePolicy();  // oracle (reads *_real)
    if (name == "honest" || name == "context-honest")
                           return makeContextAwareHonestPolicy();
    if (name == "prm" || name == "partitioned")
                           return makePartitionedRMPolicy();
    return makeRateMonotonicPolicy();  // "rm" / default
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

OverrunPolicy parseOverrun(const std::string& s) {
    if (s == "skip" || s == "skipnext") return OverrunPolicy::SkipNext;
    return OverrunPolicy::KillAndHold;  // "kill" / default
}

// Append the run's per-vehicle summary to a CSV (header written if the file is
// new/empty), so parameter sweeps can be aggregated across invocations.
// net_delay_ms is the --net-delay override; -1 = challenge default delays.
void appendCsv(const std::string& path, const RunRecording& rec,
               const std::string& scheduler, const std::string& exec,
               const std::string& overrun, double netDelayMs, uint64_t seed) {
    std::FILE* f = std::fopen(path.c_str(), "a");
    if (!f) {
        std::fprintf(stderr, "Warning: cannot open CSV file %s\n", path.c_str());
        return;
    }
    if (std::ftell(f) == 0)
        std::fprintf(f, "scheduler,profile,vehicles,cores,exec,overrun,net_delay_ms,seed,"
                        "duration_s,missed_jobs,veh,avg_perf,max_roll,soft_pct,hard,"
                        "max_age_fresh_ms,max_age_path_ms\n");
    for (int v = 0; v < rec.nVehicles; ++v) {
        const VehicleSummary& s = rec.summary[v];
        std::fprintf(f, "%s,%s,%d,%d,%s,%s,%.2f,%llu,%.3f,%ld,%d,%.6f,%.6f,%.4f,%d,%.2f,%.2f\n",
                     scheduler.c_str(), profileName(static_cast<Profile>(rec.profile)),
                     rec.nVehicles, rec.nCores, exec.c_str(), overrun.c_str(),
                     netDelayMs, static_cast<unsigned long long>(seed), rec.duration(),
                     rec.missedJobs, v, s.average_real, s.max_rolling_real,
                     s.soft_violation_pct, s.hard_violations,
                     s.max_data_age_ms, s.max_data_age_oldest_ms);
    }
    std::fclose(f);
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
        "  --scheduler rm|prm|edf|context|honest   scheduling policy (default rm;\n"
        "                               context = oracle, honest = remote metrics only)\n"
        "  --vehicles N                 number of vehicles (default 1)\n"
        "  --cores N                    shared cloud cores (default 3)\n"
        "  --profile 10|12.5|15         speed profile (default 10)\n"
        "  --duration SEC               sim seconds (default 0 = one lap)\n"
        "  --exec avg|worst|best|pert   execution-time model (default avg)\n"
        "  --overrun kill|skip          job overrun policy (default kill = kill-and-hold;\n"
        "                               skip = overrunning job finishes, releases skipped)\n"
        "  --net-delay MS               fix BOTH network delays to MS (overrides --exec\n"
        "                               for the networks; for delay-tolerance sweeps)\n"
        "  --seed N                     RNG seed for pert mode (default 0)\n"
        "  --headless                   run without the GUI, print metrics\n"
        "  --csv FILE                   append per-vehicle summary rows to FILE\n"
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
        const std::string execName    = argValue(argc, argv, "--exec", "avg");
        const std::string overrunName = argValue(argc, argv, "--overrun", "kill");
        params.execMode      = parseExec(execName);
        params.overrun       = parseOverrun(overrunName);
        params.netDelayMs    = std::atof(argValue(argc, argv, "--net-delay", "-1"));
        params.seed          = static_cast<uint64_t>(std::atoll(argValue(argc, argv, "--seed", "0")));
        const double durSec  = std::atof(argValue(argc, argv, "--duration", "0"));
        if (durSec > 0) params.durationSteps =
            static_cast<long>(durSec / vr::kBaseStepSeconds);

        const std::string schedName = argValue(argc, argv, "--scheduler", "rm");
        const std::string csvFile   = argValue(argc, argv, "--csv", "");
        auto scheduler = std::make_unique<PolicyScheduler>(makePolicy(schedName));

        Simulation sim(params, std::move(scheduler));

        if (hasFlag(argc, argv, "--headless")) {
            std::printf("Running headless: %s, %d vehicle(s), %d cores, profile %s, "
                        "exec %s, overrun %s\n",
                        schedName.c_str(), params.nVehicles, params.nCores,
                        profileName(params.profile), execName.c_str(), overrunName.c_str());
            sim.runToCompletion(true);
            if (!csvFile.empty())
                appendCsv(csvFile, sim.recording(), schedName, execName, overrunName,
                          params.netDelayMs, params.seed);
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
