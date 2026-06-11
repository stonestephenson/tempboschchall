# CPS Challenge Visualizer — Usage

A plug-and-play harness + raylib visualizer for the Bosch RTAS 2026
Physics-Driven Real-Time CPS Challenge. It drives the `LateralMotionControl` FMU
for N vehicles sharing `N_cores` cloud cores under a scheduling policy you choose,
records the run, and shows the cars lapping the track — the reference path, the
actual driven path, the lateral error, and where a car breaches its bounds.

The FMU itself is documented in [`readme.md`](readme.md); the task/network/metric
parameters are in [`examples/`](examples).

## Build

Requires CMake ≥ 3.16 and a C++17 compiler. raylib is fetched and built
automatically the first time you configure (needs network once).

```sh
cmake -B build -S . -DCMAKE_BUILD_TYPE=Release
cmake --build build -j
```

The executable is `build/cps`.

## Run

```sh
# Live visualization (opens a window). Defaults: RateMonotonic, 1 vehicle, v=10.
./build/cps

# Multiple vehicles sharing 3 cloud cores, context-aware scheduler:
./build/cps --vehicles 6 --scheduler context

# Partitioned RM: vehicle v is assigned to cloud core v % N_cores.
./build/cps --headless --vehicles 6 --cores 3 --scheduler partitionedRM --duration 120

# Write a zoomed core/job timeline for the first 120 ms of any scheduler.
./build/cps --headless --vehicles 6 --cores 3 --scheduler rm \
  --duration 2 --schedule-viz rm-schedule.svg

# Headless (no window): run fast and print per-vehicle metrics.
./build/cps --headless --vehicles 6 --scheduler rm --duration 120

# Save a run, then replay/scrub it later without re-simulating:
./build/cps --headless --vehicles 4 --save run.cpsr
./build/cps --replay run.cpsr
```

Key options: `--scheduler rm|edf|context|partitionedRM|partitionedEDF|partitionedContext`,
`--vehicles N`, `--cores N`,
`--profile 10|12.5|15`, `--duration SEC`, `--exec avg|worst|best|pert`,
`--seed N`, `--schedule-viz FILE.svg`, `--schedule-viz-ms MS`, `--headless`,
`--save FILE`, `--replay FILE`, `--screenshot FILE`.
Run `./build/cps --help` for the full list.

### Visualizer controls

| Key | Action | Key | Action |
| --- | --- | --- | --- |
| `Space` | play / pause | `[` `]` | select previous / next vehicle |
| `←` `→` | scrub / step (replay) | `F` | follow selected car / overview |
| `↑` `↓` | playback speed | `wheel` | zoom |
| `,` `.` | error exaggeration | `H` | toggle help |

The track shows the **gray centerline** (expected path), **yellow ±0.2 m** soft
(comfort) bounds and **red ±0.8 m** hard (safety) bounds. The driven path is
offset from the centerline by the lateral error, exaggerated (default ×25) so
it's visible, and colored green→red by error magnitude. Hard breaches are marked
red on the track and on the timeline so you can jump straight to them.

## Add your own scheduling method

The common case is a new **core-arbitration policy**: given the cloud jobs that
want to run this tick (across all vehicles) and the shared core count, decide
which get a core. You may use each vehicle's live control metrics (`VehicleView`)
for context-aware decisions.

1. Create `src/sched/policies/MyPolicy.cpp` (auto-picked up by CMake):

   ```cpp
   #include <algorithm>
   #include "sched/policies/Policies.h"

   namespace cps {
   namespace {
   class MyPolicy : public CorePolicy {
   public:
     void assign(const std::vector<ReadyJob>& ready, int nCores,
                 const std::vector<VehicleView>& ctx,
                 std::vector<int>& chosen) override {
       // ... pick up to nCores indices into `ready` ...
     }
     const char* name() const override { return "MyPolicy"; }
   };
   }  // namespace
   std::unique_ptr<CorePolicy> makeMyPolicy() {
     return std::unique_ptr<CorePolicy>(new MyPolicy());
   }
   }  // namespace cps
   ```

2. Declare `makeMyPolicy()` in [`src/sched/policies/Policies.h`](src/sched/policies/Policies.h)
   and add a case to `makePolicy()` in [`main.cpp`](main.cpp). Rebuild — done.

See [`RateMonotonic.cpp`](src/sched/policies/RateMonotonic.cpp) and
[`ContextAware.cpp`](src/sched/policies/ContextAware.cpp) for worked examples.

For full control over the 16 FMU triggers (e.g. data-driven, aperiodic triggering
— Challenge Q6), subclass `Scheduler` directly (see
[`src/sched/Scheduler.h`](src/sched/Scheduler.h)) and pass it to the `Simulation`
instead of a `PolicyScheduler`.

## Layout

```
src/fmu/    FMI 2.0 wrapper (loads the dylib once, N instances) + value refs
src/trace/  loads a profile's reference track + per-tick FMU inputs (CSVs)
src/sched/  Scheduler interface, declarative task model, CorePolicy, policies/
src/sim/    co-simulation master loop + run recording
src/viz/    raylib visualizer
main.cpp    pick scheduler / vehicles / profile / mode
```
