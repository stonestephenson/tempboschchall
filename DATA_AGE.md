# End-to-End Data-Age Tracking

How the harness measures the **worst-case end-to-end latency** of the applied
control command — i.e. for each actuation, how old is the sensor data that
command traces back to — and the design decisions behind it.

This is the metric for **Challenge Q5** ("data age of the applied control
commands"). We measure it empirically, by tracking real provenance through each
run, so it can be compared against an analytical (paper-written) bound.

---

## 1. What we measure

> **Data age** = (tick the command is applied at the actuator) − (tick the
> sensor *sampled* the physical state that this command traces back to).

In base ticks of 0.1 ms; reported in ms. It spans the whole chain:

```
sensor sample → sensor exec → network (sensor→cloud) → estimator → controller
            → merger → network (cloud→actuator) → actuator → command applied
```

so it includes sensor execution, both network hops, all cloud queuing/execution
under real core contention, the actuator stage, **and** the time the command
then stays applied before a fresher one replaces it (the *hold time*).

We report, per vehicle, the **maximum** data age observed over the whole run,
plus the overall worst across vehicles.

## 2. Why this way — measured, not derived

We do **not** evaluate a latency formula. We attach a *timestamp ("stamp")* to a
sensor sample and let it ride the exact trigger events the scheduler emits, so
the number carries the run's real core contention and its real (sampled) network
delays. The stamp is read off at actuation. This makes the measurement an
empirical ground truth for *this run*, suitable for checking an analytical bound
rather than restating it.

It is implemented **in the harness** (`src/sched/TaskModel.cpp`), not by editing
the FMU. Two reasons:

1. The end-of-run metrics already live in the harness, and no FMU recompile is
   needed (the FMU ships as a prebuilt library).
2. Our chosen definition (see §4) deliberately ignores the estimator's internal
   filter memory — the one thing the harness cannot see — so the harness-side
   measurement is faithful to the definition.

## 3. How it works

Alongside the trigger generation, every data buffer the FMU owns gets a parallel
`long` **stamp**: the sim tick of the sensor sample underlying that buffer's
current data (`-1` = no data yet). The stamps are updated on the *same* trigger
events the scheduler emits, mirroring the FMU's data routing
(`process_trigger_events` in `LateralMotionControl/sources/LateralMotionControl.c`).

**Provenance chain** (sensor-derived path to the actuator):

```
sensor sample ─(sensor)→ sens_out ─[SC network FIFO]→ network_sc_rec
   ─(estimator)→ est_states_out ─(controller)→ fb_psi_dot_out
   ─(merger)→ agg_delta_out ─[CA network FIFO]→ network_ca_fb_rec ─(actuator)→ act_out
```

**Stamp rules** (applied in `endTick`, in the FMU's finish-before-activate order):

| Event | Stamp rule |
| --- | --- |
| sensor activates | sensor stamp ← current tick (the sample is taken now) |
| sensor finishes / sends | published sensor stamp pushed into the SC network FIFO |
| SC network receives | received-sensor stamp ← stamp popped from the FIFO |
| estimator activates | estimator stamp ← received-sensor stamp |
| controller activates | feedback stamp ← estimator's published stamp |
| **merger activates** | merged stamp ← **freshest(feedback stamp, estimator stamp)** |
| merger finishes / sends | published merged stamp pushed into the CA network FIFO |
| CA network receives | received-command stamp ← stamp popped from the FIFO |
| actuator activates | actuator-in stamp ← received-command stamp |
| **actuator finishes** | applied-command stamp ← actuator-in stamp |
| **every tick** | age = current tick − applied-command stamp; keep the running max |

Network packets carry their stamp as part of the in-flight packet, so a stamp
and its data stay paired through delay and any queueing. A stage that is **not
granted a core** does not advance, so its stamp does not progress — real
contention shows up as growing age automatically. A **dropped/missed job** never
re-publishes, so the older (staler) stamp keeps propagating — which correctly
*worsens* the measured age, exactly what a worst-case bound must capture.

## 4. The decisions that define the number

These conventions ARE the definition the bound is proven against. They are also
written in the `endTick` comment so the code and the bound stay in step.

### 4a. Origin = the sensor *sample* tick
The stamp is set when the sensor **activates** (when it reads the physical
state), not at its release or completion. So age measures from the instant the
sensor saw the world, and includes the sensor's own execution time.

### 4b. Freshest contributing sensor sample
The estimator is a **filter with memory**: each estimate blends the newly
received sensor sample with its own previous estimate. So the estimate depends
on data of *many* ages — there is no single source sample. We define age against
the **freshest** sensor sample that contributed (the newest information the
command reacts to).

*Why:* this is the standard cause-effect-chain / reaction-latency definition and
it is well-defined. The alternative ("oldest contributing sample") is unusable
here: because the filter always retains some fraction of every past estimate,
the oldest contribution traces back to the very first sample and the age would
grow without bound.

### 4c. Feedforward is excluded
The merged command combines a **feedback** branch (estimator → controller, which
traces to a sensor sample) and a **feedforward** branch. The feedforward branch
is computed from the reference trajectory (`ff_ref`), **not** from any sensor
reading. Since we are measuring the age of *sensor* data, the feedforward branch
carries no stamp and is excluded.

*Why it matters:* without this exclusion the metric would be ill-defined at the
merge — half the command's lineage isn't sensor data at all.

### 4d. Freshest-wins at the merger
At the merge the command depends on two sensor-derived inputs: the controller
feedback **and** the merger's own direct read of the estimator state. Both trace
to a sensor sample, possibly of different ages. We take the **freshest** of the
two (the most recent sample), consistent with 4b.

### 4e. Hold time is included
A command, once applied, stays applied until the actuator latches a fresher one
(the actuator runs on a ~30 ms period). Its data keeps aging the whole time. We
therefore sample the age of the **currently-applied command every tick** and
take the max over the run — not just the age at the instant of each fresh latch.

*Example:* a command latched at tick 100 carrying data stamped at tick 80, not
replaced until tick 150, reaches age 69 ticks at tick 149. The per-tick sampling
captures that 69; sampling only at latch time would record just 20 and miss the
true worst case.

## 5. Faithfulness and caveats

- **Ordering.** Within one FMU step, all receives/publishes happen before all
  samples/computes/sends. We apply stamp updates in that same order (network
  receives first), so cross-stage same-tick dependencies match the FMU. This is
  behaviour-preserving for the triggers because an in-flight packet always
  arrives at least one tick later than it was sent.
- **Use a fixed execution mode for the bound comparison.** In `best`/`avg`/
  `worst` every packet on a network gets the same delay, so deliveries stay
  FIFO and the stamp matches the data the FMU actually delivered. In `--exec
  pert`, per-packet random delays can reorder deliveries (the harness delivers
  earliest-arrival; the FMU pops strict FIFO), so a stamp may be paired with the
  wrong packet. **Do the formal comparison in `--exec worst`.**
- **Startup transient.** Age is only counted once a full chain has reached the
  actuator; the first valid actuation may carry a large warm-up age. If the
  bound is steady-state, consider discarding an initial window.

## 6. Where it lives in the code

| Concern | Location |
| --- | --- |
| Stamping + propagation + per-tick max | `src/sched/TaskModel.cpp` (`endTick`) |
| Stamp/`NetPacket` state | `src/sched/TaskModel.h` |
| Per-vehicle accessor (scheduler interface) | `Scheduler::maxDataAgeTicks(vehicle)` (`src/sched/Scheduler.h`), overridden in `src/sched/PolicyScheduler.cpp` |
| ticks→ms into the summary | `Simulation::finalizeSummary` (`src/sim/Simulation.cpp`) |
| Stored field | `VehicleSummary::max_data_age_ms` (`src/sim/Recording.h`, `.cpsr` format v2) |
| Printed column + overall line | `Simulation::runToCompletion` (`src/sim/Simulation.cpp`) |

## 7. Reading the output

Any headless run prints it in the metrics table:

```sh
./build/cps --headless --vehicles 6 --scheduler rm --exec worst --duration 30
```

```
  veh      avg_perf     max_roll      soft%       hard  max_age(ms)
  ...
  3         0.50700      2.73888     13.43%          0        90.50
  worst-case data age: 90.50 ms
```

`max_age(ms)` is the per-vehicle worst-case data age; the final line is the
overall worst across all vehicles. (`n/a` appears only if a scheduler does not
track it — i.e. a raw `Scheduler` that bypasses `PolicyScheduler`.)

Reference magnitudes (6 vehicles / 3 cores): ~70 ms in `avg`, ~90 ms in `worst`
— dominated by the two ~8–16 ms network hops plus the ~30 ms actuator-period
hold. The number climbs as you add vehicles (cloud tasks get starved), which is
where a context-aware scheduler should beat rate-monotonic.
