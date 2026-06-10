# Session Handoff — CPS Challenge Visualizer

Snapshot for a fresh agent to resume quickly. Last updated 2026-06-10 (PM —
research-kickoff session; see also BOUND.md and ZONE_TOLERANCE.md, new).

## Goal and research plan
Use this simulator to **empirically validate an analytical bound on end-to-end
data age** (latency) of the control chain. The measured worst-case `age_path`
must sit **at or below** the bound (soundness check); the gap shows the bound's
pessimism (tightness). Simulation can *refute* a bound (if ever exceeded) but
cannot *prove* it (a run may not hit the worst-case phasing).

The bound is being **derived in this project** — current draft in `BOUND.md`
(v0.1, needs verification). `relatedPapers/` (untracked) holds third-party
literature only: Guo-lab priors (Wilson et al. MEMOCODE'24 physics-aware MC;
Arafat et al. DAC'22 ROS2 chain RTA), Li et al. RTSS'24 (tightest multi-rate
chain reaction-time bound), the mixed-criticality canon (Vestal, AMC,
Burns–Davis), and foundations (Liu & Layland, Audsley).

**Plan of record** (10-week REU w/ Dr. Guo, ~6.5 weeks left as of 2026-06-10):
- **Route A** (now): workshop-grade paper — formal chain model + proven age
  bound + soundness/tightness experiments on this harness, plus capacity
  (Q1) and oracle-vs-honest (Q2) studies. Target: RTSS'26 WiP / workshop.
- **Route B** (builds on A): physics-derived *age-criticality* scheduling —
  per-track-zone max tolerable age (`ZONE_TOLERANCE.md` derives it
  empirically) enforced by a mode-switching cloud scheduler with a
  schedulability test. Target: RTAS'27. Extends Wilson et al. MEMOCODE'24
  from 1 vehicle/verification to N vehicles/scheduling theory.
- **Team**: user + CS student (sweeps, fixed-point solver, infra) + EE student
  (zone tolerance, control-side) + Kurt Wilson (PhD mentor: guidance and
  spot-checks on formal claims; students do the bulk of verification).

## Current state (2026-06-10 end of day)
- Remote **`tempbosch`** = `github.com/stonestephenson/tempboschchall`, branch `main`.
- **End-to-end data-age tracking: DONE**, dual conventions (`age_fresh`,
  `age_path`). Full design + rationale in **`DATA_AGE.md`**; candidate bound +
  draft RTA in **`BOUND.md`**; EE experiment spec in **`ZONE_TOLERANCE.md`**.
- Working tree clean except untracked `relatedPapers/` (user-added; not committed).

## What the data-age feature does (full detail in DATA_AGE.md)
Stamps each sensor sample with the tick it fires, rides that stamp through the
**real trigger events** (real core contention + sampled network delays) to the
actuator, computes `age = now − stamp` **every tick (incl. hold time)**, keeps a
per-vehicle running **max** → printed as `age_fresh(ms)` / `age_path(ms)`
columns + a `worst-case data age:` line in the headless metrics table.

**Conventions (the bound is defined against `age_path`):** feedforward excluded
(reference, not sensor data); hold time included; at the merger BOTH rules are
tracked — `age_fresh` = freshest contributing sample (≡ the S→E→M→A shortcut)
and `age_path` = oldest direct input (≡ the classical S→E→B→M→A chain), which
coincide up to the controller and satisfy `age_fresh ≤ age_path`.

## Key facts — do NOT re-derive these
- Measurement is **harness-side** (`src/sched/TaskModel.cpp` `endTick`). The FMU
  carries **no timestamps**; it only returns `e_y`, perf metrics, steering. Age
  is pure harness bookkeeping shadowing the FMU's data routing.
- For the bound comparison use **`--exec worst`** (fixed delays → FIFO preserved).
  `--exec pert` can reorder network deliveries (harness earliest-arrival vs FMU
  strict FIFO) and desync the stamp from the delivered data.
- `measured worst ≤ true worst ≤ analytical bound`.
- **The architecture is a flat cross-vehicle ready pool.** `PolicyScheduler`
  hands `CorePolicy::assign` the pool + a core *count* (no core identity).
  Partitioning is a choice *inside* `assign()` — `PartitionedRM` does exactly
  that (`vehicle % nCores`) with no architecture change.
- **Fixed-priority tie-breaks are the strict total order (period, vehicle,
  kind)** — deterministic across STLs and exactly the FP model BOUND.md §7
  analyzes. Stage-major (kind-first) ordering was tried and starves the whole
  Merger class under overload (BOUND.md §7.1).
- **`context` is an oracle, `honest` is the legitimate variant.** One class,
  two information sets (`ContextAware.cpp`): oracle scores on `*_real`
  (ground-truth) metrics the cloud could never see; honest scores on the
  estimator-derived remote metrics. In the triplicate metrics: `real` = grade,
  `in_remote_platform` = what the cloud scheduler may use,
  `in_local_platform` = the vehicle's view (currently unread/unused).

## Done in the 2026-06-10 PM session (uncommitted in working tree)
1. **Dual age conventions**: `age_fresh` (freshest-contributing = S→E→M→A
   shortcut) and `age_path` (oldest-direct-input = classical S→E→B→M→A chain;
   the bound's target) tracked in parallel; both printed, in `VehicleSummary`
   (recording format v3, loads v2), and in `--csv` rows. See DATA_AGE.md §4d.
2. **Honest ContextAware** (`--scheduler honest`): remote flags VR 1026/1038
   plumbed through `readOutputs` → `VehicleOutputs` → `VehicleView`. Finding:
   honest ≈ oracle at 6 veh/worst (estimation penalty ~0 at moderate load).
3. **PartitionedRM** (`--scheduler prm`). Finding: beats global RM at 6
   veh/worst (veh 3 soft 9.2% vs 13.4%; uniform 90.5 ms path age).
4. **Overrun toggle** `--overrun kill|skip` (kill = the Challenge's
   "kill-and-hold"; skip = continue-to-completion). Headline finding at 12
   veh/RM: kill silently destroys 4 vehicles (2 never actuate → age n/a +
   divergence); skip rescues 3 of them and makes the last failure visible as a
   29.9 s age.
5. **`--csv FILE`** append-mode summary rows for sweeps.
6. **BOUND.md**: candidate analytical bound v0.1 (parametric in R_i) — needs
   Kurt's line-by-line review. Soundness consistent (N=1: 90.5 ≤ 120.8;
   N=6: 100.5 ≤ 216.6). Instructive negative result in §5.4 (hold-free bound
   survives N=1 by 0.3 ms via slack cancellation).
7. **ZONE_TOLERANCE.md**: experiment spec for zone-wise max tolerable age
   (EE-student track). Phase 1a needs a tiny `--net-delay MS` flag (not yet
   implemented).

## Also done (second commit batch, same day)
8. **`--net-delay MS`** (fixes both network delays; CSV gained a net_delay_ms
   column). Validation: N=1/worst at delay 4 → 65.5 ms vs linear prediction
   66.5 — the 1.0 ms residual is phasing quantization (BOUND.md work item 2).
9. **Deterministic strict priority order (period, vehicle, kind)** in
   RM/EDF/PRM/Context tie-breaks: reproducible across STLs and *exactly* the
   FP model the new RTA (BOUND.md §7) analyzes. Vehicle-major chosen to match
   the Challenge's Q1 exemplar; a stage-major (kind-first) trial starved the
   entire Merger class at 12 veh (every chain dead) — kept as a finding, see
   BOUND.md §7.1.
10. **BOUND.md §7**: tick-quantum global FP RTA (exact per-tick interference
   argument), preliminary hand-iterated R_i at N=6/worst (all ≤ T ⇒ P1
   certified), per-vehicle bound instantiation: veh 5 167.2 vs measured 100.5
   (1.66×), veh 0 131.6 vs 110.5 (1.19×). Hand numbers need machine solving.

## Re-baselined numbers (vehicle-major order; worst exec, RM)
- N=1: 90.5 / 90.5 (fresh/path) — unchanged. With `--net-delay 4`: 65.5.
- N=6: fresh 90.5 / path 100.5, missed 0, veh 3 = 0.50700 / 13.43% (matches
  the original pre-tie-break baselines — the old libc++ order happened to act
  vehicle-major).
- 12 veh kill: veh 10–11 never actuate (n/a), identical to original baseline.
- 12 veh skip: **all 12 chains alive** (0 n/a), worst path age 505.5 ms.

## Open next-steps
1. **Kurt review of BOUND.md** (Lemma 1 pairing, hold-term composition, §7.2
   workload bound for the discrete model).
2. Machine-solve the §7 fixed points + sweep N for the certified-capacity
   number (CS-student script; also general CSV sweep automation).
3. EE student: ZONE_TOLERANCE.md Phase 1 (unblocked — `--net-delay` exists).
4. Offset/harmonic-aware sampling terms + limited carry-in (after 1–2).
5. (Optional) plumb `in_local_platform` metrics (VR 1025/1028/1031/1034/1037) if
   vehicle-side decisions are wanted; currently unread.

## Run / verify
```sh
cmake --build build -j
./build/cps --headless --vehicles 6 --scheduler rm --exec worst --duration 30
# compare: --scheduler rm|prm|edf|context|honest ; --overrun kill|skip ;
# raise --vehicles to stress contention ; --net-delay MS for delay sweeps ;
# --csv out.csv to accumulate sweep rows
```
Reference magnitudes (6 veh / 3 cores, RM): ~70 ms data age in `avg`,
90.5/100.5 ms (fresh/path) in `worst`, 0 missed jobs. N=1 worst: 90.5/90.5
(deterministic).

## Key files
- `DATA_AGE.md` — data-age design + decision rationale (dual conventions: §4d)
- `BOUND.md` — candidate analytical bound v0.1 + draft RTA (§7); review flags inline
- `ZONE_TOLERANCE.md` — EE experiment spec (per-zone max tolerable age, Q4)
- `USAGE.md` — build/run/controls + how to add a scheduler
- `src/sched/TaskModel.cpp` (`endTick`) — stamping + propagation + per-tick max;
  `releaseIfDue` — overrun policies (kill-and-hold / skip-next)
- `src/sched/PolicyScheduler.cpp` — per-tick selection (beginTick/assign/grantCore/endTick)
- `src/sched/policies/` — RateMonotonic, PartitionedRM, Edf, ContextAware
  (oracle+honest via InfoSet)
- `src/sched/Scheduler.h` (`VehicleView`), `CorePolicy.h` — interfaces
- `src/fmu/Fmu.cpp` (`readOutputs`) — which FMU outputs are read
