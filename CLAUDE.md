# CLAUDE.md — Agent Bootstrap (read first, follow exactly)

Stable orientation for AI agents on this project. Volatile state lives in
`HANDOFF.md`. Rule of thumb: if you change how the project *works*, update
this file; if you change what is *true right now*, update `HANDOFF.md`.

## What this project is

REU research project (Dr. Guo's lab) on the Bosch RTAS 2026 "Physics-Driven
Real-Time CPS Challenge": N simulated vehicles' control chains share N_c cloud
cores; we study the end-to-end **data age** (staleness of the applied steering
command) under different scheduling policies, in this repo's harness around
Bosch's LateralMotionControl FMU.

Two deliverables:
- **Route A** (active): workshop paper — a proven analytical upper bound on
  data age, validated for soundness and tightness against this harness.
- **Route B** (builds on A): *age-criticality* scheduling — per-track-zone
  maximum tolerable age, enforced by a mode-switching cloud scheduler with a
  schedulability test. Extends Wilson et al. (MEMOCODE 2024).

Team: Stone (lead) + CS student (solver, sweeps, infra) + EE student (zone
tolerance, control side) + Kurt Wilson (PhD mentor: spot-checks formal claims;
students do the bulk of verification). Status and timeline: `HANDOFF.md`.

## Reading map (in order; stop when your task is covered)

Always:
1. This file.
2. `HANDOFF.md` — current state, baselines, key facts, open next-steps.
   Do NOT re-derive or contradict its "key facts" without evidence.

Then by task:
- **Running / experiments**: `USAGE.md` (build, flags, adding a scheduler).
- **Anything touching the age metric**: `DATA_AGE.md` — its §4 conventions
  ARE the definition of the measured quantity.
- **Theory / the bound**: `BOUND.md` (its status header says what is
  verified). Literature in `relatedPapers/`: Li et al. RTSS'24 (tightest
  multi-rate chain bounds) under DirectlyRelated...; Arafat et al. DAC'22
  (chain RTA machinery) and Wilson et al. MEMOCODE'24 (Route B's prior)
  under GuoLabSpecifics/.
- **Zone-tolerance experiments**: `ZONE_TOLERANCE.md`.
- **Scheduler / policy code**: `src/sched/` — `TaskModel.cpp` (`endTick` =
  stamp propagation, `releaseIfDue` = overrun policies),
  `PolicyScheduler.cpp`, `policies/*.cpp`, interfaces in `Scheduler.h` +
  `CorePolicy.h`.
- **FMU semantics**: `readme.md` (Bosch's) and
  `LateralMotionControl/sources/LateralMotionControl.c` —
  `process_trigger_events` is ground truth for data routing.
- **Challenge requirements / framing**: `ChallengeProposal/RTAS2026_Invited.pdf`
  (§III–IV) and `examples/*.md` (task parameters, constraints).

## Invariants (violating these silently invalidates the research)

1. `DATA_AGE.md` §4 conventions are the DEFINITION the bound is proven
   against. Never change them without explicit human sign-off; a change
   invalidates `BOUND.md` and every recorded baseline.
2. `age_path` is the bound's target; `age_fresh` is reaction latency.
3. Formal/soundness runs use `--exec worst` only (pert reorders packets vs
   stamps) and require `missed jobs: 0` (precondition P1) — check it.
4. Fixed-priority tie order is the strict total order (period, vehicle,
   kind). Changing tie-breaking or `--overrun` changes the analyzed system:
   re-run the `HANDOFF.md` baselines and update HANDOFF + `BOUND.md` §7 in
   the same commit.
5. `BOUND.md` is unverified draft until humans sign off. Numbers marked
   "preliminary / hand-iterated" must be machine-verified before use. No
   lemma goes into a paper without human re-derivation.
6. The FMU is a prebuilt black box — never edit or recompile it; all
   measurement is harness-side shadowing of its trigger events.
7. Git: push to `tempbosch` only. NEVER push to `origin` (the Bosch
   upstream). `relatedPapers/` stays untracked.

## How to work here

- **The simulator is the adversary.** Any run where measured `age_path`
  exceeds an instantiated bound is a counterexample — the most valuable
  possible result. Report it loudly; never smooth it over. Conversely,
  "measured ≤ bound" can validate a structurally WRONG bound (slack in one
  term masking an omitted term — this happened: a hold-free bound survived
  N=1 by 0.3 ms). Decompose gaps per term before claiming tightness.
- Before changing scheduling-visible behavior: predict the effect on the
  baselines, make the change, re-run, compare. Surprises are findings —
  document them in `HANDOFF.md`, don't tune them away.
- Verify load-bearing doc claims against code before relying on them (docs
  drift; code is truth). Cite `file:line` in discussions.
- Runs are cheap (~40× real time): settle questions empirically.

## Quick reference

    cmake -B build -S . -DCMAKE_BUILD_TYPE=Release && cmake --build build -j
    ./build/cps --headless --vehicles 6 --scheduler rm --exec worst --duration 30
    # schedulers: rm | prm | edf | context (oracle) | honest
    # other flags: --overrun kill|skip, --net-delay MS, --csv FILE, --seed N

## Before you end a session (this keeps the system alive)

1. Update `HANDOFF.md`: current state, changed baselines, done/open lists.
2. If you changed a convention, structure, or process: update the owning doc
   (`DATA_AGE` / `BOUND` / `ZONE_TOLERANCE` / `USAGE` / this file) in the
   same commit as the code change.
3. Commit in the existing log style (imperative summary + bullet body).
   Push to `tempbosch` when asked.
4. Keep this file stable and short (≤ ~150 lines). Detail belongs in the
   owning docs; state belongs in `HANDOFF.md`.
