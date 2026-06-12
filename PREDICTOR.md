# Held-Command Prediction: TTV, TTPNR, and the TimeToUnsafe Scheduler

What the predictor computes, the exact definitions and caveats, and how the
`ttu` policy and the visualization consume it. This is the Route B building
block: a **physically-derived dynamic deadline** per vehicle (Challenge Q4),
used online for scheduling (Q2) — the predictive successor to the reactive
`ContextAware` policy.

---

## 1. The two quantities

For each vehicle, ask: *if the actuator keeps holding its current (stale)
command, what happens?*

- **TTV — time to violation**: the first instant the predicted |e_y| crosses
  the hard 0.8 m bound under the held command.
- **TTPNR — time to point of no return**: the last instant at which switching
  to a *recovery* command (full assumed steering authority, see §3) can still
  keep |e_y| < 0.8 m forever after. **TTPNR ≤ TTV always** — the point of no
  return comes before the crash, and it is the real deadline: after it,
  giving the vehicle compute no longer helps (under the assumed limit).

Both are computed by rolling the plant model forward from the vehicle's
current state with the applied command frozen, against the *known* future
reference (ff_ref) and velocity from the trajectory — the reference is known
a priori, so even a cloud-side scheduler may legitimately use it. Values are
capped at the 500 ms horizon ("relaxed"); `ttpnr == 0` means **past PNR**.

## 2. The plant model (and why you can trust it)

`src/sim/Predictor.cpp` carries a **verbatim port** of the FMU's dynamics:
`calculate_matrices_from_velocity` (`LateralMotionControl.c:793–880`,
including the v < 0.1 clamp) and the state update `x⁺ = Ad(v)x + Bd(v)u +
Fd(v)r` (noise off ⇒ deterministic). The FMU itself is untouched (CLAUDE.md
invariant 6).

**Fidelity gate** (`--validate-predictor`): at every actuator latch, predict
e_y over the upcoming ~30 ms hold and compare against what the FMU then
actually does. Result on all three speed profiles (N=1, `--exec worst`, full
120 s lap): **4000 holds, 1.2 M tick-level samples, max |deviation| =
1.49e-08 m — the float32 storage floor.** The ported model is a tick-exact
replica. Re-run this gate after ANY change to the predictor.

Production rollouts use two documented approximations the gate does *not*
cover (it runs the exact path):
- **Velocity quantization**: matrices on a 0.01 m/s grid (shared cache) —
  sub-millimeter effect on predicted e_y over the 500 ms horizon.
- **Coarse stepping**: 1 ms blocks via a precomputed 10-tick affine
  composition per velocity cell (`x_{n+10} = Ad¹⁰x + ΣAdⁱ·(Bd u + Fd r)`),
  so TTV/TTPNR resolve to 1 ms — far finer than the 10 ms refresh cadence.

## 3. The steering limit and the recovery heuristic

The FMU's commanded steering angle (δ_des = `act_out`) is **amplitude-
unbounded** — only rate-shaped by the model's second-order steering dynamics.
"Unrecoverable" is therefore undefined without an assumed limit, which lives
**only in the predictor** (decision 2026-06-11):

| profile | max |act_out| observed (clean N=1 worst lap) | default δ_max (×1.5) |
|---|---|---|
| v10  | 0.1903 rad | **0.285** |
| v12.5| 0.3561 rad | **0.534** |
| v15  | 0.2793 rad | **0.419** |

Override with `--delta-max RAD`. Run a ±50 % sensitivity sweep before
claiming anything quantitative about PNR.

**Recovery law (heuristic, NOT certified reachability):** after a configurable
fresh-command latency (default 4 ms ≈ best-case chain), steer bang-bang at
±δ_max opposing `e_y + 0.3·e_y_dot`, through the model's own steering
dynamics. The steering sign convention is self-probed from the model at
startup, never hand-assumed. "Recovered" = back inside the 0.2 m comfort band
(after ≥ 50 ms) or never breaching for 1 s. TTPNR is found by binary search
over 5 ms-spaced hold snapshots, which assumes recoverability is **monotone**
in hold time — plausible but unproven. Consequences:
- TTPNR is an *estimate* with ~5 ms grid resolution, refreshed every 50 ms
  and aged in between (TTV/polyline refresh every 10 ms). The `min_pnr`
  summary statistic therefore depends on these cadences.
- A better recovery policy (e.g., LQR-based, or true reachable-set
  computation) would move PNR later; the bang-bang heuristic is conservative
  in spirit but not provably an under- or over-approximation. Refining this
  is an open EE/Kurt task.

## 4. Where the numbers flow

```
Simulation (cache: TTV+polyline @10ms, PNR @50ms, aged between)
  ├── VehicleView.ttv_ms / .ttpnr_ms  → policies (ttu ranks on them)
  ├── VehicleSummary.min_ttpnr_ms / .past_pnr_ticks → table + --csv
  ├── Frame.phys[6] / .ttv_ms / .ttpnr_ms (recording v4) → replay
  └── Simulation::prediction(v) → live visualizer overlay
```

- **`--scheduler ttu`**: rank vehicles by TTPNR ascending (the deadline),
  TTV tie-break, then the strict (period, vehicle, kind) order. Past-PNR
  cars clamp to maximum urgency by default — the real plant's steering is
  unbounded, so the controller may still save them; **`--triage`** inverts
  this (drop them to the bottom) for the rescue-vs-triage experiment.
- **`--scheduler hybrid`** (`Hybrid.cpp`): two-tier *guarded triage*.
  Vehicles with TTPNR below the guard `--guard MS` (default 150) form an
  emergency tier scheduled by ttu's rule; all remaining capacity goes to the
  comfort tier ranked by the shared comfort score (identical to `context`'s
  oracle rule by construction — `comfortUrgencyOracle` in `Policies.h`).
  Limits: guard → 0 ⇒ exactly `context`; guard ≥ horizon ⇒ exactly `ttu`
  (verified empirically, §5b). `--triage` applies as in ttu.
- **Visualizer**: for the selected car, a dotted predicted-e_y line ahead of
  it (same ×exaggeration as everything else), a red ring at the predicted
  0.8 m crossing, an orange diamond at the PNR point, and a HUD line
  ("pred: hits 0.8m in X ms — PNR in Y ms" / "PAST POINT OF NO RETURN").
  Works live and in replays of format-v4 recordings (`--select`, `--speed`,
  `--screenshot-at` help aim scripted screenshots).

## 5. First results (2026-06-11, worst exec, kill-and-hold, 30 s, 3 cores)

Sweep over N = 6..14 × {rm, prm, edf, context, honest, ttu}
(`predictive_sweep.csv`). Hard-breach totals (fleet) and the fleet-minimum
TTPNR ("closest call", ms; "-" = never below the 500 ms horizon):

| N  | rm        | edf      | prm       | context (oracle) | honest   | **ttu**      |
|----|-----------|----------|-----------|------------------|----------|--------------|
| 6  | 0         | 0        | 0         | 0                | 0        | 0            |
| 8  | 0         | 0        | 0         | 0                | 0        | 0            |
| 10 | 0         | 0        | 280 ⚠     | 0                | 0        | 0            |
| 12 | 4519 ☠2   | 3854 ☠2  | 5867 ☠3   | 0 (pnr 295)      | 0 (245)  | **0 (220)**  |
| 14 | 11619 ☠5  | 9457 ☠4  | 15911 ☠7  | 0 (pnr **0**)    | 3125     | **0 (150)**  |

(☠k = k vehicles never completed a chain; "pnr X" = fleet min TTPNR.)

Readings:
- **Safe capacity**: classic policies top out at N≈10–11 (prm < 10 —
  partition imbalance bites first). The reactive oracle (`context`) survives
  N=14 but with **zero** PNR margin — it reaches the brink. **ttu survives
  N=14 with 150 ms of margin fleet-wide**, despite both using ground-truth
  state: *prediction beats reaction on safety margin at equal information*.
- **Honest gap re-motivated**: the honest reactive variant collapses at N=14
  (3125 breaches). The phase-2 honest *predictor* is where that fight goes.
- **Comfort trade**: at N=12, context gets 17 % worst soft-time vs ttu's
  72 % — ttu buys margin with comfort. A hybrid (ttu term near deadlines,
  error term otherwise) is the obvious next policy.
- **Triage A/B**: identical to default at N≤14 — under ttu no vehicle ever
  goes past PNR, so the toggle never engages. It only differentiates beyond
  ttu's capacity or under injected delay (pair with `--net-delay`).

Prediction overhead: +17 % wall time at 12 vehicles (13× → 11× real time;
the plan's "≥20× at 12 veh" gate was mis-calibrated against the 6-vehicle
baseline — the no-predictor floor at 12 vehicles is already 13×).

## 5b. The hybrid: guarded triage (2026-06-12)

`hybrid` wraps ttu's safety guard around context's comfort optimization
(§4). Results (worst exec, kill, 3 cores, 30 s; "soft" = worst per-vehicle
soft-violation share, "floor" = fleet-min TTPNR in ms):

| N  | context        | ttu            | **hybrid (θ=150)** |
|----|----------------|----------------|--------------------|
| 12 | 16.7 % / 295   | 71.8 % / 220   | **16.7 % / 295** (≡ context: guard never fires) |
| 14 | 30.9 % / **0** | 75.5 % / 150   | **36.1 % / 35**    |

All cells are zero hard breaches. At light load the hybrid IS context,
bit-for-bit; at N=14 it keeps a positive safety floor for ~5 points of
comfort, where context reaches the brink (floor 0).

**The guard is a real dial** (N=14 sensitivity; floor rises ≈ 1:1 with θ):

| θ (ms)   | 100 | 150 | 200 | 250 | 300 | 400 |
|----------|-----|-----|-----|-----|-----|-----|
| worst soft | 32.9 | 36.1 | 42.8 | 33.7 | 33.7 | 35.4 |
| floor      | 0   | 35  | 110 | 120 | 205 | 295 |

The θ−floor gap is ≈ 100 ms ≈ the measured worst-case command round-trip
(data age): **achieved floor ≈ θ − round-trip**, because after the guard
fires, the rescue command still needs one chain traversal to take effect.
This is the empirical composition with BOUND.md: set θ ≥ desired floor +
(the age bound) and the floor is guaranteed-by-construction in spirit —
formalizing exactly that implication is the Route B theory hook.

Consequences worth quoting:
- **θ=300 dominates ttu at N=14** on both axes (33.7 % vs 75.5 % soft,
  205 vs 150 floor).
- **N=16 frontier**: θ=150 breaks (2416 hard) and context collapses
  (19 318); ttu survives (76.1 % / 145). **θ=400 also survives N=16 and
  dominates ttu there too (62 % / 235)**; θ=600 reproduces ttu's numbers
  exactly — the θ→∞ limit confirmed.
- So the guard should *scale with load* (round-trip grows with contention).
  An adaptive-θ policy (e.g., θ tracking the observed fleet-min margin or
  the live age measurement) is the natural next step — §6.

Data: `predictive_sweep.csv` (hybrid rows appended),
`hybrid_guard_sweep.csv` (N=14, rows grouped per guard in the order
100/150/200/250/300/400 — the guard value is not a CSV column),
`frontier_sweep.csv` (N=16).

## 6. Open items

1. **Adaptive guard**: θ scaled online with load (track the live worst-case
   age or the fleet-min margin) — §5b shows fixed θ must grow with N; the
   adaptive version should dominate ttu at every load.
2. δ_max sensitivity sweep (±50 %) and the triage-vs-rescue figure (triage
   never engages at N≤14 under ttu/hybrid — needs higher load or
   `--net-delay` injection).
3. Recovery-policy refinement / monotonicity assumption (EE + Kurt).
4. Honest-information variant: predict from estimated state + last-sent
   command via the InfoSet pattern (`ContextAware.cpp`) — phase 2.
5. Formalize the §5b composition with BOUND.md: bounded age ⇒ bounded
   guard-to-actuation lag ⇒ guaranteed floor (θ − bound). This is the Route
   B theorem shape: *a scheduler parameter with a physically provable
   safety-margin guarantee*.
6. Differentiate from Wilson et al. (MEMOCODE'24): their lookahead checks
   crash-vs-no-crash for one vehicle offline/at-verification-time; TTV/TTPNR
   are continuous online quantities driving multi-vehicle arbitration.
