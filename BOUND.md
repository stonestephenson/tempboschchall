# Candidate Analytical Bound on End-to-End Data Age — v0.1 (for review)

Status: **draft derivation, not yet human-verified.** Every lemma needs a line-by-line
re-derivation (Kurt) before anything here is claimed in a paper. The numeric
instantiations below are consistent with measurement (soundness not refuted),
but consistency is not proof.

The quantity bounded is the **oldest-direct-input ("path") data age** measured by
the harness (`age_path` column; `TaskChainModel::maxDataAgeOldestTicks`). See
`DATA_AGE.md` for why this equals the classical S→E→B→M→A cause-effect-chain
age, and §6 below for the freshest-convention relation.

---

## 1. System model (what the harness implements)

Per vehicle v, a chain of periodic stages over base ticks of Δ = 0.1 ms:

```
S (T=5, in-veh) → [net SC: delay δ_SC] → E (T=10, cloud) → B (T=20, cloud)
                → M (T=20, cloud) → [net CA: delay δ_CA] → A (T=30, in-veh)
```

- **Register (implicit/last-is-best) communication**: each stage reads its input
  register at *activation* and publishes to its output register at *completion*
  (matches the Challenge paper §III-C read-execute-write and the FMU's
  finish-before-activate processing).
- **Releases**: synchronous (all offsets 0), periods as above, implicit deadlines.
- **In-vehicle stages** (S, A) run on dedicated resources: activation = release,
  response time = execution time.
- **Cloud stages** (E, B, M — and F, which carries no sensor data) contend for
  N_c cores under a global, fully-preemptive, free-migration policy with 1-tick
  quanta (`PolicyScheduler`).
- **Networks**: pure delays, FIFO, one packet per upstream completion; a packet
  sent at t arrives at t + δ, δ fixed per run in `--exec worst` (δ_SC = δ_CA = 16 ms).
- **Overrun policy: kill-and-hold.** A job unfinished at its next release never
  publishes; the register holds. (Under the bound's precondition P1 below this
  never triggers, so the bound is identical under skip-next.)

**Preconditions for the bound:**
- **P1 (no overruns):** every job of every stage completes within its period.
  Verified per run: `missed jobs: 0`. This gives R_i ≤ T_i and one publication
  per stage per period.
- **P2 (FIFO delivery):** fixed network delays (`--exec worst|avg|best`), so
  stamps pair with packets and register stamps are monotone. Excludes `pert`.
- **P3 (steady state):** the first full chain has propagated (the harness only
  ages applied commands, so warm-up is excluded by construction).

Notation per stage i: period T_i, WCET C_i, response time bound R_i
(release→completion; R_i ≤ T_i by P1), all in ms.

## 2. Definitions

- **stamp(ρ, t)**: the sensor-sample tick underlying register ρ's content at
  time t (oldest direct input through the merge; no recursion through the
  estimator's filter memory).
- **age(ρ, t) = t − stamp(ρ, t)**.
- **Effective applied age**: age(act_out, t) for all t — what the harness maxes
  per tick. This *includes hold time* (Challenge Q5 asks for the age of the
  *applied* command; the FMU applies act_out to the plant every tick).

## 3. Per-hop lemmas

**Lemma 1 (publication gap).** Under P1, a stage with period T and response
bound R publishes at least once in any half-open window of length T + R − C:
consecutive completions f_k, f_{k+1} satisfy f_{k+1} − f_k ≤ T + (R − C).
*Sketch:* releases are exactly T apart; a completion lies in [r + C, r + R].
Worst spacing: one job finishes as early as possible (r + C... earliest is r + C
only if it starts immediately; lower bound on f_k is r_k + C), the next as late
as possible (r_k + T + R). Gap ≤ T + R − C. ∎ *(check the direction of the
early/late pairing — this is the lemma most worth scrutiny)*

**Lemma 2 (register staleness at a reader).** If a writer satisfies Lemma 1 and
its published value has sample-age ≤ A_pub at each publication instant, then at
any read instant t (after the writer's first publication),
age(ρ, t) ≤ A_pub + (T_w + R_w − C_w).
*Sketch:* the last publication before t is at f* ≥ t − (T_w + R_w − C_w); its
content had age ≤ A_pub at f* and ages linearly until t. ∎

**Lemma 3 (stage traversal).** A stage reading at activation a and publishing at
completion f adds at most (f − a) ≤ R − C + C = R to the age between its read
and its publication. More precisely f − a ≤ R (a ≥ r, f ≤ r + R) and the read
value's age at publication is the read-time age + (f − a).
*(Tighter: f − a ≤ R − w where w = a − r is the waiting time; we keep R.)*

## 4. The bound

Composing along S → netSC → E → B → M → netCA → A:

- Sensor publishes with age C_S (samples at activation = release, publishes C_S
  later; in-vehicle ⇒ R_S = C_S).
- netSC arrival: + δ_SC. Arrivals inherit the sensor's publication cadence
  (gap ≤ T_S, since R_S = C_S makes Lemma 1's gap exactly T_S).
- E reads (Lemma 2 with writer = arrival process): + T_S; traverses: + R_E;
  publishes every ≤ T_E + R_E − C_E (Lemma 1).
- B reads: + (T_E + R_E − C_E); traverses: + R_B; publication gap T_B + R_B − C_B.
- M reads: + (T_B + R_B − C_B); traverses: + R_M; its completions feed netCA:
  arrival adds + δ_CA, arrival gap ≤ T_M + R_M − C_M.
- A reads: + (T_M + R_M − C_M); traverses: + R_A (= C_A, in-vehicle).
- **Hold**: the applied command then ages until the next actuator completion
  *re-latches* (Lemma 1 for A): + (T_A + R_A − C_A) = + T_A.

**Theorem (candidate).** Under P1–P3, for all t in steady state:

```
age_path(t) ≤ C_S + δ_SC + T_S
           + R_E + (T_E + R_E − C_E)
           + R_B + (T_B + R_B − C_B)
           + R_M + (T_M + R_M − C_M) + δ_CA
           + C_A + T_A
```

equivalently  **Σ_hops (sampling gap) + Σ_stages (response) + Σ network delays
+ actuator hold**, with each cloud stage contributing (T_i + 2·R_i − C_i).

### Instantiations (worst-case exec, δ_SC = δ_CA = 16, all ms)

With R_i = C_i (**uncontended**; exact at N=1, optimistic otherwise), each cloud
stage contributes R_i + (T_i + R_i − C_i) = T_i + C_i:

```
(C_S + δ_SC + T_S) + (T_E + C_E) + (T_B + C_B) + (T_M + C_M + δ_CA) + (C_A + T_A)
= 22.0 + 11.1 + 20.5 + 36.5 + 30.7 = 120.8
```

With R_i = T_i (**degenerate**, always sound under P1; cloud stage contributes
T_i + 2T_i − C_i):

```
22.0 + (10 + 20 − 1.1) + (20 + 40 − 0.5) + (20 + 40 − 0.5 + 16) + 30.7 = 216.6
```

*(F's WCET 2.5 appears nowhere: feedforward carries no sensor data. It does
contend for cores, so it enters only through the R_i of E/B/M.)*

### Soundness and tightness against measurement (worst exec, RM)

| Config | measured `age_path` | uncontended bound | degenerate bound |
|---|---|---|---|
| N=1 (R=C exact; deterministic, same at 30 s and 120 s) | **90.5** | 120.8 ✓ (1.34×) | 216.6 ✓ |
| N=6 / 3 cores | **100.5** | (R=C not valid here) | 216.6 ✓ (2.2×) |

No violation observed ⇒ soundness not refuted. The 1.34× gap at N=1 — where
R is exact — isolates the *structural* pessimism: the per-hop sampling terms
assume adversarial phasing, but releases are synchronous and the periods nearly
harmonic (5|10|20, with T_A = 30, gcd(20,30) = 10), so the real phase offsets
are fixed and benign. That slack is what work item 2 below attacks; closing it
matters more than sharpening R.

## 5. Week-2 work items (tightening)

1. **Real R_i for cloud stages**: a sufficient global-RM/EDF response-time bound
   for the 1-tick-quantum model. → **Draft framework now in §7** (the discrete
   model admits an exact per-tick interference argument); remaining: machine-
   solve the fixed points and re-derive the workload bound for this model.
2. **Offset/harmonic-aware sampling terms**: replace the (T + R − C) gaps with
   exact phase analysis under synchronous release (Li et al. RTSS'24's Δ = k·gcd
   quantization applies directly; T_A = 30 vs T_M = 20 gives gcd 10).
3. **Hold-term sharpening**: the actuator re-latches the *same* stamp if no
   fresher packet arrived; the + T_A term composes with the netCA arrival gap —
   check for double counting (I believe the composition is correct but it is
   the second-most-delicate step after Lemma 1).
4. **Refutation experiment — first attempt failed, instructively.** A
   hold-free variant of the bound (drop + T_A) gives 90.8 at N=1 (uncontended,
   R exact), and the measured N=1 path age is 90.5 — the hold-free bound
   *survives by 0.3 ms*. Decomposition: the chain-latency terms are loose by
   ≈ one actuator period at N=1 (actual latch-time age ≈ 60, bound says ≤ 90.8)
   while the real hold adds ≈ 30 — a near-exact cancellation. Two lessons for
   the paper: (i) "measured ≤ bound" can hold for a *conceptually wrong* bound
   when slack in one term masks the omission of another — quantitative
   tightness alone does not validate a bound's structure; decompose the gap
   per term. (ii) The clean refutation of hold-free bounds needs either the
   true R_i at N≥6 (work item 1: at N=6 measured 100.5 already exceeds 90.8,
   but hold-free-with-true-R is larger than hold-free-with-C and we cannot
   evaluate it until the RTA exists) or an actuator with a longer period.
   Re-run this experiment after work items 1–2 land.

## 6. Relation between the two measured conventions (see §7 for R_i)

At the merger, fb_psi_dot_out was computed from an *earlier* read of
est_states_out, and register stamps are monotone under P2. Hence
stamp_fb ≤ stamp_est always, so:

- oldest-direct = stamp_fb = **the S→E→B→M→A path** (what this bound covers);
- freshest = stamp_est = the S→E→M→A shortcut (reaction latency; shorter chain,
  same structure minus B's two terms ⇒ corollary bound = theorem − (T_B + 2R_B − C_B)).

Measured (6 veh, RM, worst): freshest 90.5 / path 100.5 — skew = 10 ms ≤
T_B + R_B as expected. ✓

---

## 7. Response-time analysis for the tick-quantum global model (W2, draft)

Status: **framework + preliminary hand-iterated numbers. The fixed points must
be machine-solved (CS-student script) and the lemma adapted-from-literature
re-derived (Kurt) before use.**

### 7.1 Why the implementation now matches an analyzable model

As of the tie-break change, every fixed-priority policy uses the strict total
order **(period, vehicle, kind)** — vehicle-major within a period tier,
matching the Challenge's own Q1 exemplar ("each additional vehicle's tasks at
lower priority"), with the within-vehicle chain order Controller < Feedforward
< Merger. Since each vehicle has at most one ready job per kind, this key is
unique: the scheduler realizes a **static per-task fixed-priority order**,
identical on every STL implementation. FP response-time analysis applies
*exactly* to what runs — no tie nondeterminism to over-approximate.

(Empirical aside that motivated the choice: a stage-major order
(period, **kind**, vehicle) was tried first. At 12 vehicles under kill-and-hold
it starved the entire Merger class — *every* vehicle's chain died, including
vehicle 0's. Vehicle-major degrades per vehicle instead: vehicles 10–11 die,
0–9 keep driving. Priority *structure*, not just priority *assignment*, governs
the overload mode — a finding in itself, and the hook for Audsley/Li-style
priority optimization as future work.)

### 7.2 Model and the interference argument

Discrete time, unit quanta, m identical cores, free migration/preemption: each
tick, the m highest-priority ready jobs advance one tick. For task k with the
higher-priority set hp(k) fixed by the total order:

A tick in which J_k is ready but not served has all m cores serving hp(k) work
(strictly: jobs above k; with the total order this is exactly hp(k)). So if x
is J_k's response time, the hp work executed inside its window is ≥
m · (x − C_k), giving the standard fixed point — which in this discrete model
is **exact per tick** (no fractional-quantum slop, a small rigor win over
continuous-time global RTA):

```
x = C_k + floor( (1/m) · Σ_{i ∈ hp(k)} W_i(x) )
```

with the workload bound (carry-in/jitter form; R_i from higher-priority tasks,
solved in priority order):

```
W_i(x) = ceil( (x + R_i − C_i) / T_i ) · C_i
```

Refinement to bring in after the basic version is verified: limited carry-in
(only m − 1 tasks can carry in — Guan-style RTA-LC), which §7.3's numbers do
NOT yet use except where noted.

The same computation certifies **P1** (no overruns): if x_k ≤ T_k for all k,
the kill-and-hold path is never taken, and the bound of §4 applies.

### 7.3 Preliminary numbers — N = 6, m = 3, worst exec (hand-iterated)

Ticks (0.1 ms): C_E = 11, C_B = 5, C_F = 25, C_M = 5; T_E = 100, T_20 = 200.
Priority: all E's (by vehicle), then vehicle blocks [B_v, F_v, M_v].

| task | hp set | R (ticks) | R (ms) |
|---|---|---|---|
| E_0 | ∅ | 11 | 1.1 |
| E_5 | 5 E's | 29 | 2.9 |
| B_0 | 6 E's | 27 | 2.7 |
| F_0 | 6E + B_0 | 48 | 4.8 |
| M_0 | 6E + B_0 + F_0 | 37 | 3.7 |
| B_5 | 6E + 15 band-20 | 107 | 10.7 |
| F_5 | + B_5 | 129 | 12.9 |
| M_5 | + F_5 | 117 | 11.7 |

All ≤ T ⇒ **P1 certified at N = 6 / worst** (preliminary). Instantiating §4's
theorem per vehicle:

- vehicle 5 (worst R's): 22.0 + 14.7 + 40.9 + 58.9 + 30.7 ≈ **167.2 ms**
  vs measured path age 100.5 → 1.66×.
- vehicle 0 (best R's): ≈ **131.6 ms** vs measured 110.5 → 1.19×.

Hold-free check (§5.4): 167.2 − 30 = 137.2 > 100.5 — still not refuted with
true-R; the refutation experiment needs higher load or a longer actuator
period, as predicted.

### 7.4 Open items

1. Machine-solve the fixed points (script over the CSV configs; also sweep N
   until some x_k > T_k — that N is the *certified* capacity, vs the larger
   empirical capacity: the certification gap is a headline number for Q1).
2. Re-derive the workload bound for the discrete synchronized-quantum model
   rather than citing continuous-time sporadic results (Kurt).
3. Limited carry-in (m−1) refinement, then offset/harmonic-aware sampling
   terms in §4 (these two interact; do them together).
4. Priority-order optimization over `kind` rank within a vehicle (the B→M
   handoff phasing is age-relevant: the stage-major experiment showed ±10 ms
   on path age at N=6 from tie structure alone).
