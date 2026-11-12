# Pushkaralu 2027: Technical Report
## PS-01 Real-Time Crowd Monitoring · PS-03 Lost-Person Reunification

**Challenge:** RTIH × RMC *Innovation-Led Solutions for Godavari Pushkaralu 2027*
**Status as of:** 12 Nov 2026. Submission deadline is 15 Nov 2026 (extended); demo/pitch day is 24 Nov 2026.
**Scope:** every approach tried, including the ones that failed, every result with its exact numbers, the limitations, and how to reproduce each result. All numbers come from the files in `pushkaralu/sim/results/`.

---

## 0. Status at a glance

| Item | Status |
|---|---|
| Literature survey (86 verified references) | ✅ `literature_survey.md`, `references.bib` |
| Pedestrian simulator verified (RiMEA tests 1, 4, 12) and calibrated (Weidmann) | ✅ |
| PS-01 algorithm: route-graph quorum-sensing alerts | ✅ simulated (compartment and site scale) |
| PS-01 closed-loop agent-based test of the **final** route-graph algorithm | ✅ E3, 5 seeds: **0 s above 4 p/m² in 5 of 5** (§2.4) |
| PS-01 repeated runs | ✅ E3 5 seeds, E8 20 random scenario draws (§2.4, §2.6) |
| PS-01 node (board) failures | ✅ found a real weakness (one dead board ≈ no system), fixed in firmware in 3 steps, tested in SIL, JuPedSim and the site model; remaining gaps: 2 adjacent dead boards, a dead approach-road board (§2.8) |
| PS-01 hybrid ICCC + mesh | ✅ (§2.9) |
| Mathematical model checked against the simulations | ✅ E11 (§2.10) |
| PS-03 phone-free reunion design + simulations | ✅ |
| PS-01 realistic sensing (biased counters, noisy camera readings, Kalman fusion) | ✅ E12 (§2.11): counters alone fail 15 of 20 surges; the fused layout is safe in **100 of 100** surge and 100 of 100 blocked-exit draws; sensor requirement derived; camera and board failures tested |
| PS-01 firmware v4 options (light timing, dynamic shares, dead-node release) | ✅ E13 (§2.12): a 120 s dwell on every light drop is **unsafe** (held crowds upstream pass 4 p/m²); amber-only slowing is safe and cuts light changes ~35%; the other two are neutral |
| Field response: police delay, partial compliance | ✅ E14 (§2.13): full compliance safe even at 3 min delay; 70 % compliance needs action within 60 s (120–180 s: 14 and 9 of 20 safe) |
| Alert-threshold sensitivity | ✅ E15 (§2.14): every setting safe in 20/20; θ_watch trades holds against peak |
| Realistic sensing in the JuPedSim compartment | ✅ E16 (§2.15): found an estimator lag (0/5 safe); fixed with a physics-based random walk (5/5), now the default in Python and firmware |
| ESP32 firmware (3 roles) | ✅ compiles. Runs the **route-graph QS** with node-failure handling. ✅ Firmware code tested in software-in-the-loop (SIL) against the Python model: 100% identical (§4.1).  The final change (held + bypassed = the larger level) was re-checked on 31 Oct under WSL2 `g++`: still 100% identical. ⚠ Not yet flashed or tested on boards |
| Backend (REST API for the command centre) + dashboard | ✅ end-to-end tests pass (`backend/test_app.py`) |
| Concept notes, pitch decks, demo videos | ✅ concept note, architecture and budget updated 11–12 Nov (E12–E16; budget costed node by node with sourced prices); demo video re-cut with voiceover 12 Nov |
| Eligibility of a student team | ✅ the official guidelines admit students who are Indian citizens (checked 11 Nov) |
| DPDP Rules 2025 Gazette check (child tracking, §9) | ⚠ pending |

---

## 1. Simulation framework and why it is credible

### 1.1 Acceptance basis
No government "approves" a crowd simulator. Regulators accept **validated tools checked against standard test suites**, so we use three layers of credibility:
1. **A recognised tool.** JuPedSim (Forschungszentrum Jülich, open source).
2. **Standard verification.** The RiMEA guideline test cases, the de facto international benchmark (Vadere runs them on every code change).
3. **Calibration** against Weidmann's fundamental diagram.

The supporting frameworks we cite are ISO 20414:2020, NIST TN 1822 and, for India, the NDMA guide *Managing Crowds at Events and Venues of Mass Gathering* (2014).

### 1.2 RiMEA verification (`sim/rimea_tests.py`, default time gap 1.0 s)
| Test | Requirement | Measured | Result |
|---|---|---|---|
| RiMEA 1: walking speed in a 40 m corridor | 26–34 s | 30.1 s | Pass |
| RiMEA 4: fundamental diagram | speed falls with density | RMSE 0.202 m/s vs Weidmann | Pass |
| RiMEA 12: bottleneck | congestion before, none after | 5.0 vs 0.38 p/m²; flow 0.85 p/(m·s) | Pass |

Fundamental diagram, measured vs Weidmann (m/s): 0.5 p/m² → 1.139 vs 1.326 · 1.0 → 0.70 vs 1.06 · 2.0 → 0.45 vs 0.61 · 3.0 → 0.283 vs 0.336 · 4.0 → 0.21 vs 0.159 · 5.0 → 0.157 vs 0.038.

### 1.3 Calibration (`sim/calibrate.py`)
The model's time gap *T* was swept against Weidmann's diagram and against the RiMEA 12 bottleneck flow.

| T (s) | FD RMSE (m/s) | Bottleneck flow p/(m·s) | Note |
|---|---|---|---|
| 1.0 | 0.198 | 0.85 | default |
| 0.7 | **0.108** (best fit) | **0.11** | **rejected: the bottleneck clogs** |
| **0.5** | 0.207 | **1.46** | **chosen** |
| 0.35 | 0.339 | 1.85 | |

**Criterion:** lowest error, subject to bottleneck flow ≥ Weidmann's peak capacity of **1.22 p/(m·s)**. Picking the best fit alone (T = 0.7) would have produced unrealistic clogging. The ghat scenarios use T = 0.5, loaded from `results/calibration.json`.

### 1.4 Compartment model (`sim/ghat_scenario.py`)
A Kotilingala-style compartment. The dimensions are **assumed** until RTIH shares the model-compartment specs.
- **Approach road** 60 × 8 m, in two zones: R1 (240 m²) and R2 (240 m²).
- **Entry gate** 3 m wide. The gate-approach zone G is 64 m²; this is the 2015 failure point.
- **Compartment** 30 × 20 m: holding area C1 (300 m²) and bathing steps C2 (300 m²). Pilgrims stand in the river 60–180 s.
- **Exit corridor** 2 m wide (zone X, 16 m²).
- **Arrivals:** desired speed N(1.25, 0.2) m/s, clipped to 0.6–1.8 m/s.
- **Scenarios:**
  - **normal:** 1.2 arrivals/s;
  - **surge:** 1.2 → 4.5/s from 300 to 900 s, the day-1 surge;
  - **blocked:** 1.5/s, with the exit blocked from 300 s.
- **Uncontrolled peak densities (p/m²):**
  - normal: X 1.38, all else < 0.4;
  - **surge: C2 6.05, C1 4.59**, X 2.56, G 1.67;
  - **blocked: X 4.31**, C2 1.45.
- **Sensing:** six nodes (R1, R2, G, C1, C2, X), each reporting zone density every second, plus crowd pressure ρ·Var(v), the measure of Johansson et al. 2008. Danger means any core zone (G, C1, C2, X) ≥ 4 p/m² for 10 s.

---

## 2. PS-01: Crowd monitoring and early warning

### 2.1 Problem framing
- **The 2015 Pushkaralu failure:** 27 deaths in a day-1 surge at a narrow entrance.
- **Today's tool:** CCTV counts updated hourly. IIIT-H's 2015 system reached about 70–75% accuracy.
- **The physics:** a crush builds when inflow exceeds outflow. The people who can stop it (police on the approach road) can't see it. **The warning must travel upstream, early, and keep working when the network or the command centre fails.**
- **Literature thresholds:**
  - 4 p/m², the Green Guide limit for a moving crowd;
  - 5–6 p/m², where shock waves become possible (Still);
  - flow collapse, stop-and-go waves and crowd turbulence come 10–30 min before a disaster (Helbing 2007, Johansson 2008).

### 2.2 Architecture
- **Sensing (revised after E12, §2.11):** line counters (ToF, shaded) only at gates and exits, where barricades already form lanes; on wide roads a density reading from existing CCTV or an edge camera wired to the node; a 2-state Kalman filter per node fuses them. The alert algorithm takes density from any source.
- **Node:** an ESP32 running the alert algorithm locally, with a 3-colour output.
- **Mesh:** ESP-NOW for short hops, LoRa for long hops. LoRaWAN uplink is unsuitable for low-latency alerts (Adelantado 2017), so we use peer-to-peer.
- **Command centre:** a FastAPI backend that aggregates for the ICCC, with documented REST/OpenAPI at `/docs`. **Decisions never depend on it.**

### 2.3 Algorithm evolution (what failed and why)

**v1: averaging diffusion ("Laplacian"). Failed.**
A_i(t+1) = δA_i + (1−δ)s_i + D·Σ_j w_ij (A_j − A_i)
- Surge: CRITICAL came **49 s after** the danger point (lead −49 s), and the road was **never** warned.
- **Why:** averaging pulls the hotspot down towards its calmer neighbours. The literature survey had flagged exactly this pitfall. It is kept only as an ablation.

**v2: gradient relay.**
A_i(t+1) = δA_i + (1−δ)·max(s_i, γ·max_j A_j)
- The hotspot keeps its level, and neighbours hear γ, γ², … of it.
- **Rate-term fix:** in normal flow the rate term produced **72 CRITICAL alerts/h**, because one person entering a 16 m² zone looks like a surge. Fix: the rate counts only at ρ ≥ 1.5 p/m² and is measured over 30 s. False alerts became **0**.
- **γ sweep** (`e5_gamma_sweep.csv`, road lead in s):

| γ | Surge core lead | Surge road lead | Blocked core | Blocked road | False WATCH/h |
|---|---|---|---|---|---|
| 0.7 | 102 | **−25** | 107 | 85 | 0 |
| 0.8 | 102 | 68 | 107 | 92 | 0 |
| **0.9** | 102 | **121** | 107 | 96 | 0 |
| 0.95 | 102 | 146 | 107 | 98 | 0 |

- **Chosen γ = 0.9**, about 3 hops of reach.
- **Remaining weakness:** the signal still decays per hop, and it reaches nodes that don't feed the hotspot. At site scale this caused heavy over-alerting (§2.6).

**v3 (final): route-graph QS. Alerts follow the pilgrim route** (`sim/route_graph.py`, `qs_core.RouteQSNetwork`).
- **The site as a directed graph:**
  - nodes: Nagara/source, road, junction, gate, compartment, exit;
  - edges: route share, i.e. the fraction of the upstream node's outflow that takes the edge, and walk time;
  - node capacity = width × 1.22 p/(m·s).
- **Own level:** each node keeps A_i = δA_i + (1−δ)s_i, which stays local.
- **Relayed threat** (heard from the nodes it sends pilgrims to):

  threat_i = Σ_{j ∈ successors} share_ij · L_j,  where L_j = max(A_j, threat_j)

  This is the **fraction of my outflow heading into trouble**. A junction whose 4 branches are all crowded hears the full alarm; one crowded branch out of four gives ¼.
- **Actions** (through a hysteresis state machine on the threat):
  - WATCH → METER (50% flow);
  - CRITICAL → HOLD (10% trickle);
  - **junction DIVERT:** the share of a branch whose own level is ≥ 0.6 moves to sibling branches below 0.3 that still have room (density behind them < 2.0 p/m²).
- **Unchanged from v2:** hysteresis, minimum dwell time, staleness timeout and rate floor.
- **Two design corrections the site test forced** (§2.6):
  1. **Single-branch share weighting → SUM over branches.** Junctions never acted, only small gates held, and the gates jammed.
  2. **The time-horizon rule was dropped.** "Hold only within 180 s walk of the hotspot" jammed the small near nodes; holding further upstream, where there is buffer space, is safer.
  - There was also a **diversion bug**: diversion was tested on the *share-weighted* level (¼ of the alarm), so it never triggered. It now uses each branch's own level.

**Final parameters** (`QSParams`, `RouteParams`):
- ρ_crit = 4.0;
- δ = 0.85;
- rate horizon 60 s, rate floor 1.5 p/m², rate window 30 s;
- θ_watch = 0.6, θ_crit = 0.85, hysteresis 0.1;
- minimum dwell 5 s;
- staleness timeout 5 s;
- γ = 0.9 (gradient mode only);
- use_share = True, use_horizon = False, meter = True.

### 2.4 Compartment-scale results (`qs_alert.py`)

**E1: alert timing (lead before danger; danger at 669 s in the surge, 438 s in the blocked scenario):**
| Method | Surge core | Surge road | Blocked core | Blocked road |
|---|---|---|---|---|
| Threshold (today's practice) | 47 s | never | 83 s | never |
| Threshold + rate (ablation) | 107 s | never | 294 s | never |
| Centralised command centre (backhaul up) | 107 s | 185 s | 294 s | 294 s |
| Gradient QS (v2) | 102 s | 121 s | 107 s | 96 s |
| Laplacian QS (v1, ablation) | −49 s | never | 97 s | never |
| **Route-graph QS (v3)** | **102 s** | **172 s** | **107 s** | **108 s** |

**E2: robustness (road warned %, median road lead), no backhaul needed:**
| Link loss | Gradient QS, surge | Route QS, surge | Gradient QS, blocked | Route QS, blocked |
|---|---|---|---|---|
| 0% | 100%, 121 s | 100%, 172 s | 100%, 96 s | 100%, 108 s |
| 10% | 100%, 120 s | 100%, 172 s | 100%, 96 s | 100%, 107 s |
| 30% | 100%, 119 s | 100%, 170 s | 100%, 94 s | 100%, 105.5 s |
| 50% | 100%, 105.5 s | 100%, 167.5 s | 100%, 89 s | 100%, 101.5 s |

With the backhaul cut at 250 s, the centralised method warned the road **0%** of the time in the surge. In the blocked scenario it warned 100% (294 s ahead), only because its warning came *before* the cut.

**E4: false alerts in normal flow:** 0 CRITICAL/h and 0 WATCH/h for every method (peak density 1.38).

**E3: closed loop, JuPedSim, 5 seeds** (`qs_alert.py --experiment e3 --e3-seeds 1 2 3 4 5`; `e3_surge_summary.csv`).
- **The control:** each method controls inflow at the road head (WATCH → HOLD 0.5/s, CRITICAL → CLOSE).
- **What a seed varies:** arrivals, walking speeds and the radio-loss stream.
- **Format:** values are mean ± sd over the 5 seeds.

| Variant (surge) | Peak (p/m²) | s ≥ 4 | s ≥ 5 | Seeds never ≥ 4 | Diverted |
|---|---|---|---|---|---|
| No system | 6.01 ± 0.04 | 831 ± 6 | 746 | 0 / 5 | 0 |
| Threshold (today) | 6.01 ± 0.04 | 831 ± 6 | 746 | 0 / 5 | 0 |
| Centralised, backhaul cut at 250 s | 6.01 ± 0.04 | 831 ± 6 | 746 | 0 / 5 | 0 |
| Gradient QS (v2), 30% link loss | 4.14 ± 0.25 | 327 ± 217 (0–487) | 0 | 1 / 5 | 1,953 |
| **Route-graph QS (v3), 30% link loss, no backhaul** | **3.34 ± 0.22** | **0** | **0** | **5 / 5** | 2,002 |
| Centralised, backhaul up | 2.93 ± 0.34 | 0 | 0 | 5 / 5 | 1,911 |
| Hybrid ICCC + route QS, backhaul up (§2.9) | 2.93 ± 0.34 | 0 | 0 | 5 / 5 | 1,911 |
| Hybrid, backhaul cut mid-surge at 520 s (§2.9) | 3.13 ± 0.39 | 0 | 0 | 5 / 5 | 1,965 |
| Route QS, gate board dies, **no** failure handling (§2.8; 3 seeds) | 6.07 ± 0.04 | 767 (749–778) | 684 | 0 / 3 | 259 |
| Route QS, gate board dies, failure handling (3 seeds) | 3.28 ± 0.26 | **0** | 0 | 3 / 3 | 2,122 |
| Route QS, holding-area board dies, **no** failure handling (3 seeds) | 6.06 ± 0.02 | 767 (749–778) | 686 | 0 / 3 | 261 |
| Route QS, holding-area board dies, failure handling (3 seeds) | 3.30 ± 0.21 | **0** | 0 | 3 / 3 | 2,196 |

- **Today's threshold practice behaves exactly like no system.** The road nodes never alarm in time. The runner checks this exactly: the controller is replayed on the uncontrolled run, and if it never leaves OPEN, the closed loop *is* the uncontrolled run.
- **Route-graph QS kept the crowd under 4 p/m² in every seed, without any backhaul.** The peak is 0.4 p/m² above the centralised command centre with its network up. That centralised case is an oracle (all data, no radio loss) and is not available when the network fails.
- **Gradient QS (v2) is seed-sensitive.** It spends 0 to 487 s above 4. The single-seed number reported earlier (567 s) came from an older code version.
- **Limitation:** 5 seeds of one surge scenario. The spread is small, but this is not a confidence interval over scenarios. E8 covers that (20 scenario draws).

### 2.5 Site-scale model (`sim/site_flow.py`, `sim/site_experiment.py`)
- **Synthetic site:** 10 Nagaras (entry areas, 20,000 m², 40 m wide) feed 10 approach roads (7,500 m², 30 m), which feed 20 junctions (1,000 m², 25 m; each Nagara road splits 50/50 to two junctions). These feed 68 compartments: 8 junctions × 4 and 12 junctions × 3, at equal shares. Each compartment has a gate (5 m), the compartment itself (600 m², 20 m) and an exit (3 m), 100 m of river bank apiece.
- **Flow model:** cell-transmission style.
  - sending_i = min(N_i/T_i, w_i·1.22) × control_i;
  - receiving_j = w_j·1.22·clip((5.4 − ρ_j)/(5.4 − 1.75), 0, 1);
  - proportional scaling when receivers are full (back-pressure);
  - compartments add a 300 s bathing dwell to T_i.
- **Control multipliers:** open 1.0, METER 0.5, HOLD 0.1.
- **Scenarios:**
  - **site-surge:** 6 p/s per Nagara (60 p/s site-wide), with Nagara 3 at 50 p/s from 600 to 1,800 s;
  - **site-blocked:** compartment 30's exit drops to 2% capacity from 300 s.
- **Uncontrolled outcomes:** surge 4.85 p/m², **1,603 s ≥ 4**; blocked 4.48 p/m², **311 s ≥ 4**.
- **Mesh methods** run with 30% radio-link loss.
- **"Needless hold"** = node-seconds of METER/HOLD at nodes that feed none of the places exceeding 3.5 p/m² *in the uncontrolled run*. Successful prevention is not counted as needless. (A first version of the metric did count it as needless and was wrong; it was fixed.)

### 2.6 Site-scale results (E8, `e8_site_scale.csv`)

**Surge:**
| Method | Compartment s ≥ 4 | Any node s ≥ 4 | Peak anywhere | Held node-s | Needless | Exited |
|---|---|---|---|---|---|---|
| None | 1,603 | 1,603 | 4.85 | 0 | – | 201,957 |
| Threshold | 431 | **1,700** | 5.18 | 5,703 | 0% | 199,071 |
| Gradient QS (v2) | 0 | **1,842** | 5.03 | 22,573 | **45%** | **168,652** |
| Centralised (up) | 0 | 0 | 2.29 | 9,240 | 0% | 197,897 |
| Centralised (cut) | 1,603 | 1,603 | 4.85 | 0 | – | 201,957 |
| **Route QS (v3)** | **0** | **0** | 3.39 | 9,389 | **0%** | **200,549** |
| Route QS, no share weighting | 0 | 0 | 3.04 | 6,994 | 0% | 198,290 |
| Route QS **with horizon** | 0 | **585** | 4.20 | 12,917 | **44%** | 203,809 |
| Route QS, HOLD only (no METER) | 0 | 0 | 2.88 | **4,137** | 0% | 200,291 |
| Route QS, no diversion | 0 | 0 | 3.39 | 9,575 | 0% | 199,690 |

**Blocked exit:**
| Method | Compartment s ≥ 4 | Peak anywhere | Held node-s | Needless | Exited |
|---|---|---|---|---|---|
| None / threshold / centralised-cut | 311 | 4.48 | 0 | – | 154,137 |
| Gradient QS | 0 | 3.96 | 5,276 | 36% | 153,364 |
| Centralised (up) | 0 | 3.37 | 5,640 | 0% | 151,929 |
| **Route QS** | **0** | **2.42** | **1,358** | **0%** | **155,083** |
| Route QS, no share weighting | 0 | 2.41 | 3,888 | 0% | 153,238 |
| Route QS, **no diversion** | **60** | 4.26 | 1,383 | 0% | 154,137 |

**E8 over 20 random scenario draws** (`site_experiment.py --seeds 0..19`; `e8_site_scale_summary.csv`).
- **Seed 0** is the nominal scenario above, with a different link-loss stream.
- **Seeds 1–19 draw:**
  - **surge:** the surging Nagara, surge rate 35–60 p/s, start time, duration 15–25 min;
  - **blocked:** the blocked compartment, residual exit capacity 1–10%, block time;
  - **both:** base demand 5–7 p/s per Nagara and dwell 4–6 min.

| Method | Surge: draws safe everywhere (s ≥ 4 = 0) | Surge: mean / worst peak | Blocked: draws safe | Blocked: mean / worst peak |
|---|---|---|---|---|
| None | 6 / 20 | 4.15 / 4.85 | 12 / 20 | 3.41 / 4.49 |
| Threshold | 6 / 20 | 4.33 / 5.18 | 12 / 20 | 3.41 / 4.49 |
| Gradient QS (v2) | **5 / 20** | 4.30 / 5.03 | 20 / 20 | 3.19 / 3.97 |
| Centralised (up) | 20 / 20 | 2.27 / 2.47 | 20 / 20 | 2.83 / 3.37 |
| Centralised (cut at 300 s) | 6 / 20 | 4.15 / 4.85 | 12 / 20 | 3.41 / 4.49 |
| **Route QS (v3)** | **20 / 20** | 3.14 / 3.57 | **20 / 20** | **2.33 / 2.43** |
| Route QS with horizon | 7 / 20 | 3.84 / 4.29 | 20 / 20 | 2.33 / 2.43 |
| Route QS no diversion | 20 / 20 | 3.16 / 3.57 | **14 / 20** | 3.48 / 5.03 |
| Route QS with failure handling (§2.8) | 20 / 20 | 2.98 / 3.57 | 20 / 20 | 2.33 / 2.43 |
| Hybrid ICCC + route QS (§2.9) | 20 / 20 | 2.27 / 2.47 | 20 / 20 | 2.32 / 2.41 |
| Hybrid, backhaul cut at 300 s | 20 / 20 | 3.14 / 3.57 | 20 / 20 | 2.33 / 2.43 |

**Needless holds over 20 draws: a correction of the single-run claim.**
- **The data.** Route QS holds needlessly in **4 of 20 surge draws and 6 of 20 blocked draws**. So does the centralised baseline, in the same draws.
- **Why.** In every one of those draws, the uncontrolled crowd peaked at 2.6–3.4 p/m². That is below the 3.5 p/m² line that defines a hotspot, so there is no hotspot and every hold counts as needless.
- **What it means.** These are near-misses where the rate term predicted a crossing that did not happen. Holding there is arguably prudent, but it is over-alerting and is reported as such.
- **What it does not mean.** In draws with a real hotspot, route QS had 0 needless holds.

**Findings:**
1. **Route-graph QS matches the centralised command centre without any backhaul.** It was safe in 20 of 20 draws of both scenarios.
   - **Surge:** a higher peak than centralised (3.14 vs 2.27) but slightly higher throughput.
   - **Blocked:** a lower peak (2.33 vs 2.83, worst 2.43 vs 3.37) with **4× fewer holds**, because it diverts instead of stopping.
2. **Hop-based QS (v2) over-alerts at site scale:** 45% needless holds, roads pushed above 4 p/m² for 1,842 s, and throughput down 16%. **Route awareness is essential.**
3. **Holding only near the hotspot is harmful.** Small near nodes jam (585 s ≥ 4). Hold where there is buffer space.
4. **Diversion is essential when a single compartment is blocked.** Without it: 60 s ≥ 4.
5. **METER added little.** HOLD-only kept the surge safe with fewer holds (4,137 vs 9,389 node-s). Simpler operations ("slow / hold / divert") are justified.

### 2.7 PS-01 limitations (state these openly)
- **Repeated runs are few.** E3 has 5 seeds of one scenario and E8 has 20 scenario draws. NIST TN 1822-style convergence is shown only informally, through the small spread in E3.
- **Sensing (E1–E11):** inputs are exact simulated densities. Imperfect sensing is tested at site scale in E12 (§2.11) and in the JuPedSim compartment in E16 (§2.15); the sensor errors are modelled, not measured.
- **Site scale uses a macroscopic flow model and an invented layout.** The compartment geometry is assumed.
- **Route shares at junctions are assumed known.** In the field: planned shares, updated from gate in/out counts.
- **Over-alerting in near-miss draws** (above): 4–6 of 20 draws hold where no place would have passed 3.5 p/m².
- **Blocked exit at compartment scale:** QS warns at 107 s against 294 s for centralised. The leak δ smooths a brief early spike in the small exit zone. A per-zone leak is a proposed fix, untested.
- **Field requirements:** narrowed gates for counting; route map drawn per site; power, rain and theft of nodes.

### 2.8 Node (board) failures (E9 SIL, E3 JuPedSim, E10 site; `failure_experiment.py`)

**The weakness, as first found.** The SIL test (E9) killed a board in the middle of a surge. A node that stops hearing its successor drops that successor's level after 5 s, so the road node released its hold **4 s after the gate's board died**, while the compartment was still filling. In JuPedSim (E3, 3 seeds) one dead board, either the gate or the holding area, put the crowd **above 4 p/m² for 767 s on average**. With no system at all the figure is 831 s.

**The fix, built in three steps. Each step was added because the previous one failed in a test:**
1. **Hold until restored.** A successor that goes silent while it is alarming keeps its last level until it is heard again or an operator clears it.
2. **2-hop backup.** Each node also listens to its successors' successors, and uses them only while the successor in between is silent. This bypasses a dead *relay*, such as a gate.
3. **Escalation, "unknown is not safe".** Added after E10 found two failures of steps 1–2:
   - **Problem 1: a board that died just before its own alarm.** It was "silent but calm", so nothing was held. Fix: a successor silent for more than 30 s counts as at least WATCH, and one silent while alarming counts as CRITICAL.
   - **Problem 2: a dead *compartment* bypassed via its exit.** The exit was calm while the crowd inside the compartment kept growing unseen. Fix: a silent successor that is both held and bypassed counts as **the larger** of the two levels.

   Escalation is the firmware default (`failsafeEscalate`).

**JuPedSim, with failure handling (steps 1–2):** gate board or holding-area board dead → **0 s above 4 in 3 of 3 seeds**, for both (E3 table above).

**E10: site scale, closed loop, 5 scenario draws per case.**
- **Where boards die:** the hotspot's own compartment, the gate in front of it, its junction, its approach road, gate + junction together, and 5 / 10 / 20% of all boards at random.
- **When:** at the first alert of a failure-free run. Gate and junction also 2 min before and 5 min after.
- **What a dead board means:** it sends nothing, and its marshals get no signal.
- **Each cell:** the number of draws where no place goes above 4 p/m².

| Surge: boards that die | none (E1–E8 model) | hold | 2-hop backup | hold + backup | **+ escalation (deployed)** | ICCC, network up |
|---|---|---|---|---|---|---|
| hotspot compartment | 0 / 5 | 3 / 5 | 3 / 5 | 3 / 5 | **5 / 5** | 5 / 5 |
| gate (2 min before / at / 5 min after the first alert) | 0 / 1 / 2 of 5 | 3 / 3 / 4 | 5 / 5 / 5 | 5 / 5 / 5 | **5 / 5 / 5** | 5 / 5 / 5 |
| junction (before / at / after) | 0 / 0 / 0 | 0 / 0 / 3 | 5 / 5 / 5 | 5 / 5 / 5 | **5 / 5 / 5** | 5 / 5 / **1** |
| approach road | 0 / 5 | 0 / 5 | 4 / 5 | 4 / 5 | **4 / 5** | **0 / 5** |
| gate + junction together | 0 / 5 | 0 / 5 | 3 / 5 | 3 / 5 | 3 / 5 | 5 / 5 |
| random 5% of boards | 2 / 5 | 2 / 5 | 3 / 5 | 3 / 5 | **5 / 5** | 4 / 5 |
| random 10% | 3 / 5 | 3 / 5 | 4 / 5 | 4 / 5 | 4 / 5 | 4 / 5 |
| random 20% | 1 / 5 | 2 / 5 | 2 / 5 | 2 / 5 | **4 / 5** | 4 / 5 |

**Blocked exit** (4 draws with an alert):
- **The deployed firmware** is safe in **4 of 4** draws in every failure case, including when the blocked compartment's own board dies.
- **Without escalation** that case is 2 of 4.
- **The ICCC with its network up** is also only 2 of 4 when the blocked compartment's board dies.

**Findings:**
- **A single dead board** anywhere on the route to the hotspot is fully covered by the deployed firmware: compartment, gate or junction, at any time, 5 of 5. Only the **approach road** misses one draw (4 of 5): its marshals are the ones who hold, and they are gone.
- **Two adjacent dead boards (gate + junction)** defeat a 2-hop backup in 2 of 5 draws. This is expected: the model gives P(blind) = f² (§2.10 M3). The ICCC handles it, because it sees every board.
- **The ICCC (network up) has its own failure weakness.** It cannot hold where the dead board's marshals stand. When the approach road's board dies it is unsafe in **5 of 5** draws (1,854 s above 4). When the junction's board dies after the first alert it is unsafe in 4 of 5.
- **Cost:**
  - Escalation serves about 1–5% fewer pilgrims when boards die, and up to 5% at 20% random failures, because unknown places are metered.
  - With no failures, the deployed firmware behaves like route QS (E8 "route QS with failure handling": 20 of 20 draws safe, 0 false node-down reports).
- **Staff alerts:**
  - Each dead board is reported once, after 30 s.
  - The first version reported after 5 s. At 51% message loss that flooded the control room: 108 false reports in a run with no failure.
  - A second bug reported boards that had never been heard (at boot). Both are fixed.

### 2.9 Hybrid: ICCC when the network is up, route QS always
**Why.** The ICCC with a working network sees every sensor with no radio loss. It is the best a central system can do, and it beats route QS slightly on surges (peak 2.93 vs 3.34 in E3). But it fails completely when the network is cut: in E3 a command centre cut off at 250 s behaves exactly like no system. Mass gatherings are where mobile networks and backhaul fail.

**What.** Each node takes the **stronger** action of the ICCC command and its own route-QS action. While the backhaul is up that is never weaker than the ICCC; when it is down, route QS alone continues.

| | Surge, JuPedSim (5 seeds) | Surge, site (20 draws) | Blocked exit, site (20 draws) |
|---|---|---|---|
| ICCC, network up | 5 / 5 safe, peak 2.93 | 20 / 20, mean peak 2.27 | 20 / 20, mean peak 2.83 (worst 3.37) |
| ICCC, network cut | **0 / 5**, 831 s ≥ 4 | **6 / 20** | **12 / 20** |
| Route QS alone | 5 / 5, peak 3.34 | 20 / 20, 3.14 | 20 / 20, 2.33 (worst 2.43) |
| **Hybrid, network up** | **5 / 5, peak 2.93** | **20 / 20, 2.27** | **20 / 20, 2.32 (worst 2.41)** |
| **Hybrid, network cut** (E3 at 520 s mid-surge; site at 300 s) | **5 / 5, peak 3.13** | **20 / 20, 3.14** | **20 / 20, 2.33** |

**The result:**
- **With the network up,** hybrid is as good as the ICCC on surges and better on blocked exits, because route QS diverts.
- **With the network down,** hybrid keeps the route-QS result instead of collapsing.
- **The pitch:** the mesh does not compete with the ICCC. It makes the ICCC **resilient**.

### 2.10 Mathematical model, checked against the simulations (E11, `sim/analytic_model.py`)
Every prediction is listed against the matching measurement in `e11_analytic_vs_sim.csv`. Where the predictions are fitted to anything, the fit uses different runs from the ones they are tested on.

**M1: one-equation model of the compartment, closed loop.**
- **The equation.**
  - dN/dt = I(t) − Q(N), with ρ_C2 = min(N / A_eff, ρ_jam);
  - Q(N) = Q₀ / (1 + (N/N₅₀)^k): outflow that **falls** as the compartment fills, because bathers going in and coming out block each other.
- **The fit.** Fitted on JuPedSim seeds 1–2 (uncontrolled): Q₀ = 1.25 p/s, N₅₀ = 375, k = 1.75, A_eff = 334 m², with an outflow RMSE of 0.17 p/s.
- **The closed loop.** The road command comes from the real alert rules (`qs_core`):
  - route QS: leaky level, relayed over 2 hops with an expected wait for lost messages;
  - ICCC: no leak, no hops.

| Quantity | Model | JuPedSim, seeds 3–5 (not used for fitting) |
|---|---|---|
| Time density first reaches 4 p/m², no control | 670 s | 672 s (665–679) |
| Seconds above 4, no control | 830 | 828 (821–835) |
| Peak density, no control | 6.05 | 5.99 |
| Peak density, route QS (30% link loss) | 3.24 | 3.28 (2.95–3.46) |
| Seconds above 4, route QS | 0 | 0 |
| First road hold, route QS | 499 s | 513 s (503–523) |
| Peak density, ICCC (network up) | 3.06 | 2.91 (2.65–3.39) |
| First road hold, ICCC | 488 s | 498 s (486–513) |

- **What the model explains.**
  - **The tipping point.** The compartment's largest steady throughput is max Q = **1.25 p/s**. A normal day (1.2 p/s) sits just under it and stays steady. A surge (4.5 p/s) pushes N past the point where Q starts to fall, and from there the jam feeds itself, because outflow drops as inflow keeps coming.
  - **Why early holds matter.** The alert has to cut inflow **before** N reaches the falling part of Q(N), not when density reaches 4.
  - **Why route QS is about 0.3 p/m² above the ICCC.** Route QS holds 11 s later (the leak plus 2 relay hops). In the fluid model that alone accounts for the difference.

**M2: radio loss.**
- **Effective loss.** With per-direction link loss p, a message gets through only if both directions are up, so q = 1 − (1 − p)². At p = 0.3 that gives q = 51%.
- **Extra delay.** Each hop is a geometric number of tries at 1 message per second, so the extra delay over h hops is E[Δ] = h·q/(1 − q).
- **Stale links.** A link is treated as stale (> 5 s silent) with probability q⁶.

| | Model | Measured (300 Monte-Carlo seeds, compartment surge, 3 hops) |
|---|---|---|
| Extra road-warning delay at q = 19% | 0.70 s | 0.82 s (p95 3 s) |
| at q = 51% | 3.12 s | 3.15 s (p95 8 s) |
| at q = 75% | 9.0 s | 9.74 s (p95 24 s) |
| P(link stale) at q = 51% | 1.8% | (explains the false node-down reports of the first SIL version) |
| P(30 s silence) at q = 51% | ≈ 2×10⁻⁹ | 0 false node-down reports with the 30-s rule |

**M3: reliability when boards die.** For the chain compartment → gate → junction → road, with a fraction f of boards dead at random:
- **No protection:** P(road blind) = 1 − (1 − f)², because the road needs both the junction and the gate.
- **With the 2-hop backup:** P(road blind) = f², because both must die.

| Boards dead | No protection: model / Monte Carlo | 2-hop backup: model / Monte Carlo |
|---|---|---|
| 5% | 9.75% / 9.61% | 0.25% / 0.24% |
| 10% | 19.0% / 18.97% | **1.0% / 0.99%** |
| 20% | 36.0% / 36.07% | 4.0% / 3.90% |

(Monte Carlo: 2,000 draws on the 68-compartment route graph.)

**M4: alarm stability.**
- **Upper bound.** A state can only drop after MIN_DWELL = 5 steps, so each node changes state at most once per 6 s (≤ 600 per hour).
- **Measured:** worst node in SIL 16.8 per hour; road command in the E3 closed loop 38.9 per hour on average (max 52.8). Mostly WATCH ↔ CRITICAL while the hold rate meters the surge.
- **Site scale (E13, light seen by marshals, per lit node):** about 20 changes per hour (worst node 73–86) with the deployed firmware; about 13 with amber-only slow release (§2.12).
- **False alerts:** on a normal day 0, as predicted (E4). ρ stays below 2.4, so s = ρ/4 < 0.6, and the rate term is off below the 1.5 p/m² floor.

**M5: radio capacity.**
- **ESP-NOW.** A 25-byte broadcast occupies about 720 µs at 1 Mbit/s. At 1 message per second that loads the channel by 0.7% with 10 boards in range, 3.6% with 50 and 7.2% with 100.
- **LoRa SF7 / 125 kHz.** One message takes 61.7 ms (Semtech AN1200.13). Under ALOHA (≤ 18% usable) one channel carries **only about 2 boards** sending every second, or about 29 sending every 10 s or on a state change. Design consequence: ESP-NOW for neighbours, LoRa only for long hops, sending on change.
- **Measured traffic:** 24.4 bytes per board per second in SIL (model 25; the loop runs slightly slower than 1 s).

### 2.11 Realistic sensing (E12, `sim/sensing.py`, `sensing_experiment.py`, `sensing_failures.py`)

**Why.** E1–E11 gave every node the true density. Reviewers rightly asked: (1) occupancy computed as in − out drifts without bound; (2) line counters need lanes, and approach roads are 8–40 m wide; (3) the VL53L1X loses most of its range in direct sun (datasheet: 73 cm in long mode at 200 kcps/SPAD); (4) a density feed that arrives through the ICCC is lost when the network fails.

**Sensing model** (site scale, per node, every second):
- **Line counters** on a boundary: counted = true crossings × (1 + β) + N(0, 0.3·√crossings); β is a fixed per-counter bias, β ~ N(0, 2 % or 5 %). A counter on edge i → j is "out" for i and "in" for j. Placement: *all* boundaries (the idealised layout), or *narrow* ones only (≤ 6 m: junction → gate, gate → compartment, compartment → exit, exit outflow), since wide roads cannot be line-counted without lanes.
- **Density reading** (edge camera wired to the node, existing CCTV, or RF link attenuation): every T s (staggered per node), z = ρ·(1 + γᵢ)·(1 + ε) + N(0, 0.05); γᵢ ~ N(μ, σ_γ) fixed per node, ε ~ N(0, σ_rel); missing with probability p_drop.
- **Estimator:** a 2-state Kalman filter per node, x = [ρ, b], b the drift of the counted net flux. Predict ρ ← ρ + (in − out)/A − b; correct with z when a reading arrives (this removes accumulated drift and learns b). Nodes without full counter coverage run a random walk corrected by z; nodes with neither sensor are blind and report 0. Scalar operations only, so it ports to the firmware header.
- **Random-walk step (changed 12 Nov after E16, §2.15):** density at an uncounted node may change by at most 1.22 × width / area per second (a full-capacity flow through its boundary). The first E12 runs used a fixed 0.01 p/m²/s, which lags small, fast-filling zones by about 2 minutes. All E12–E15 results in this report were rerun with the physics-based step; at site scale the numbers barely moved (peaks within 0.12 p/m², identical safe/unsafe counts in E12b and E12d).
- **Checks:** with "truth" sensing E12 reproduces E8 exactly (route QS and hybrid, both scenarios). Unit tests (`test_sensing.py`): dead reckoning drifts exactly as β·q·T/A predicts (1.8 p/m² after 1 h at 5 % on 6 p/s into 600 m²); the filter keeps the error ≤ 0.35 p/m², beats the camera alone, and learns b on average.

**E12: sensing set-ups, 20 draws × 2 scenarios** (route QS = the mesh alone, 30 % link loss; hybrid = ICCC + mesh). Surge:

| Sensing | Route QS safe | Worst peak | Draws with needless holds | Hybrid safe | Est. RMSE (compartments) |
|---|---|---|---|---|---|
| Perfect (E8) | 20/20 | 3.57 | 4 | 20/20 | 0 |
| Counters everywhere, 2 % bias, no density reading | **5/20** | 5.07 | 20 | 20/20 (needless holds 74 %, throughput −16 %) | 0.083 (max error where dense 5.5) |
| Counters everywhere, 5 % bias | **2/20** | 5.03 | 20 | 20/20 (needless 90 %, throughput −34 %) | 0.164 (max 9.1) |
| Counters (5 %) + camera every 60 s, fused | 20/20 | 3.57 | 6 | 20/20 | 0.055 |
| **Counters at gates/exits (2 %) + camera every 10 s, fused** | **20/20** | **3.57** | 5 | 20/20 | 0.039 |
| Same, 5 % counter bias | 20/20 | 3.57 | 5 | 20/20 | 0.039 |
| Same, RF attenuation instead of camera (σ_rel 25 %, σ_γ 10 %, 20 % lost) | 20/20 | **4.00** | 7 | 20/20 | 0.119 |
| Cameras only, every 10 s | 20/20 | 3.53 | 9 | 20/20 | 0.065 |
| Counters at gates/exits only, wide areas unsensed | **4/20** | 5.07 | 20 | 20/20 | 0.083 |

Blocked exit: route QS is safe in 20/20 with every set-up, but counters without a density reading make 90–96 % of holds needless.

**E12d: 100 draws** (route QS): perfect sensing 100/100 surge (worst 3.57) and 100/100 blocked (2.44); the realistic layout (counters at gates/exits + camera every 10 s) **100/100 surge (worst 3.57) and 100/100 blocked (2.86)**. 100/100 gives a two-sided 95 % Clopper–Pearson lower bound of 96.4 % (20/20: 83.2 %). Rerun 12 Nov with the physics-based random walk: identical safe counts and worst peaks; needless holds in 27/100 surge and 51/100 blocked draws.

**E12b: how good the density reading must be** (realistic layout, surge, route QS, 20 draws; one property changed at a time):

| Density reading | Safe | Worst peak |
|---|---|---|
| every 10 / 30 / 60 / 120 s | 20/20 each | 3.57 |
| every 300 s | 20/20 | 3.71 |
| 20 % / 50 % of readings lost | 20/20 | 3.57 |
| per-camera bias σ_γ 10 % / 20 % | 20/20 | 3.57 |
| all cameras undercount 10 % / 20 % | 20/20 | 3.71 / 3.96 |
| noise σ_rel 20 % | 20/20 | 3.57 |
| **noise σ_rel 30 %** | **18/20** (up to 624 s above 4) | **4.42** |
| **RF at 30 s / 60 s** (σ_rel 25 %, σ_γ 10 %, 20 % lost) | **19/20** | 4.11 / 4.11 |

**Requirement for the hardware:** a density reading at least every 120 s with ≤ 20 % error per reading. Losing readings matters far less than noisy ones. RF attenuation is a fallback only, and only at 10 s.

**E12 rerun note (12 Nov, physics-based random walk):** every row of the table above is unchanged except cameras-only (needless holds 6 → 9 draws, RMSE 0.051 → 0.065): with no counters anywhere, the filter now follows each 10 % camera reading more closely on compartments, so noise triggers a few more precautionary holds. The legacy fixed-walk run reproduces the old realistic-layout row exactly.

**E12c: failures under the realistic layout** (deployed firmware: hold + 2-hop backup + escalation; 5 surge and 4–5 blocked draws; at the first alert; rerun 12 Nov with the physics-based walk: identical safe counts, random 10 % peak 4.3 → 4.2):
- **Boards die** (as E10): compartment, gate, junction, road 5/5 each (road was 4/5 in E10); gate + junction 3/5 (the known gap, covered by the ICCC); 5 / 10 / 20 % at random 5/5, 4/5, 4/5. Blocked exit 4/4 in every case. Imperfect sensing does not weaken failure handling.
- **A camera fails while its board runs on**, two firmware behaviours: *naive* (dead-reckon on counters where present, else blind = 0) and *flag* (report max(last estimate, WATCH level)). Safe in **every** case (compartment, gate, junction, road; 40 runs). Flagging holds more (road peak up to 3.95); recommended rule: keep counting where counters exist, report at least WATCH only where they do not.

### 2.12 Firmware v4 options (E13, `sim/v4_experiment.py`)

Options in `qs_core.RouteParams` (all off by default; E1–E12 reproduce): **slow release** (a light may only drop after 120 s unchanged), **amber-only slow release** (only amber → green waits 120 s; red → amber stays at 5 s), **dynamic shares** (relayed threats follow a junction's current, diverted shares, smoothed with τ = 30 s), **dead-node release** (a silent successor held at CRITICAL drops to WATCH after 180 s if every 2-hop backup behind it is heard and calm, level < 0.375). Part A: 20 draws × 2 scenarios × perfect/realistic sensing. Part B: E12c board failures, realistic sensing, 5 draws.

| Option | Surge (A) | Blocked (A) | Board failures (B, surge) | Light changes / lit node / h |
|---|---|---|---|---|
| Deployed | 20/20, worst 3.57 | 20/20 | as E12c | ~20 |
| **Slow release (all drops)** | **14–15/20**, worst 4.85 | 20/20 | worse in every case (gate 2/5, junction 2/5, gate+junction 0/5) | ~10 |
| Amber-only slow release ("v4b" with the two below) | 20/20, worst 3.57 | 20/20 | same as deployed except dead road board 4/5 (deployed 5/5) | **~13 (−35 %)** |
| Dynamic shares | identical | identical | – | ~20 |
| Dead-node release | identical | identical | identical | ~20 |

- **Slow release is unsafe.** In every failing draw the compartments stayed below 2.5 p/m²; the crush was upstream, where a red hold lasting ≥ 120 s piled held pilgrims above 4 p/m² (the "hold where there is buffer" lesson of §2.6, from the other side). Red → amber must stay fast so held crowds can be metered.
- **Rerun 12 Nov** with the physics-based random walk: identical safe counts in every case of Parts A and B; worst peaks moved by at most 0.14 p/m².
- **Amber-only slow release** keeps the lights calmer with no loss in normal operation, but costs one draw when the approach road's board dies. It is offered as a pilot-tunable option, not the default.
- **Dynamic shares and dead-node release had no effect** here: the share-weighted sum already tracks diversion closely, and in a surge the nodes behind a dead board are never calm, so the release correctly never fires. After random failures, the needless holds (74–95 %) come from the WATCH floor of "unknown is not safe", which by design is never released to green without the board or an operator.
- **Correction:** the stored E8 "route-qs fail-safe" row predated later failure-handling fixes. E8 was regenerated on 11 Nov with the current code: only that row changed (still 20/20 safe in both scenarios), every other row reproduced exactly.

### 2.13 Police reaction delay and partial compliance (E14, `sim/field_experiment.py`)

**Why.** Every earlier experiment assumed the crowd obeys a light the second it changes. E14 delays the response by D seconds and lets only a fraction c of it take effect: holds and meters reach c of their effect, and diversion moves c of the planned share change. Realistic sensing (counters at gates/exits + camera every 10 s), deployed firmware (hold + 2-hop backup + escalation), route QS alone with 30 % link loss, 20 draws per cell.

Surge, draws safe of 20 (worst peak in brackets); the blocked exit was safe in 20/20 in every cell (worst 3.58):

| Delay D | c = 100 % | c = 70 % | c = 50 % |
|---|---|---|---|
| 0 s | 20 (3.57) | 20 (3.70) | 18 (4.04) |
| 30 s | 20 (3.55) | 20 (3.99) | 20 (3.98) |
| 60 s | 20 (3.58) | 20 (4.00) | 19 (4.03) |
| 120 s | 20 (3.81) | **14** (4.05) | 18 (4.06) |
| 180 s | 20 (3.96) | **9** (4.05) | 18 (4.13) |

- **Full compliance tolerates even a 3-minute delay.** The danger is late action *and* partial compliance together: with 70 % compliance, 60 s is safe in 20/20 but 120 s and 180 s keep only 14 and 9 of 20 safe (up to 375 s above 4 p/m²).
- **Not monotone in compliance:** 50 % compliance does better than 70 % at long delays (18/20). Our reading, untested: weaker holds build smaller held crowds upstream, which then release less violently; at 50 % the crowd is also never fully stopped where it is dense.
- **Field rule:** drills train the road-head police to act within a minute of their light changing, and the pilot measures the actual delay and compliance.

### 2.14 Alert-threshold sensitivity (E15, `sim/field_experiment.py`)

One parameter changed at a time around the defaults, 20 draws per setting, same set-up as E14 (no delay, full compliance):

| Setting | Surge safe | Surge worst peak | Draws with needless holds (surge / blocked) | Held node-s (surge) |
|---|---|---|---|---|
| default (θ_watch 0.6, θ_crit 0.85, δ 0.85, horizon 60 s, floor 1.5) | 20/20 | 3.57 | 5 / 7 | 7,578 |
| θ_watch 0.5 / 0.7 | 20/20 / 20/20 | 3.56 / 3.97 | 6 / 8 · 4 / 0 | 10,274 / 6,659 |
| θ_crit 0.75 / 0.95 | 20/20 | 3.57 | 5 / 7 | ~7,530 |
| δ 0.75 / 0.92 | 20/20 | 3.57 | 5 / 7 | 7,768 / 7,372 |
| rate horizon 30 / 90 s | 20/20 | 3.57 | 5 / 7 | 7,233 / 7,967 |
| rate floor 1.0 / 2.0 | 20/20 | 3.57 | 5 / 7 | ~7,600 |

- **Robust:** every setting kept every draw safe in both scenarios (blocked worst 2.84–2.88).
- **θ_watch is the one real dial:** 0.5 holds 36 % more node-seconds; 0.7 removes every needless hold in the blocked exit but lets the surge peak reach 3.97, close to the limit. The default 0.6 sits between them; the pilot data should set it.
- The other parameters barely matter at these settings.

### 2.15 Realistic sensing in the JuPedSim compartment (E16, `sim/e3_sensing.py`)

**Why.** E12 tested sensing only on the macroscopic site model. E16 runs the closed-loop JuPedSim compartment of E3 (variant 5: route QS, 30 % link loss, no backhaul; surge and blocked exit, seeds 1–5) through the same sensing layer and the same estimator (`sensing.kf_step`). In/out crossings per zone come from agent zone membership each second (`ghat_scenario.run`).

**Set-ups.** Perfect sensing; line counters on every zone (5 % bias, no density reading: dead reckoning); the realistic layout (only the 2 m exit corridor X is narrow enough to line-count, so a camera reads every zone every 10 s, 10 % noise, 5 % per-camera bias); the same with RF attenuation (25 % noise, 10 % bias, 20 % of readings lost).

**Check.** Perfect sensing reproduces plain E3 rerun on the same machine exactly (seeds 1 and 4: peak 3.4133 and 3.4367). The stored E3 numbers (run on the Windows build) differ slightly per seed (all safe in both): JuPedSim runs are not bit-identical across platforms.

| Sensing | Random walk | Surge: safe seeds | Surge: worst peak | Surge: mean s above 4 | Blocked: seeds never above 4 | Estimate RMSE (surge) |
|---|---|---|---|---|---|---|
| Perfect | – | 5/5 | 3.44 | 0 | 3/5 (max 2 s above 4) | 0 |
| Counters on every zone, 5 % | – | 5/5 | 3.87 | 0 | **0/5 (up to 441 s)** | 0.45 |
| Realistic layout, camera 10 s | fixed 0.01 | **0/5** | **4.77** | **820** | 3/5 (max 1 s) | 0.23 |
| **Realistic layout, camera 10 s** | **physics** | **5/5** | **3.37** | **0** | 3/5 (max 1 s) | **0.08** |
| RF instead of camera | fixed 0.01 | 0/5 | 6.08 | 828 | 3/5 | 0.99 |
| RF instead of camera | physics | 2/5 | 4.44 | 284 | 3/5 | 0.26 |

- **The estimator was the problem, not the alert method.** With the fixed random walk (0.01 p/m²/s) the filter trusted each 10 % camera reading too little and trailed a fast-filling zone by about 2 minutes. Replaying the recorded E3 surge offline, WATCH was recognised 105 s late on compartment C2. Site-scale zones are 600–20,000 m² and fill slowly, so E12 never showed it; the 64 m² gate and 300 m² compartments here fill 3–10 times faster.
- **Fix:** the random-walk step follows the physics, 1.22 × boundary width / area per second (the fastest a full-capacity flow can change the density). Offline replay: WATCH 5 s late. Closed loop: safe in 5/5, estimate RMSE 0.23 → 0.08. It is now the default in `sensing.py` and in the firmware (`PATCH_WIDTH_M`); the old step is kept as the "fixed-rw" set-ups.
- **Dead reckoning at compartment scale:** counters alone stay safe in the surge but miss the blocked exit in every seed (up to 441 s above 4): drift accumulates while the zone fills slowly. Counters need a density reading.
- **RF attenuation** is not enough here even with the fix (2/5): it stays a fallback only.
- **Still small:** 5 seeds per scenario at compartment scale.

---

## 3. PS-03: Lost-person identification and reunification

### 3.1 Constraints that shaped the design
- **No facial recognition** (challenge rules). Privacy-preserving, compatible with the DPDP Act. ⚠ §9 restricts tracking of children (Gazette check pending).
- **Rural families often have no smartphone, and not every member has a phone.** A child may be with another relative, or with a stranger.
- **Bands only for children and senior citizens.** Distributing tokens to every family member is not viable.
- **Indian precedents:**
  - Sabarimala (Kerala Police, under-10 bands with a guardian number and QR code; Vi Suraksha);
  - Gangasagar "e-Parichay" (about 1–2 lakh QR bands for children and the elderly);
  - Maha Kumbh 2025 digital Khoya-Paya kendras (about 50–54k reunited, but they used facial recognition);
  - Aarogya Setu IVRS 1921 (missed-call call-back for feature phones).

### 3.2 Design evolution
| Version | Idea | Why it changed |
|---|---|---|
| v1 | Child tag + guardian device; RSSI state machine NEAR → DRIFTING → SEPARATED → LOST | Assumes a guardian device; fails for rural families, a single phone, or a child with a relative |
| v2 | Phone-free: band + registry, description matching, IVR | The user also ruled out a family band or card |
| v3 | "Co-walker" inference: the tag learns which nearby devices stay with it | Works only if a family member carries an advertising device; families without one are invisible to any RF method |
| **v4 (final)** | Bands for children and seniors only; node sighting buffers + LoRa search after a report; **converge-at-desk**; exit-gate watch; optional co-walker auto-flag | – |

### 3.3 Final design (v4)
- **Band:**
  - **ID:** a rotating 8-byte ID = HMAC-SHA256(per-band key, time slot). The MAC address rotates at the same instant, so strangers can't track the band. Each band has its **own key**; a test caught two bands sharing a demo key, and this was fixed.
  - **Beacons:** every 300 ms; every 100 ms when LOST or flagged.
  - **Rotation period:** 60 s in the demo; 10–15 min in deployment, GAEN practice.
  - **Printed:** QR code + helpline + case code; no phone number in plain view.
- **Registration:** at issuance, record any reachable contact (feature phone, relative, group leader, village contact) and tell the adult which help desk to go to if separated.
- **Nodes:** the same nodes as PS-01. Each keeps a **60-minute local buffer** of opaque band IDs (time, RSSI). Nodes can't link IDs without the key. Nothing is uploaded centrally.
- **Report** (help desk / IVR missed call / volunteer handset):
  - the desk computes the band's IDs for the last 60 min and broadcasts a search over the node network (LoRa in deployment);
  - nodes reply with matches → **last-seen zone + trail**;
  - a command beacon switches the band to LOST.
- **Converge-at-desk:** the found person is taken to the desk nearest the **separation point**, the same desk the family was told to go to.
- **Exit-gate watch:** a LOST band seen at an exit node raises an alert there. This covers a child led away by a stranger.
- **Optional auto-flag (families with a learnable device):**
  - the band learns its co-walkers;
  - if they vanish for 2 min, it flags "possible separation";
  - **two-stage escalation:**
    - **stage 1:** silent watch;
    - **stage 2:** after T_confirm with no recovery, the station nearest the child dispatches a volunteer. Nodes look for the family's devices across the region, and the station nearest the parents tells them which desk to go to.
- **Physical limit:** **LoRa cannot sense people.** A parent with no device can't be detected by any RF method. For them the system relies on converge-at-desk, the registry and the trail.

### 3.4 PS-03 simulation results

**(a) Separation detection with a guardian device (v1 model, `separation_sim.py`; 20 family-hours per density):**
- **Radio model:**
  - P0 = −59 dBm at 1 m, path-loss exponent n = 2.0;
  - body shadowing: 10 dB for the first body in the way, +5 dB per extra body (Januszkiewicz 2018);
  - 3% deep 30 dB fades (Cotton 2011);
  - σ = 6 dB;
  - collisions up to 30% (Nikodem 2019);
  - scan duty 50%.

| Density | Naive RSSI threshold: false alarms/h | Ours: false buzz / false LOST per h | Ours: buzz delay | Ours: LOST delay |
|---|---|---|---|---|
| 0.5 | 1.6 | 0 / 0 | 43 s | 59 s |
| 2.0 | 76.0 | 0 / 0 | 18 s | 26.5 s |
| 4.0 | 143.9 | 0 / 0 | 13.5 s | 23.5 s |
| 6.0 | 145.8 | 0 / 0 | 11 s | 29.5 s |

The "0 false alarms" figure is probably optimistic, because the child trajectory model rarely wanders 5–8 m. The sighting-coverage experiment (node spacing × volunteer phones) has **not yet been run**.

**(b) Co-walker inference (`companion_sim.py`; 20 runs per configuration; tag-side learning):**
- **Model:**
  - phones rotate their BLE address every 15 min (Android CDD; Bluetooth SIG default);
  - the tag scans 1 s in every 5 s;
  - imprinting at issuance;
  - re-link across rotation by payload class + RSSI within 6 dB;
  - a companion arms only after staying ≥ 10 min.

| Density | Companion precision | Recall | Separation auto-detected | False flags / device-family-hour |
|---|---|---|---|---|
| 0.5 | 0.63–0.65 | 0.81–0.83 | 95–100% | 0.89–1.16 |
| 2.0 | 0.59–0.65 | 0.62–0.73 | 53–89% | 1.14–1.30 |
| 4.0 | 0.53–0.57 | **0.23–0.33** | **12–26%** | 0.55–2.47 |

- **Families with no device get 0 false flags.**
- **The fixed-node co-occurrence alternative is worse:** precision 0.007–0.31 at low and medium density, and false flags up to 4/h (up to 7.4/h for no-device families).

**(c) Two-stage escalation (`companion_escalation.csv`):**
| Confirm delay | Volunteer call-outs per 1,000 bands/h (ρ 0.5 / 2 / 4) | Family phone locatable at escalation (ρ 0.5 / 2 / 4) |
|---|---|---|
| 0 s | 455–804 / 714–1,111 / 235–2,253 | 39–44% / 25–33% / 0% |
| 180 s | 341–697 / 663–918 / 235–901 | 22–43% / 0–24% / 0% |
| 300 s | **0–54** / 364–532 / 241–676 | 22–36% / 0–24% / 0% |

**The false call-out load is still too high at medium and high density, even with a 5-min confirmation.** Auto-flag is therefore an **experimental extension** only.

**(d) Reunion time (`reunion_sim.py`; 3,000 episodes per strategy).**
- **Site model:** a 12 × 5 zone grid of 40 m zones, 6 help desks, node detection 0.9 per zone visit.
- **Behavioural assumptions** (not measured; stated openly):
  - the parent notices after U(2, 10) min;
  - searches nearby for U(10, 30) min;
  - the child drifts to a neighbouring zone with probability 0.3/min;
  - bystanders find a lost child at 1/20 min;
  - alerted volunteers find one at 1/4 min (1/3 min with an auto-flag dispatch);
  - families follow the desk advice with probability 0.7.

| Strategy | Median | 90th percentile | Within 30 min | Within 60 min |
|---|---|---|---|---|
| A. PA announcements + manual registers (today) | 36.7 min | 74.0 | 28.1% | 83.6% |
| B. QR band + registry | 34.7 | 52.7 | 33.4% | 92.2% |
| C. BLE band + last-seen trail | 33.0 | 43.7 | 36.1% | 98.7% |
| **D. C + converge-at-desk** | **17.0** | **37.7** | **78.6%** | 98.9% |
| E. D + co-walker family zone (family has phone) | 17.0 | 38.0 | 78.4% | 98.8% |
| F. D + auto-flag + nearest-station action (family has phone) | 14.0 | 35.0 | 81.8% | 99.6% |

- **Sensitivity:**
  - **D's median is 14–29 min** across family compliance from 0.3 to 0.9 and manual-link probability from 0.3 to 0.8, against **36.7–38 min for A**. D wins under every combination tested.
  - **F's median is 13–15 min** across confirmation delays of 0/180/300 s and probabilities of reaching the parents of 0.3–0.9.
  - **F, population-wide:** 14–16 min across learnable-device shares of 0.3–0.9.
- **F's gain over D comes from dispatching a volunteer to the child.** Parent location contributed little: at density 2 with a 3-min confirmation, the family phone was locatable 0% of the time at escalation.

### 3.5 PS-03 limitations
- **The reunion results depend on assumed behaviour.** No field data was used. This is a model, not evidence.
- **The radio model is from the literature, not from measurements.** It is **unverified that ordinary Android phones advertise over BLE at all** (iPhones and wearables do); co-walker detection depends on this. It should be measured at the pilot.
- **Co-walker detection is weak in dense crowds**, and the false call-out load is high.
- **DPDP §9(3)** restricts tracking of children. The design relies on consent at issuance, opaque IDs, local buffers expiring after 60 min, and search only after a report. The exact Fourth Schedule wording is pending.

---

## 4. Prototype

### 4.1 Firmware (`firmware/`, PlatformIO, ESP32 Arduino 2.0.x)
All three roles compile:
- **gateway** (node 4, zone C1, USB bridge to the backend);
- **guardian** (node 2, zone R2; a demo stand-in for a family phone);
- **tag**.

| Role | Function |
|---|---|
| **Node** | 1 s loop: BLE scan; **route-graph QS step** (`include/pk_qs.h`, shared with the SIL harness) with node-failure handling; ESP-NOW broadcast of its 25-byte `RouteMsg` to the nodes that feed it. 60-min buffer of tag sightings; answers search requests; family-device watch (hashed addresses, 10-min buffer); relays desk commands (LOST on/off) as a BLE command beacon. JSON lines over serial (state, action, threat level, diversion, nodes reported down). Serial `CLEARDOWN <node>` lets an operator release a fail-safe hold after checking a dead node. |
| **Tag** | Rotating HMAC ID + rotating random MAC. Co-walker learning (scores, imprinting, re-link across rotation). Flags SEPARATED / LOST. Faster beacons when flagged. Carries hashes of two learned family devices when flagged. |

**Density estimator** (`include/pk_sense.h`, port of `sensing.kf_step`): a 2-state Kalman filter per node, fed by line-counter crossings and density readings (serial `SENSE <in> <out> <z>`, or `CountMsg` 'N' relayed by the gateway; real drivers add to the same accumulators). Per-node settings at installation: `PATCH_AREA_M2`, `PATCH_WIDTH_M` (sets the physics-based random walk, §2.15), `HAS_COUNTERS`, `HAS_READING`. A failed density sensor can be flagged (reports at least WATCH). `test_sense_port.py` compiles it with `g++` and feeds it the same inputs as the Python reference: max difference 5.2 × 10⁻⁴ p/m², alert levels identical in 100 % of node-seconds.

**ESP-NOW messages:**
- route-QS threat `Q` (`RouteMsg`, 25 bytes);
- sighting `S`;
- search `F` / hit `H`;
- watch `W` / family `M`;
- command `C`;
- clear-down `D`;
- simulated density `R`;
- raw sensor counts for the estimator `N` (`CountMsg`).

**Build** (`pio run`, 12 Nov, with the density estimator and per-patch width):

| Role | RAM | Flash |
|---|---|---|
| Node (gateway / guardian) | 71.1 KB (21.7%) | 1.53 MB (48.7%) |
| Tag | 40.3 KB (12.3%) | 1.13 MB (36.1%) |

**Node-failure handling (deployment defaults, §2.8):**
- **Hold until restored.** A successor that falls silent (> 5 s) while it was alarming keeps its last level until it is heard again, or until an operator sends `CLEARDOWN`.
- **2-hop backup.** A node also listens to its successors' successors. A backup counts only while the successor in between is silent.
- **Escalation, "unknown is not safe"** (`failsafeEscalate = true`):
  - a silent successor that was alarming counts as CRITICAL;
  - one silent for more than 30 s counts as at least WATCH;
  - one that is both held and bypassed counts as the larger of the two levels.
- **Down report.** Staff are told a node is DOWN only after **30 s** of silence. At 51% message loss a 5-s gap happens on about 2% of link-seconds by chance; a 30-s gap is about 10⁻⁹ (§2.10). The first SIL version reported after 5 s and flooded the control room: 108 false reports in a run with no failure. A node that has never been heard (at boot) is never reported down. That was a second bug, found in the same run.
- **Junction diversion stays safe.** It never diverts into a branch whose board is silent, because its room is unknown. A silent compartment behind a gate reads as "full", not "empty".
- **Backend.** It logs `NODE DOWN … send staff to check it` once per outage, logs `back in service` when the node returns, and offers `POST /api/v1/nodes/{n}/clear-down`.

**E9: software-in-the-loop (SIL) test of the firmware** (`firmware/sil/sil.cpp`, `sim/esp32_sil.py`).
- **How the harness works.** It compiles `pk_qs.h`, the exact code that runs on the board, for the PC (zig c++), and runs one virtual ESP32 per route node over a simulated radio. Messages travel as raw bytes, as `esp_now_send` sends them.
- **Equivalence, lock-step mode.**
  - 1.000 s loops, no loss: firmware and Python agree in **100% of node-seconds** on 4 traces (compartment surge and blocked: 6 boards × 1,500 s; site surge and blocked: 244 boards × 3,600 s). That is about 1.8 M node-seconds, with nothing different.
  - **With failure handling on and boards killed:** gate, compartment, both, a site gate and a site compartment, also **100% identical**, with escalation off and on. Getting there needed one fix in the Python model: how a junction treats a silent branch.
  - **Final re-check, all identical.** After the last change (a held + bypassed successor counts as the larger of the two), the whole suite was re-run on 31 Oct under WSL2 Ubuntu with `g++`. Equivalence: 100% on all 4 traces (about 1.8 M node-seconds). With boards killed: 100% in all 10 cases, escalation off and on. Radio (10 seeds): 0 missed and 0 extra alerts. Node-failure table: unchanged, now with the deployed row.
- **Realistic radio.** Each board has its own boot time, a loop of 1.005–1.06 s with jitter, 2–15 ms latency and 30% message loss; 10 seeds per trace.
  - **Alerts:** the firmware missed no alert the Python model raised, and raised none it didn't.
  - **Timing:** the first METER/HOLD came a median 0 s from Python's, between 2 s earlier and 5 s later.
  - **Agreement and traffic:** action agreement 99.2–100%; 24.4 bytes per board per second.
  - **Needless holds:** in the open-loop site surge, both firmware and Python hold about 5,300–5,500 node-s where no place would pass 3.5 p/m². Four gates feed compartments whose density rose fast to 2.98 and stopped. It is an open-loop artefact of the rate term, and is 0 in the closed loop (E8).
- **Node failure, open loop** (compartment surge, gate or holding-area board killed 60 s after the road was first told to act; 872 s of danger remain):

| Firmware option | Road node acting during the danger |
|---|---|
| none (model as in E1–E8) | **0 s**: the road released 4 s after the kill |
| hold 5 min | ~229 s, then released |
| hold until restored | **872 s (all)** |
| 2-hop backup | **872 s (all)** |
| hold + backup | **872 s (all)** |

(These open-loop SIL runs were made before escalation was added. The closed-loop E10 covers the deployed setting.)

**Gaps:**
- **Not yet flashed** on the boards.
- LoRa is stubbed behind the same messages; ESP-NOW only, since there is no LoRa module.
- The SIL test checks the code and radio timing, not the ESP32 hardware (scan timing, RF, power). That needs the boards (hardware-in-the-loop).

### 4.2 Backend + dashboard (`backend/app.py`, `dashboard/index.html`)
- **Modes:**
  - `playback`: replays a simulated scenario, no hardware;
  - `serial`: reads the gateway over USB;
  - `hil`: pushes simulated zone densities to the boards and reads back their states.
- **PS-01 API:**
  - `/api/v1/zones`;
  - `/api/v1/alerts`;
  - `/api/v1/inflow` (OPEN / HOLD / CLOSE);
  - `/api/v1/nodes/down` and `POST /api/v1/nodes/{n}/clear-down` (dead boards held by the fail-safe; operator release);
  - WebSocket `/ws`.
- **PS-03 API:**
  - `POST /api/v1/bands` (issue with a per-band key);
  - `POST /api/v1/reports` (desk / IVR / volunteer → search + LOST);
  - `POST /api/v1/found` (scan → link, notify, clear);
  - `POST /api/v1/ivr/missed-call`;
  - `/api/v1/cases`;
  - auto-flag two-stage escalation (silent watch → auto case after `--confirm-s`, cancelled if the flag stops);
  - exit-gate watch.
- **Tests** (`backend/test_app.py`, passing):
  - band → report → buffer hits → trail, last-seen zone, designated desk → exit watch → found;
  - per-band IDs don't collide;
  - auto-flag: silent before confirmation, case + actions after it, cancelled when the flag stops;
  - node down: reported once per outage, CLEARDOWN sent to the gateway, "back in service" when it reports again.
- **Dashboard:**
  - live zone map coloured by density and alert state, with the inflow command;
  - case panel showing trail, designated desk, family-zone hint, exit warning, auto-flag actions;
  - report / scan buttons.

### 4.3 Demo plan (`docs/demo_script.md`)
- **Hardware-in-the-loop:** the simulated surge drives the real boards.
- **Stop the backend,** and the road board still turns amber via the mesh.
- **Walk the tag past the nodes,** then report via IVR → trail + designated desk.
- **Move the guardian board away** → auto-flag case.
- **Gate counters are simulated.** VL53L1X sensors are an optional add-on.

---

## 5. Engineering and performance work

### 5.1 Vectorisation (logic preserved; verified against a snapshot of the old code)
| Script | Change | Verification | Speed |
|---|---|---|---|
| `ghat_scenario.py` | Speed setter only on category change; numpy box checks instead of shapely; per-id arrays; jam retries capped at 3 failed placements per second | Normal run **bit-identical**; all 5 E3 variants **identical** to the old results | Main gain in jams |
| `qs_core.py` | Ring-buffer history, vectorised state machine and baselines | **Identical** states on all scenarios and methods | 0.6× at 6 nodes; **1.5× at 30, 2.8× at 100, 3.1× at 300 nodes** |
| `qs_alert.py` | Parallel E2 seeds / E3 variants, slowest first | Results unchanged | E3: 5 variants in about 77 min |
| `companion_sim.py` | Parallel configurations; simpler RSSI | Per-run results **identical** | About 3.5–4× |
| `separation_sim.py` | Vectorised packets, naive detector, coverage | Within Monte-Carlo noise | 68 s → 20 s |
| `reunion_sim.py` | All batches as parallel jobs with spawned seeds | Within Monte-Carlo noise | 25 s for the full run with sweeps |
| `rimea_tests.py` | numpy box checks; pool removed (crashed on Windows) | **Identical** values | about 4 min (was 339 s) |

Two bugs were found during this work and fixed: JuPedSim agent IDs are a process-wide counter, so per-id arrays must use offsets or grow; and the RiMEA pool crashed (WinError 6).

### 5.2 Reproducibility
Every parallel task carries its own seed from `np.random.SeedSequence(seed).spawn(k)`, so results don't depend on the number of workers.

### 5.3 Environment issues (dev laptop)
- Windows Application Control blocks the **pandas** DLL; all scripts use `csv`.
- On 30 Oct, Code Integrity blocked JuPedSim's `py_jupedsim` DLL for a while (event 3077). By 31 Oct it loaded again, and all JuPedSim runs in this report were done locally. `ghat_scenario` still imports JuPedSim lazily, so everything else runs without it.
- The C++ compiler for the SIL harness is `ziglang` from pip (`python -m ziglang c++`). There is no other compiler on the laptop.
- **Smart App Control is on** (`VerifiedAndReputablePolicyState = 1`). It ran the first SIL builds, but at 6:12 AM on 31 Oct it blocked a rebuilt `sil.exe` (Code Integrity event 3077). Every rebuild has a new hash and no reputation. We do not work around the policy. The SIL suite runs unchanged on Linux with `g++`: it was run under WSL2 Ubuntu on the same laptop (Colab or Kaggle also work).
- About 0.7 GB of free RAM: use at most 3 worker processes. E3 runs each simulation as its own process (resumable).

---

## 6. Risks and open items before submission
1. ✅ **Kalman filter in the firmware:** ported (`include/pk_sense.h`); matches `sensing.kf_step` within 5.2 × 10⁻⁴ p/m² with 100 % alert-level agreement (`test_sense_port.py`); E9 rerun 12 Nov, still 100 % identical.
2. **DPDP Rules 2025 Fourth Schedule** wording (Gazette G.S.R. 846(E)).
3. **Kotilingala model-compartment specs:** re-parameterise the geometry when received.
4. **Hardware:** flash and range-test the boards (the route-graph QS and failure handling are in the firmware).
5. ✅ **Sensing at compartment scale:** E16 reran E3 in JuPedSim with in/out counts from agent trajectories (§2.15). It exposed an estimator lag, now fixed. Still only 5 seeds per scenario at compartment scale.
5a. **Sensor errors are modelled, not measured:** bench-test the VL53L1X in sun and shade, and measure camera density error against manual counts in the pilot.
6. **LoRa duty:** at SF7 one channel carries only about 2 long-hop boards sending every second (§2.10 M5). Long hops must send on state change or every 10 s (about 29 boards per channel).
7. **Unverified citations (⚠):** IMO MSC.1/Circ.1533 number; the IIT Madras PRL paper; Legion's owner; Fisheye State Routing and TAG (if cited).
8. **Android BLE advertising prevalence:** a pilot KPI.

---

## 7. Reproduction
```
cd pushkaralu\sim
..\.venv\Scripts\python.exe rimea_tests.py
..\.venv\Scripts\python.exe calibrate.py --reselect
..\.venv\Scripts\python.exe qs_alert.py --experiment e1 e2 e4 e5
..\.venv\Scripts\python.exe qs_alert.py --experiment e3 --e3-seeds 1 2 3 4 5 --workers 3          # E3, JuPedSim
..\.venv\Scripts\python.exe qs_alert.py --experiment e3 --e3-seeds 1 2 3 --e3-variants 8 9 10 11  # dead board
..\.venv\Scripts\python.exe qs_alert.py --experiment e3 --e3-seeds 1 2 3 4 5 --e3-variants 6 7    # hybrid
..\.venv\Scripts\python.exe site_experiment.py --workers 2 --seeds 0 1 2 ... 19    # E8, 20 draws
..\.venv\Scripts\python.exe failure_experiment.py --workers 2                     # E10
..\.venv\Scripts\python.exe esp32_sil.py                                          # E9 (pip install ziglang)
..\.venv\Scripts\python.exe analytic_model.py                                     # E11
..\.venv\Scripts\python.exe test_route_graph.py
..\.venv\Scripts\python.exe separation_sim.py --skip-coverage
..\.venv\Scripts\python.exe companion_sim.py --runs 20
..\.venv\Scripts\python.exe reunion_sim.py --runs 3000
..\.venv\Scripts\python.exe test_sensing.py
..\.venv\Scripts\python.exe sensing_experiment.py --workers 6                      # E12, 20 draws
..\.venv\Scripts\python.exe sensing_experiment.py --workers 6 --tag sweep --methods route-qs --scenarios site-surge --configs narrow-2%+cam10 cam30 cam60 cam120 cam300 cam10-noise20 cam10-noise30 cam10-drop20 cam10-drop50 cam10-under10 cam10-under20 cam10-bias10 cam10-bias20 rf30 rf60   # E12b
..\.venv\Scripts\python.exe sensing_failures.py --workers 6                        # E12c
..\.venv\Scripts\python.exe sensing_experiment.py --workers 6 --tag draws100 --methods route-qs --seeds 0 1 ... 99 --configs truth narrow-2%+cam10   # E12d
..\.venv\Scripts\python.exe v4_experiment.py --workers 6                           # E13
..\.venv\Scripts\python.exe field_experiment.py --workers 6                       # E14 + E15
..\.venv\Scripts\python.exe e3_sensing.py --workers 4                             # E16, JuPedSim
..\.venv\Scripts\python.exe test_sense_port.py                                    # firmware estimator vs Python (g++)
# long runs: start them inside tmux in WSL (they resume from results/*/*.jsonl if stopped)
cd ..\backend
..\.venv\Scripts\python.exe test_app.py
..\.venv\Scripts\python.exe app.py --mode playback --scenario surge
cd ..\firmware
..\.venv\Scripts\pio.exe run
```

## 8. File map
| Path | Content |
|---|---|
| `sim/ghat_scenario.py` | JuPedSim compartment model (closed loop capable) |
| `sim/rimea_tests.py`, `sim/calibrate.py` | Verification and calibration |
| `sim/qs_core.py` | Threshold, centralised, gradient/Laplacian QS, **RouteQSNetwork** |
| `sim/route_graph.py`, `sim/test_route_graph.py` | Route graph, compartment + synthetic site, unit tests |
| `sim/qs_alert.py` | E1–E5 (compartment scale) |
| `sim/site_flow.py`, `sim/site_experiment.py` | Site-scale flow model, E8 (incl. failures, fail-safe, hybrid) |
| `sim/failure_experiment.py` | E10: board failures at critical places, site scale |
| `sim/esp32_sil.py`, `firmware/sil/sil.cpp` | E9: software-in-the-loop test of the firmware |
| `sim/analytic_model.py` | E11: mathematical model vs simulation |
| `sim/sensing.py`, `sim/test_sensing.py` | Sensing layer (counters, density readings, 2-state Kalman filter, sensor faults) and its unit tests |
| `sim/sensing_experiment.py`, `sim/sensing_failures.py` | E12, E12b, E12d; E12c (failures under realistic sensing) |
| `sim/v4_experiment.py` | E13: firmware v4 options |
| `sim/field_experiment.py` | E14: police delay and compliance; E15: alert-threshold sensitivity |
| `sim/e3_sensing.py` | E16: JuPedSim compartment with realistic sensing |
| `firmware/include/pk_sense.h`, `firmware/sil/sense_check.cpp`, `sim/test_sense_port.py` | Firmware density estimator and its equivalence test against `sensing.kf_step` |
| `firmware/include/pk_qs.h` | Route-graph QS + failure handling, shared by firmware and SIL |
| `sim/separation_sim.py`, `sim/companion_sim.py`, `sim/reunion_sim.py` | PS-03 simulations |
| `backend/`, `dashboard/`, `firmware/` | Prototype |
| `docs/demo_script.md` | Demo |
| `literature_survey.md`, `references.bib` | Literature |

## 9. Key references (full list in `references.bib`)
Helbing 2007 · Johansson 2008 · Weidmann 1993 · Seyfried 2005 · Still 2000/2014 · Fruin 1971 · Illiyas 2013 · Vanumu 2020 · Feliciani 2023 · Boyd 2006 · Olfati-Saber 2007 · Cattivelli 2010 · Pratt 2002 · Dressler & Akan 2010 · Wokoma 2004 · Bor 2016 · Adelantado 2017 · Nikodem 2019 · Rondon 2020 · Leith & Farrell 2020 · Januszkiewicz 2018 · Cotton 2011 · Basalamah 2016 · Troncoso 2020 (DP-3T) · Heinrich 2021/2022 · DPDP Act 2023 · NDMA crowd guide 2014 · RiMEA guideline · ISO 20414:2020 · NIST TN 1822 · Moussaïd 2010 · Group-In (arXiv 2005.12848) · Solmaz & Wu (arXiv 1808.01023).
