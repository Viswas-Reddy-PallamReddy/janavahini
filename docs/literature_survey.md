# Literature Survey: Quorum-Sensing IoT for Crowd Safety and Lost-Person Reunification

**Context:** RTIH × RMC *Innovation-Led Solutions for Godavari Pushkaralu 2027*.
- **PS-01:** Real-Time Crowd Monitoring and Congestion Management.
- **PS-03:** Lost-Person Identification and Reunification Assistance.
- **Compiled:** 30 Oct 2026. Submission deadline 7 Nov 2026.

**How to read this:** `[Key]` citations match `references.bib`. Every DOI was confirmed as registered at doi.org on 30 Oct 2026. Non-DOI sources were opened during the search. A few sites block automated checkers (ResearchGate, FBK, Jülich data archive, Umm Al-Qura repository); open those in a browser. Items marked ⚠ need a check before you cite them.

---

## 0. Executive summary

1. **The problem is well documented.**
   - 64% of stampede fatalities happen at religious gatherings [Vanumu2020].
   - India is a global hotspot for crowd accidents [Illiyas2013; Feliciani2023].
   - The 2015 Godavari Pushkaralu lost 29 people on day 1, in a surge at a narrow ghat entrance [Pushkaram2015GapAnalysis].
   - The only prior local system was CCTV line counting at 13 ghats. It gave hourly updates with an accuracy of about 70–75% [IIITH2018].
   - **Room for improvement:** faster updates, higher accuracy, no cameras, and no dependence on a central server.

2. **Density alone is a weak alarm.** In the 2006 Hajj Jamarat disaster:
   - Flow fell below about 0.8 persons/m/s more than 30 minutes before the accident.
   - Stop-and-go waves appeared, followed by "crowd turbulence" about 10 minutes before it.
   - Density stayed high for hours without giving a specific warning [Helbing2007; Johansson2008].
   - **Consequence:** the alert variable should combine density, flow, its rate of change, and crowd pressure.

3. **Cheap, camera-free counters work, with known limits.**
   - Thermal 8×8 arrays count doorway flow with about 93% accuracy indoors [Mohammadmoradi2017], but lose contrast when the air approaches skin temperature [Cerutti2018].
   - ToF line counters miss people walking side by side [STVL53L1XCounting].
   - mmWave radar tracking breaks down above about 2 persons/m² [Toha2025].
   - **Consequence:** narrow the gates, fuse the sensors, and re-zero the counters regularly.

4. **RF (Wi-Fi/BLE) counts are proxies, not headcounts.**
   - Raw counts correlate only r ≈ 0.53–0.61 with ground truth [Schauer2014].
   - MAC randomisation inflates unique-device counts [Martin2017; Vanhoef2016].
   - The best privacy template is per-minute counting with a rotating peppered hash [Determe2022].

5. **Our quorum-sensing (QS) algorithm has solid theoretical grounding.** It combines:
   - quorum thresholds from biology [Pratt2002];
   - consensus and gossip [Boyd2006; OlfatiSaber2007];
   - diffusion estimation [Cattivelli2010];
   - bio-inspired networking [DresslerAkan2010].

   Prior QS-inspired networking targets clustering [Wokoma2004], ad-hoc network voting [Peysakhov2006] or robot swarms [Oddi2024]. **To our knowledge, nobody has applied a leaky-diffusion "autoinducer" field to crowd-crush early warning that works without backhaul.**

6. **BLE child tags are proven at Hajj, but signal-strength (RSSI) distance is not reliable.**
   - 600 BLE tags carried by Hajj pilgrims were detected about 90% of the time [Basalamah2016].
   - RSSI does not fall reliably with distance [Leith2020].
   - One person blocking the line of sight costs about 10 dB [Januszkiewicz2018], and outdoor body-to-body links can lose up to 40 dB [Cotton2011].
   - **Consequence:** use coarse states with persistence and a guardian "cancel" step. Our design does this.

7. **Privacy and law are the biggest risk for PS-03.**
   - DPDP Act §9(3) bans tracking children by default [DPDPAct2023].
   - The 2025 Rules may exempt real-time location for safety, but sources disagree ⚠ [DPDPRules2025].
   - Design pattern: rotating IDs as in DP-3T/GAEN [Troncoso2020; AppleGoogle2020], plus crowd-sourced sightings encrypted to the guardian's key, adapted from Apple's Find My [Heinrich2021].

---

## 1. Problem evidence: stampedes and mass gatherings in India

| Key | What it gives us | Supports |
|---|---|---|
| [Illiyas2013] | Five decades of Indian stampedes. Hazards: narrow passages, slopes, dead ends, slippery floors, converging flows. A press-cited "79% at religious events" ⚠ is not verified in the paper. | PS-01 |
| [Vanumu2020] | 137 stampedes, 1883–2017. Religious gatherings account for 64% of fatalities. Causes: narrow passages about 27%, overcrowding 23%, rumours 21%. | PS-01 |
| [Feliciani2023] | 281 crowd accidents, 1900–2019. India is a hotspot; about 70% of Indian cases in 2000–2019 were at religious events. | PS-01 |
| [Pushkaram2015GapAnalysis] | 2015 Pushkaralu: 29 deaths. Gaps in diversion, evacuation and health posts. Day-1 surge at a narrow entrance, with barricade breaches. Author and year are not shown on the listing ⚠. | both |
| [IIITH2018] | **Direct local precedent.** IIIT-H drew virtual counting lines on CCTV at 13 ghat entrances in 2015, with an hourly Web-GIS dashboard used by police for diversion. Accuracy about 70–75%. Repeated at Krishna Pushkaralu 2016. "Technology is 30–40% of the job." | PS-01 |
| [Balsari2016] | Kumbh 2013 (more than 70 million pilgrims): administrators could neither estimate nor limit attendance. | PS-01 |
| [Mehrotra2015] | Harvard book *Mapping the Ephemeral Megacity*: sector layout and governance of a temporary city. | both |
| [Felemban2020] | Survey of Hajj crowd technology (RFID, BLE, Wi-Fi, cameras, apps). Use it for the related-work comparison table. | both |
| [Liang2024] | Seoul Itaewon crush. The first emergency call came about 3 h 41 min before the crush. Simulated one-way routing cut peak density to 2.86 persons/m². | PS-01 |

**Lessons:**
- Failures cluster on **day 1 and at auspicious bathing times (muhurtham)**, and at **narrow entries**. Alert on the **inflow rate at entries**, not only on occupancy.
- **Response workflow matters as much as sensing** [IIITH2018]. Include who receives the alert and what action follows, such as diverting pilgrims to the next ghat.

---

## 2. Crowd safety science: what the alert should measure

| Key | Method or number | How we use it |
|---|---|---|
| [Helbing2007] | Laminar flow turns into stop-and-go, then turbulence, before people fall (Hajj 2006 video). | Regime detector per compartment; each regime transition raises the alert one level. |
| [Johansson2008] | Crowd pressure **P = ρ·Var(V)**. Turbulence began at about 0.02 s⁻²; the value depends on the averaging radius, so calibrate it. | Compute P per node over a sliding window as an extra QS production term. |
| [Seyfried2005] | Speed falls linearly with 1/density (fundamental diagram). | Convert density into expected flow and capacity. |
| [Weidmann1993] | Fundamental diagram J = ρ·v₀·(1 − exp(−γ(1/ρ − 1/ρ_max))), with v₀ = 1.34 m/s, ρ_max = 5.4 /m², γ = 1.913 /m². | Default flow model at ghat entries; recalibrate locally because flow on bathing steps goes both ways. |
| [Fruin1971] | Level-of-service bands A–F (walkway: A > 3.3 m²/person, B 2.3–3.3, C 1.4–2.3). | Operator colour scale only, **not** a crush criterion. |
| [Still2000; Still2014] | Green Guide limit of 4 persons/m² for a moving crowd; alert if it holds for more than 6 minutes. Shock waves are possible above 5–6 persons/m². Time-to-critical formula. | Two-part trigger: density plus how long it has lasted. **Time to critical** = (ρ_crit − ρ_now)·Area / (inflow − outflow). |
| [Zhang2017] | ST-ResNet: grid inflow/outflow forecasting from recent, daily-cycle and trend branches plus external features. | Compartment-level flow forecast, using the muhurtham timetable as an external feature. |
| [Singh2020] | Wi-Fi-sensor crowd counts at an event forecast 30 minutes ahead with a ConvLSTM, which beat a random-walk baseline. | 15–30 minute forecast; **always report the gain over a naive baseline**. |

**Thresholds collected from the literature (calibrate per ghat):**

| Signal | Value | Source |
|---|---|---|
| Safe limit for a moving crowd | 4 persons/m² | [Still2000] |
| Shock waves possible | 5–6 persons/m² | [Still2014] |
| Jam density | 5.4 persons/m² | [Weidmann1993] |
| Critical flow drop | < 0.8 persons/m/s | [Johansson2008] |
| Crowd pressure, turbulence onset | ≈ 0.02 s⁻² | [Johansson2008] |
| Densities reached in disasters | ≥ 10 persons/m² (Jamarat); 7.57 mean / 9.95 peak (Itaewon, simulated) | [Johansson2008; Liang2024] |

---

## 3. Sensing layer A: camera-free people counters (PS-01)

| Key | Sensor | Reported performance | How we use it |
|---|---|---|---|
| [Beltran2013] | 8×8 Grid-EYE thermal array plus PIR | Occupancy RMSE ≈ 0.35 persons, indoors | Pipeline: background subtraction → features → count. |
| [Mohammadmoradi2017] | 8×8 IR array at a doorway | ≈ 93% entry/exit accuracy; break-beams fail when several people pass at once | **Bidirectional gate counter**; occupancy = cumulative in − out. ⚠ Bosch patent US10948354. |
| [Cokbas2020] | Overhead low-res thermal "tripwire" | First public low-res thermal entry/exit dataset, with metrics for drift and jitter | **Training and benchmark data**; track counter drift. |
| [Cerutti2018] | 8×8 thermal, outdoors | Signal falls as air temperature approaches skin temperature; a temperature-dependent threshold beats a fixed one | **Critical for Rajahmundry**: air at 30–38 °C, and bathers come out of the river cooler. |
| [Perra2021] | Low-cost IR array at a doorway | Citation verified only | Secondary reference. |
| [STVL53L1XCounting] | VL53L1X ToF, two zones | Zone trigger order gives direction; misses people side by side | **Cheapest gate counter** for gates of 1–1.5 m or less. |
| [Zhao2019] | TI mmWave radar (IWR1443) | Median position error 0.16 m; DBSCAN clustering + Hungarian matching + Kalman filter | Tracker for gate-level flow. **Drop the gait-identification stage** for privacy. |
| [Toha2025] | mmWave, static dense crowds | F1 87% in familiar environments, 60% in unseen ones; tracking fails above about 2 persons/m² | Radar for **flow at gates**, not headcount inside a packed compartment. |
| [TIPeopleCounting] | TI IWR6843 reference design | Vendor states ~1 person/m² counting density; anecdotal undercount of 8 counted for 10 present | Ready-made firmware; budget for undercounting. |

**Pitfalls and design responses:**
- **Side-by-side passage and occlusion** break every line counter. Use **barricades that force single or double file** at gates, which is also standard crowd practice.
- **Cumulative counters drift** [Cokbas2020]. Re-zero when a compartment is cleared between bathing slots, or cross-check against RF counts.
- **Outdoor thermal contrast** [Cerutti2018]. Make the threshold depend on air temperature, and fuse with ToF.

---

## 4. Sensing layer B: RF (Wi-Fi/BLE) counting and privacy (PS-01)

| Key | Method | Numbers | How we use it |
|---|---|---|---|
| [Schauer2014] | Passive Wi-Fi/BT capture at an airport | r = 0.53 (BT), 0.61 (Wi-Fi) raw; 0.75 after filtering | RF is a proxy; **calibrate against gate counts**. |
| [Weppner2013] | Collaborative BT scanning by phones | > 75% accuracy across 7 density classes with about 0.2% of people scanning | Output **density classes** (green/amber/red), not exact counts. |
| [Determe2022] | Probe counting in 1-minute windows; SHA-256 with a pepper rotated every minute | MAPE 12.7% over 9 weeks; about 5 devices per person (partial coverage) | **Privacy template for ESP32 firmware.** |
| [Rusca2024a; Rusca2024b] | Clustering to undo MAC randomisation, then a Bloom filter with added noise (γ-deniability) | Accuracy not read (full text closed) | Nodes keep sketches, never raw IDs; the gateway merges them. |
| [Martin2017] | Study of MAC randomisation | Android randomisation defeated in about 96% of tested phones | Expect inflated unique-device counts; use windowed counting. |
| [Vanhoef2016] | Fingerprinting from probe-request information elements and sequence numbers | ≥ 50% of devices trackable for 20 minutes or more | De-duplicate only on the node and only briefly, because fingerprinting is itself a privacy risk. |
| [Bessho2021] | Fixed BLE sniffers in Tokyo streets | Matches manual counts; SVR model R² ≥ 0.9 | Fixed cheap BLE scanners work; aggregate over 15–30 minutes. |
| [Depatla2018] | Device-free RSSI link counting | Within 2 people for up to 20 people | Backup flow sensor only; not validated in dense crowds. |
| [Xi2014] | Device-free crowd counting from Wi-Fi CSI | Numbers not verified ⚠ | Background for the ESP32 CSI option. |
| [Wirz2013] | Smartphone walking speed → density | 0.36 persons/m² calibration error from 828 users | Gate-to-gate **transit speed** as a density proxy. |
| [Blanke2014] | Official festival app with GPS (Züri Fäscht) | 28k contributing users | Optional opt-in app layer; uptake skews the sample. |

**Pitfalls:**
- Rural and elderly pilgrims carry fewer phones with Wi-Fi on, which biases RF counts downward. Use RF only as a **secondary** signal.
- Keep only per-window aggregates and **rotate salts every minute** [Determe2022].

---

## 5. The quorum-sensing decentralised alert algorithm (PS-01 core)

| Key | Contribution | How we use it |
|---|---|---|
| [Pratt2002] | Ants switch behaviour when the rate at which they encounter nestmates passes a quorum. | The biological basis for "local encounter rate passes a threshold, so the state switches". |
| [DresslerAkan2010] | Survey of bio-inspired networking, with a method for mapping biology to protocols. | Template for framing the "bio-inspired" claim. |
| [Wokoma2004] | WSN clustering inspired by quorum sensing and lateral inhibition. | **Closest prior art**; it does clustering, we do safety alerts. |
| [Peysakhov2006] | Quorum sensing on real mobile ad-hoc networks. | Prior art: collective thresholds work on real ad-hoc radios. |
| [Oddi2024] | Minimal quorum-estimation protocols for robot swarms; anonymous protocols are cheapest. | Supports anonymous (identity-free) density sensing on nodes. |
| [WernerAllen2005] | Firefly-inspired synchronisation on lossy multi-hop motes, about 100 µs. | Proof that bio-inspired rules work on real radios; can align gossip slots. |
| [Boyd2006] | Randomised gossip; convergence set by the second-largest eigenvalue; Metropolis weights. | Choosing D and the gossip period; weight design. |
| [OlfatiSaber2007] | Laplacian consensus: stability, link failures, delays. | Our diffusion term is a Laplacian step. |
| [Cattivelli2010] | Diffusion LMS ("adapt-then-combine") for tracking changing quantities, robust to node and link failures. | Justifies leaky diffusion over pure consensus. |

**Algorithm (as proposed), and what the literature implies for it:**

```
A_i(t+1) = δ·A_i(t) + p·s_i(t) + D·Σ_j w_ij (A_j(t) − A_i(t))
s_i(t)   = f(density, inflow − outflow, crowd pressure P, flow drop)   ← §2 metrics
state_i  = NORMAL / WATCH / CRITICAL by θ thresholds with hysteresis and minimum dwell time
```

- **Do not use pure averaging:** it smooths away the hotspot. The leak δ < 1 plus the local source keeps a gradient. The steady state is A ≈ p·[(1−δ)I + D·L]⁻¹·s, a "screened diffusion" filter. **D/(1−δ) sets the alert's reach in hops.** This is our own derivation from [OlfatiSaber2007; Cattivelli2010]; state it and tune it.
- **Stability:** keep D·(maximum node degree) < 1, or use Metropolis weights w_ij [Boyd2006]. Re-normalise when links drop.
- **Timestamp messages and tolerate stale values from neighbours** [WernerAllen2005].
- **Early-warning feature:** a rising A at upstream nodes on an access route signals a surge before it reaches the ghat.

---

## 6. Networking layer

| Key | Finding | Design decision |
|---|---|---|
| [Bor2016] | LoRa with fixed settings: about 120 nodes per 3.8 ha per sink; scaling needs dynamic spreading factor (SF) and power, or more sinks. | LoRa as a **sparse backbone**, several gateways; LoRaSim for sizing. |
| [Adelantado2017] | LoRaWAN limits on capacity, duty cycle and downlink. | LoRaWAN uplink is wrong for low-latency alerts, so use **peer-to-peer gossip**. |
| [Cotrim2020] | Review of LoRa mesh; most mesh proposals were evaluated at small scale. | Taxonomy; argues for our own scaled simulation. |
| [Meshtastic; HernandezOrtiz2026] | Managed flooding: a node skips its rebroadcast if it hears another relay. Usable beyond 100 nodes but "success not guaranteed". SF7–8 links fail at about 110–120 dB path loss; SF11–12 reach about 180 dB. The 2026 paper is a preprint. | Listen-and-suppress flooding with a **hop limit of about 3**; SF9–10. |
| [Rondon2020] | Bluetooth Mesh flooding suffers broadcast storms as density grows. | **No BLE Mesh** on tags; tags only advertise, with random jitter. |
| [Nikodem2019] | 210 co-located BLE advertisers: about 43–47% per-packet delivery, but ≥ 99.9% delivered within a 10 s window. Collision probability for 200 tags falls from 0.51 at a 200 ms interval to 0.15 at 1000 ms. The scanner is the bottleneck. | Slow advertising normally; **burst only in LOST mode**; a dedicated scanner per node. |
| [Vahdat2000] | Epidemic (store-and-forward) routing for disconnected networks. | Volunteer phones cache sightings and upload them when they reconnect. |
| [Kanjo2013] | Opportunistic phone/sensor message passing for Hajj crowds (report). | Precedent from a mass pilgrimage. |
| [TTN-IN865] | IN865–867 band: max EIRP +30 dBm; regional parameters list **no duty-cycle cap** (unlike EU 1%). | ⚠ Confirm with the DoT/WPC notification. Airtime is still shared, so budget it. |

**ESP-NOW:** no strong peer-reviewed evaluation was found. Espressif's vendor blog reports under 20 ms latency at up to about 300 m line of sight. **Run your own small field test**, and use LoRa for longer hops.

---

## 7–8. PS-03 sections
Child tags and reunification are on the `ps-03` branch.

## 9. Privacy, security and law

| Key | Content | How we use it |
|---|---|---|
| [Troncoso2020] | DP-3T: ephemeral IDs derived from a secret key; matching happens locally. | Tag ID = HMAC(K, time slot); only the guardian holds K. |
| [AppleGoogle2020] | GAEN key schedule. The rolling identifier changes every 10 minutes **together with** the BLE MAC address (about every 15 minutes). | Rotate the payload ID and MAC **at the same instant**. Avoid static fields (fixed TX power, constant flag pattern) that fingerprint the tag. |
| [Heinrich2021] | Analysis of Apple Find My: finders encrypt location to the lost device's rotating key; about 10 m accuracy in cities; a correlation flaw exposed up to 7 days of history. | Volunteers encrypt (time, zone) to the guardian's public key; short retention; avoid the correlation flaw. |
| [Heinrich2022] | AirGuard anti-stalking app: flags a tracker after ≥ 30 minutes, ≥ 3 sightings, ≥ 400 m moving with the user. | Threat-model both ways. Strangers must not follow a child's tag, and a LOST-mode child tag may set off anti-stalking alerts. |
| [Ledvina2024] | IETF DULT draft on detecting unwanted location trackers. | Align the tag's behaviour with platform tracker detection. |
| [Aktay2020] | Google mobility reports: Laplace noise, ε = 0.44 per metric, small cells suppressed. | Add differential-privacy noise to public dashboards; suppress small cells. |
| [DPDPAct2023] | §9(1) verifiable parental consent (child = under 18); §9(3) **no tracking or behavioural monitoring of children**; exemptions only by rule. | Consent at the registration kiosk; data minimisation; location revealed only in LOST state; delete after the event. |
| [DPDPRules2025] ⚠ | Fourth Schedule exemptions (education, crèches, transport). **Sources disagree** on whether a "real-time location of a child for safety" purpose was added. One secondary claim says §9 obligations start about May 2027. | **Check Gazette G.S.R. 846(E) before submitting.** |

---

## 10. Simulation and validation tools

| Key | Tool or data | Role in our demo |
|---|---|---|
| [HelbingMolnar1995] | Social-force pedestrian model | Generate synthetic compartment density fields. |
| [JuPedSim] | Jülich Python pedestrian simulator (social force from v1.2; anticipation velocity model, v1.3) | **Recommended**: script a ghat compartment, sample density at node positions. |
| [Kleinmeier2019] | Vadere simulator | Alternative to JuPedSim. |
| [TerHoeven2025] | Mesa 3 agent-based modelling (Python) | Co-simulate nodes as agents on a NetworkX graph with link drops, plus child and guardian agents. |
| [Magrin2017] | ns-3 LoRaWAN module (> 95% packet success with about 10⁴ devices across several gateways) | Get realistic LoRa loss rates. |
| [Slabicki2018] | FLoRa (OMNeT++) | Tune spreading factor and power. |
| [JulichArchive] | Jülich pedestrian experiment data (bottleneck, CroMa 2021) | **Ground truth**: replay bottleneck trajectories and show the QS alert fires **before** peak density. |
| [Cokbas2020] | Low-res thermal entry/exit dataset | Benchmark the counting algorithm. |

**Caveats:**
- These are lab-scale datasets, so present results as **mechanism validation**, not field accuracy.
- Argue scale from the decentralised design, which only ever uses local information.

---

## 11. Methods → system component map

| Component | Method to adopt | Key sources |
|---|---|---|
| Gate counter (PS-01) | IR array or ToF bidirectional line crossing, barricaded single file, temperature-adaptive threshold, periodic re-zero | Mohammadmoradi2017, STVL53L1XCounting, Cerutti2018, Cokbas2020 |
| Flow and speed sensor | mmWave DBSCAN + Kalman at gates (no identification); transit-speed proxy | Zhao2019, Toha2025, Wirz2013 |
| RF secondary signal | 1-minute windowed probe/BLE counts, pepper rotated every minute, Bloom filter | Determe2022, Rusca2024a, Schauer2014 |
| Alert metric | density + inflow−outflow + crowd pressure P + flow drop + dwell time; time-to-critical | Johansson2008, Helbing2007, Still2014, Weidmann1993 |
| QS algorithm | leaky diffusion, Metropolis weights, hysteresis, stale-tolerant | Pratt2002, Boyd2006, OlfatiSaber2007, Cattivelli2010 |
| Forecast | 15–30 min inflow forecast (LSTM/ST-ResNet), random-walk baseline | Singh2020, Zhang2017 |
| Node network | ESP-NOW short hops; LoRa peer-to-peer flooding with suppression, hop limit about 3, SF9–10 | Meshtastic, Bor2016, Adelantado2017 |
| Child tag radio | Advertising only, slow normally, fast burst in LOST; no BLE Mesh | Nikodem2019, Rondon2020, Kindt2022 |
| Separation logic | Kalman RSSI + loss rate, 10–15 dB margin, dwell times, guardian cancel, per-pair calibration | Leith2020, Januszkiewicz2018, Cotton2011, Robesaat2017, Smith2011 |
| Elderly users | Wandering-pattern trigger | Kolakowski2019 |
| Sightings and reunification | Crowd-sourced sightings, store-and-forward, encrypted to guardian key, offline wristband fallback | Basalamah2016, Liu2014, Heinrich2021, Vahdat2000, Mohandes2010 |
| Tag privacy | HMAC rotating ID + MAC rotated together, no static fields, anti-stalking threat model | Troncoso2020, AppleGoogle2020, Heinrich2022, Ledvina2024 |
| Dashboards | Differential-privacy noise, small-cell suppression, operator level-of-service colours | Aktay2020, Fruin1971 |
| Validation | JuPedSim/Mesa + Jülich bottleneck data + ns-3/FLoRa loss rates | JuPedSim, JulichArchive, TerHoeven2025, Magrin2017 |

---

## 12. Research gaps: our novelty claims ("to our knowledge")

1. **QS-style leaky-diffusion alert field for crowd safety.** Prior QS-inspired networking targets clustering, voting, robots or nanonetworks. Crowd-density work is centralised or phone-based.
2. **Camera-free, works without backhaul.** The only local precedent used CCTV with hourly updates at about 70–75% accuracy [IIITH2018].
3. **Upstream-gradient early warning** driven by flow, pressure and turbulence indicators [Helbing2007; Johansson2008], not by density thresholds alone.
4. **Privacy-preserving BLE lost-person system for mass gatherings.** No peer-reviewed BLE lost-child study in crowds was found. The Kumbh 2025 approach relies on facial recognition, and its reunion figures are unaudited.
5. **Unified sensing network:** the same nodes serve PS-01 counting and PS-03 sightings.

---

## 13. Open items to check before citing ⚠

- [ ] **DPDP Rules 2025 Fourth Schedule** wording, from Gazette G.S.R. 846(E). This is the main legal risk for PS-03.
- [ ] **IN865–867 duty cycle**: confirm with the DoT/WPC notification.
- [ ] [Illiyas2013] "79% at religious events": check against the paper text.
- [ ] [Pushkaram2015GapAnalysis]: get the authors and year from ResearchGate.
- [ ] [Mohandes2010]: confirm the year.
- [ ] [Xi2014], [Perra2021], [Rusca2024a]: read the full text for accuracy numbers.
- [ ] **Unverified lead, not cited:** an MDPI *Signals* 2026 paper on BLE playground child monitoring (https://www.mdpi.com/2624-6120/7/4/67), which blogs say reports about 99.2% accuracy and about 6.5 s response. Worth reading as a direct comparison.
- [ ] **ESP-NOW outdoor performance**: no solid peer-reviewed source; run your own range test.

Full bibliographic details are in `references.bib`.
