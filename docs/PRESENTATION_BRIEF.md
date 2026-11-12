# Safer Ghats, Faster Reunions
### Godavari Pushkaralu 2027: PS-01 Crowd Monitoring · PS-03 Lost-Person Reunification
*Team IIIT Hyderabad · RTIH × RMC Innovation Challenge*

> This is the presentation version. The full methods, parameters, ablations and limitations are in `TECHNICAL_REPORT.md`.

---

## PS-01: Crowd alerts that follow the pilgrim route

### The problem
- In **2015, 27 pilgrims died** in a first-day surge at a narrow ghat entrance.
- A crush builds when **more people walk in than can leave**. The police who can stop it are **on the approach road**, and they **can't see** the crowding ahead.
- Today's tools (CCTV counts, local thresholds) warn **late**, warn **locally**, and depend on the **control room network staying up**.

### Our solution, in one line
> **Every gate, road and junction has a counter that knows which ghats it feeds. When a ghat starts filling faster than it empties, the counters upstream turn amber or red, and the police there slow people, hold them, or send them to a neighbouring ghat with room. It keeps working even if the control room or network goes down.**

### How it works (3 parts)
| Part | What | Why |
|---|---|---|
| **1. Sensing** | Line counters at gates and exits (where barricades already form lanes), **density from existing CCTV or an edge camera** on wide roads, fused on the node | Know how full each place is, without counting errors piling up; counts only, no faces |
| **2. Route map** | A one-time map of which road feeds which junction and ghat | Warn **only** the places that feed the problem |
| **3. Alert relay** | Each counter passes warnings to the counters feeding it, over a low-cost radio mesh | Works **without** the control room |

### What the police see
🟢 **Normal**: let people through · 🟡 **Amber**: slow the flow, or divert to the free branch · 🔴 **Red**: hold here
*(Aligned with the NDMA guide for managing crowds at mass gatherings.)*

### How we tested it (four independent checks)
1. **Crowd simulator.** We used **JuPedSim**, an open-source crowd simulator from Germany's Forschungszentrum Jülich. Our model of it passes the **RiMEA** international verification tests and is **calibrated** to standard walking-speed data. Every result was **repeated over 5 random seeds**.
2. **Whole site.** A flow model of **68 compartments, 20 junctions and 10 Nagaras**, tested on **20 different surge and blockage scenarios**, then on **100 each with realistic, imperfect sensors**.
3. **The actual device code.** The ESP32 alert code was run as **244 virtual boards** over a simulated radio, losing 30% of messages. Its decisions matched our design model in **100%** of 1.8 million board-seconds.
4. **Mathematics.** A one-equation model of a filling compartment predicts the simulator's results without being fitted to them: danger at **670 s** (simulator: 672 s), peak with our system **3.24** (simulator: 3.28).

### Results
**1. Today's practice does not stop a surge.** Local thresholds behaved exactly like no system: the crowd stayed above the safe limit (4 persons/m²) for **about 14 minutes**, peaking at 6 persons/m².

**2. Our system kept the crowd below the safe limit every time.**
| Ghat surge (crowd simulator, 5 runs) | Runs kept safe | Peak crowd (persons/m²) |
|---|---|---|
| No system / today's thresholds | 0 of 5 | 6.0 |
| Command centre (**network down**) | 0 of 5 | 6.0 |
| **Our route-aware mesh (no network needed)** | **5 of 5** | **3.3** |
| Command centre (network up) | 5 of 5 | 2.9 |
| **Our hybrid: command centre + mesh** | **5 of 5** | **2.9**, and still **5 of 5** when the network is cut mid-surge |

**Across the whole site (20 scenarios each):**
- **Surges:** our mesh kept every place safe in **20 of 20**. With the command centre cut off, only 6 of 20 were safe.
- **Blocked exits:** our mesh was **20 of 20** and **diverted** pilgrims to neighbouring compartments, with a lower peak than the command centre (2.4 vs 3.4 persons/m²) and **4× fewer holds**.

**3. It survives broken devices. We broke it on purpose, found a flaw, and fixed it.**
- **The flaw.** With the first version, one dead board at the gate made the system almost as bad as having none (about 13 minutes above the safe limit).
- **The fix.** The final firmware treats a silent board as **"unknown is not safe"**: it keeps the warning, listens one hop further, and tells staff which board to check.
- **The result:** **safe in every test** with a dead gate, junction or compartment board, and with **5% of all boards dead at random**.

**4. It doesn't over-react.**
- **Alerts follow the route,** so unrelated ghats aren't held.
- **Normal day:** zero false alerts.
- **Near-misses (be honest about this):** in about 1 in 4 near-miss scenarios, where the crowd rose fast but stopped short of danger, it slowed the flow as a precaution.

**5. It warns early.** In the compartment surge it warned the road police about **3 minutes before** the danger point. Today's thresholds **never** warned the road.

### Why it is different
- **Route-aware:** warns the right places, not everywhere nearby.
- **Resilient:** decisions are made at the gates themselves, so a network failure doesn't stop them. A dead device is covered by its neighbours and reported to staff.
- **Makes the ICCC stronger, doesn't replace it.** When the network is up, the command centre's decisions are used. When the network fails (which is exactly when mass gatherings overload it), the mesh carries on.
- **Privacy-preserving:** cameras count on the device; counts only, no stored video. It **uses the existing CCTV** where available.
- **Tested against realistic sensors:** counters alone fail 15 of 20 surge tests (their error piles up); counters + camera readings fused on the node are safe in **100 of 100**.
- **Integrates:** a documented API feeds the RMC command centre's dashboard.

---

## PS-03: Reuniting lost children and elders, with nothing for families to carry

### The problem
- Mass gatherings separate thousands of people. At Maha Kumbh 2025, about **50,000** were reunited, **mostly women and the elderly**.
- Many rural pilgrims have **no smartphone**, and only one family member may have a phone.
- Facial recognition is **not allowed** for Pushkaralu.

### Our solution
| Step | What happens |
|---|---|
| **1. Band at entry** | **Only children and senior citizens** get a band, with a QR code, helpline and case code. No phone number is printed on it. The family gives **any reachable number** (a feature phone, relative or group leader) and is told **which help desk to go to** if separated. |
| **2. Quiet sightings** | The same gate and road nodes from PS-01 note when a band passes. They keep only **anonymous rotating IDs**, deleted after 60 minutes, and nothing is tracked centrally. |
| **3. Report** | The family reports at a **help desk, by missed call (IVR), or via a volunteer**. The system then finds the band's **last-seen place and trail**. |
| **4. Meet at the right desk** | The found child is taken to the **desk nearest where they were separated**, the same desk the family was told to go to. |
| **5. Exit watch** | If a reported band passes an exit gate, the post there is alerted. This covers a child led away by a stranger. |

### Results (simulation, behaviour assumptions stated)
| Approach | Median time to reunite | Reunited within 30 min |
|---|---|---|
| Today (PA announcements + registers) | **37 min** | 28% |
| **Our design** (band + trail + meet at the right desk) | **17 min** | **79%** |

- Even if only **30%** of families follow the "go to your desk" advice, the median stays **under 30 minutes**.
- **Experimental extra:** for families whose phone the band learns, the band can flag separation **on its own**, and a volunteer goes straight to the child. That brings the median to **14 minutes**. It is still being tuned to reduce false alarms in dense crowds, so it is offered as an **extension**, not the core.

### Privacy by design
No faces and no names on the band. IDs change every few minutes, so strangers can't follow a child. Data stays on local nodes and is deleted after 60 minutes, and a search runs only when a family reports. Guardian consent is taken at band issuance, in line with the DPDP Act.

---

## Prototype (to be shown live)
- **ESP32 nodes and a band tag.** The firmware is complete for all three roles and uses about 22% of the board's memory. It was tested as 244 virtual boards before going on the hardware.
- **Live demo:**
  1. A simulated surge drives the real boards.
  2. We **switch off the server**, and the road node still turns amber over the radio mesh.
  3. We **unplug a node**, and the road keeps holding. The dashboard shows "NODE DOWN – send staff".
  4. We walk the band past the nodes, then report it via IVR. The dashboard shows the **trail** and the **desk to go to**.
- **Command-centre dashboard** and a **documented REST API** (OpenAPI) for integration.

## Pilot roadmap (Maha Shivaratri, Feb–Mar 2027 → Pushkaralu, Jun–Jul 2027)
| Phase | What | Measured |
|---|---|---|
| Nov 2026 – Jan 2027 | Map routes for the Kotilingala model compartment; install counters and nodes; integrate CCTV counts and the ICCC | Counting accuracy vs manual counts |
| Feb – Mar 2027 | **Live pilot at Maha Shivaratri** (PS-01 and PS-03) | Warning lead time, false alerts, radio reliability, reunion times, how many pilgrims' phones are detectable |
| Mar – Jun 2027 | Scale to 68 compartments, 20 junctions and 10 Nagaras; police and volunteer drills | Drill response times |
| Jun – Jul 2027 | 12-day operation from the ICCC | Live KPIs |

## What we are honest about
- **All results so far are from simulation.** The site layout and the family behaviour in the reunion model are **assumptions** until the pilot.
- **The command centre with a working network is still slightly better on a surge** (peak 2.9 vs 3.3 persons/m²). That is why we propose the **hybrid**: use the command centre when it can see, and the mesh always.
- **Two neighbouring boards failing together** (gate and junction) can still leave a gap, in 2 of 5 tests. The command centre covers that case when the network is up.
- **The boards are not yet flashed and field-tested.** The device code has been tested in simulation.
- **Counters work only in lanes,** so they go only where barricades already form them (gates, exits). Wide roads need a camera density reading at least every 2 minutes with ≤ 20% error; sensor errors are modelled until the pilot measures them.
- **Nodes need a power, weather and maintenance plan.** This is part of the pilot design.
- **A parent with no phone can't be detected by any radio system.** For them, the "meet at the right desk" step is what makes the difference, and the simulation shows it works without any device.

## What we need from RMC / RTIH
1. Kotilingala model-compartment layout and the planned pedestrian routes.
2. Access to the CCTV crowd counts and the ICCC integration interface.
3. Pilot permission for Maha Shivaratri, and coordination with police and volunteers.

---
*Detailed methods, parameters, every experiment (including what failed) and the reproduction steps: `TECHNICAL_REPORT.md`.*
