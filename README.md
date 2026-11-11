# Janavahini: route-aware crowd alerts for Godavari Pushkaralu 2027

**Problem Statement 1: Real-Time Crowd Monitoring and Congestion Management**
RTIH × RMC *Innovation-Led Solutions for Godavari Pushkaralu 2027* · Team IIIT Hyderabad: Divya Teja Yekkaluru, Pallamreddy Viswas Reddy

*Janavahini* (జనవాహిని) means "a river of people".

A crush at a bathing ghat builds when more people walk in than can leave. The police who can stop it stand on the approach roads and cannot see the ghat filling. Janavahini splits each road into patches of about 25 m, each with a counter at its boundary:

- every counter estimates how full its patch is and how full it will be a minute later;
- a route map tells each counter which junctions and ghats its road feeds;
- a warning travels **backwards along the pilgrim route** to the police who can slow, hold or divert the inflow.

Decisions are made on the ESP32 nodes, so alerts keep working when the mobile network or the command centre (ICCC) fails. The ICCC gets everything through a documented REST/WebSocket API, and its commands can only add caution.

> **Branches.** This is the `ps-03` branch: PS-01 plus PS-03 (lost-person reunification). `main` holds PS-01 only.

## Status
- **All results are from simulation:** the JuPedSim crowd simulator (RiMEA-verified, calibrated) and a whole-site flow model. See [`docs/TECHNICAL_REPORT.md`](docs/TECHNICAL_REPORT.md).
- **Firmware** compiles and was tested as 244 virtual boards (software-in-the-loop). It has **not yet been flashed or field-tested**.
- **Sensing is idealised:** the simulations give each node the true zone density. Counting noise and drift are listed as open work.

## Headline results (simulation)
| Test | Today's practice | Janavahini |
|---|---|---|
| Ghat surge, JuPedSim, 5 seeds: runs kept below 4 persons/m² | 0 of 5 (peak 6.0) | 5 of 5 (peak 3.3), 30% radio loss, no network |
| Warning reaches the road police | never | 172 s before the danger point |
| Whole site (68 compartments, 20 junctions, 10 Nagaras), 20 surge draws | 6 of 20 safe | 20 of 20 safe |
| Firmware vs Python model, 244 virtual boards | – | 100% identical decisions |

## Repository layout
| Path | Contents |
|---|---|
| `sim/` | Simulations: JuPedSim compartment model, RiMEA tests, calibration, route-graph alert algorithm (`qs_core.py`), experiments E1–E11 |
| `sim/results/` | Summary results (CSV/JSON/PNG). Per-run traces are not committed; re-run the experiments to regenerate them. |
| `firmware/` | PlatformIO ESP32 firmware; `include/pk_qs.h` is the alert logic shared with the SIL test (`firmware/sil/`) |
| `backend/` | FastAPI service: REST API (OpenAPI at `/docs`), WebSocket feed, playback / serial / hardware-in-the-loop modes |
| `dashboard/` | Command-centre dashboard |
| `docs/` | Technical report, presentation brief, demo script, literature survey, references |
| `submission/` | Concept note, technical architecture and workflow, indicative budget and implementation plan (HTML + PDF), demo video |

`firmware/`, `backend/` and `dashboard/` are shared with PS-03, so they also contain the band-tag role and the lost-person API.

## Quick start (Python 3.12)
```
python -m venv .venv
.venv\Scripts\pip install -r requirements.txt
.venv\Scripts\pip install --no-deps jupedsim

cd sim
..\.venv\Scripts\python rimea_tests.py            # verification
..\.venv\Scripts\python calibrate.py              # calibration
..\.venv\Scripts\python qs_alert.py --experiment all
..\.venv\Scripts\python site_experiment.py --seeds 0 1 2

cd ..\backend
..\.venv\Scripts\python app.py --mode playback --scenario surge   # http://127.0.0.1:8000
```
The full reproduction list is in §7 of the technical report.

## Hardware (2–3 ESP32 boards)
```
cd firmware
pio run -e gateway  -t upload --upload-port COM5
pio run -e guardian -t upload --upload-port COM6
cd ..\backend
python app.py --mode hil --port COM5 --scenario surge
```

---

# PS-03: Lost-person reunification (this branch)

**Problem Statement 3: Lost-Person Identification and Reunification Assistance.** It reuses the PS-01 gate and road nodes, and families carry nothing.

| Step | What happens |
|---|---|
| 1. Band at entry | Only children and senior citizens get a band: QR code, helpline and case code, but no phone number. The family gives any reachable number and is told which help desk to go to if they are separated. |
| 2. Quiet sightings | Gate and road nodes note when a band passes. They keep only anonymous rotating IDs, deleted after 60 minutes, and nothing is tracked centrally. |
| 3. Report | The family reports at a help desk, by missed call (IVR) or through a volunteer. The system finds the band's last-seen place and trail. |
| 4. Meet at the right desk | The found person is taken to the desk nearest where they were separated, the same desk the family was told to go to. |
| 5. Exit watch | If a reported band passes an exit gate, the post there is alerted. |

**Simulated result:** median time to reunite **37 min** with today's public-address practice, **17 min** with band + trail + designated desk. Family behaviour in this model is an assumption until the pilot.

## PS-03 files on this branch
| Path | Contents |
|---|---|
| `sim/separation_sim.py` | Separation detection between a child tag and a guardian device (radio-model reference) |
| `sim/companion_sim.py` | Co-walker inference: the tag learns the family's devices, and families wear nothing |
| `sim/reunion_sim.py` | Time-to-reunion: today's practice vs band + registry, trail and designated desk |
| `sim/results/reunion_*`, `separation_*`, `companion_*` | PS-03 results |
| `submission/PS03_Concept_Note.{html,pdf}`, `submission/PS03_Reunification.mp4` | PS-03 submission |

```
cd sim
..\.venv\Scripts\python separation_sim.py --skip-coverage
..\.venv\Scripts\python companion_sim.py --runs 20
..\.venv\Scripts\python reunion_sim.py --runs 3000
cd ..\backend
..\.venv\Scripts\python test_app.py
```
