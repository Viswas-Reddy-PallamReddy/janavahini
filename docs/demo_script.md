# Demo script (about 4 minutes per track)

## Setup
- **Board 1, gateway** (NODE_ID 4, zone C1 "Holding area"): USB to the laptop.
- **Board 2, guardian** (NODE_ID 2, zone R2 "Approach road"): powered from a USB power bank. It stands in for the police post on the road.
- Start the backend:
  ```
  python backend/app.py --mode hil --port COMx --scenario surge
  ```
  Open the dashboard at http://127.0.0.1:8000.

## PS-01: quorum-sensing crowd alert (record as video 1)
1. **Show the problem (30 s).** The 2015 day-1 surge at a narrow ghat entrance killed 27 people. The alarm today is CCTV with hourly counts.
2. **Start the surge replay.** The simulation feeds each board its zone's density. The dashboard map warms up.
3. **Alert before danger.** The holding-area board's LED goes from WATCH (slow blink) to CRITICAL (solid) *before* the zone reaches 4 persons/m². Point out the lead time on the dashboard.
4. **Upstream warning with no server.** Stop the backend (Ctrl+C). The road board's LED still goes to WATCH: the alert reached it by diffusion over ESP-NOW from the holding-area board. This is the instruction to hold inflow on the road.
5. **Show the results.** Use the closed-loop figure from `sim/results` (E3): peak density and seconds above 4 p/m², with and without the system, including when the backhaul is cut.
6. **Integration.** Open `/docs` to show the documented REST API for the command centre.

## Tuning in the room
- Indoors, phones and boards read stronger than in a crowd. Walk 15–20 m away, or behind a wall, to show separation.
- Timings in the tag firmware are shortened for the demo; the deployment values are noted beside each constant.
