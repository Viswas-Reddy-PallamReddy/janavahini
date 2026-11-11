# Demo script (about 4 minutes per track)

## Setup
- **Board 1, gateway** (NODE_ID 4, zone C1 "Holding area"): USB to the laptop.
- **Board 2, guardian** (NODE_ID 2, zone R2 "Approach road"): powered from a USB power bank. It stands in for the police post on the road and for the guardian's device.
- **Board 3, child tag:** a power bank in a pocket.
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

## PS-03: lost-person reunification (record as video 2)
The family carries nothing. Only children and senior citizens wear a band.

1. **Issue the band.** Start the backend; it registers demo band PK-4821. Power on the tag at that moment: the kiosk "activates" it. The tag serial log shows its ID changing every 60 s, with the MAC address changing at the same moment, so a stranger cannot follow the child. Say: "The family is told: *if separated, go to the nearest help desk*."
2. **Auto-flag (families whose phone the band learned).** Start the backend with `--confirm-s 30` for the demo; the deployment value is 180 s.
   - Board 2 (guardian role) stands in for the family's phone: keep it next to the tag for 1–2 minutes. The tag log shows `companions: 1`.
   - Walk the guardian board away. After about 30 s the tag flags *possible separation* (fast blink).
   - **Stage 1 is silent:** nothing reaches a human yet.
   - **Stage 2:** if the flag persists past the confirmation delay, an **AUTO-FLAGGED** case appears with the station actions:
     - a volunteer dispatched to the band's live zone;
     - all nodes watching for the family's learned device;
     - the child to be taken to the desk nearest where the separation happened.
   - When a node hears the family's device, the dashboard adds "*station nearest the parents → tell the family to go to Desk N*".
   - Bring the guardian board back before the delay ends and the flag simply clears: no case, no false alarm.
3. **Walk the "child" through the zones.** Carry the tag past Board 2 (zone R2) and then Board 1 (zone C1). The nodes store only opaque IDs in their 60-min buffers.
4. **The family reports.** Type PK-4821 on the dashboard and choose channel *ivr*, i.e. a missed call from a feature phone.
   - The backend computes the band's IDs for the last hour and sends FIND over the node network. In deployment this travels over LoRa.
   - The nodes answer from their buffers, and the dashboard shows the **trail** and the **last-seen zone**.
   - The band is switched to LOST by a command beacon.
   - The dashboard shows **"Bring the person to: Desk N"**, the desk nearest where the separation happened, which is where the family was told to go.
5. **Exit watch.** If the tag passes an exit node while LOST, the dashboard raises EXIT WATCH.
6. **Found.** Click *Scan at desk*. The case links instantly, the contact is notified (SMS/call stub), and LOST is cleared.
7. **Results.** Show `reunion_time.png`:
   - today's PA practice: median 37 min;
   - converge-at-desk: median 18 min.
   - It is still 14–29 min even if only 30% of families follow the advice.
   - Also show `companion_inference.csv`, stating its limits honestly.

## Tuning in the room
- Indoors, phones and boards read stronger than in a crowd. Walk 15–20 m away, or behind a wall, to show separation.
- Timings in the tag firmware are shortened for the demo; the deployment values are noted beside each constant.
