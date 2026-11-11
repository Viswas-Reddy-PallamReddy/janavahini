"""Pushkaralu backend: REST API for the RMC command centre (ICCC), help desks,
IVR and volunteer handsets, plus a live dashboard.

PS-01 (crowd): zone densities and quorum-sensing alert states.
PS-03 (lost persons): bands for children and senior citizens only; the family
carries nothing. A report (desk / IVR / volunteer) triggers a search of the
nodes' 60-min sighting buffers for the band's rotating IDs -> last-seen zone and
trail; the found person is routed to the desk nearest the separation point,
where the family is told to go; an exit-gate watch flags a LOST band leaving.

Data sources (pick one with --mode):
  playback  replay a simulated scenario through the QS algorithm (no hardware);
            PS-03 searches return a scripted trail, labelled "simulated"
  serial    read JSON lines from the gateway ESP32 over USB and send it
            desk commands (FIND / LOSTCMD / CLEARCMD)
  hil       serial + push simulated zone densities to the boards

Run:
  python app.py --mode playback --scenario surge
  python app.py --mode hil --port COM5 --scenario surge
Then open http://127.0.0.1:8000 (dashboard) or /docs (API).

The decision logic never depends on this server: nodes alert locally and keep
their own sighting buffers. Losing the server loses the overview, not the alerts.
"""

import argparse
import asyncio
import csv
import hashlib
import hmac
import json
import os
import sys
import time
from collections import deque
from pathlib import Path

import numpy as np
import uvicorn
from fastapi import FastAPI, HTTPException, WebSocket, WebSocketDisconnect
from fastapi.responses import FileResponse
from pydantic import BaseModel

ROOT = Path(__file__).resolve().parents[1]
sys.path.insert(0, str(ROOT / "sim"))
import ghat_scenario as gs  # noqa: E402
from qs_core import STATE_NAMES, QSNetwork, QSParams  # noqa: E402

ZONES = {
    "R1": {"label": "Approach road (far)", "area_m2": 240},
    "R2": {"label": "Approach road (near)", "area_m2": 240},
    "G": {"label": "Entry gate", "area_m2": 64},
    "C1": {"label": "Holding area", "area_m2": 300},
    "C2": {"label": "Bathing steps", "area_m2": 300},
    "X": {"label": "Exit corridor", "area_m2": 16},
}
DESKS = {"Desk 1 (Entry gate)": "G", "Desk 2 (Exit)": "X"}
EXIT_ZONES = {"X"}
HW_NODE_ZONE = {4: "C1", 2: "R2"}     # firmware NODE_ID -> zone (hardware demo)
ZONE_HW_NODE = {v: k for k, v in HW_NODE_ZONE.items()}
ROTATE_S = 60                         # must match firmware ROTATE_S (demo value)
SEARCH_WINDOW_S = 3600
T_CONFIRM_S = 180                     # auto-flag must persist this long before a station acts
FLAG_GONE_S = 60                      # no flagged beacon for this long -> companions are back
DEMO_KEY = bytes([0x3a, 0x91, 0x5c, 0x07, 0xe2, 0x4f, 0xb8, 0x16,
                  0xd4, 0x6b, 0x20, 0xaf, 0x75, 0xc9, 0x0e, 0x58])


def hops(a, b):
    """Shortest path length between zones over the node radio graph."""
    adj = {z: set() for z in ZONES}
    for x, y in gs.LINKS:
        adj[x].add(y)
        adj[y].add(x)
    frontier, seen, d = {a}, {a}, 0
    while frontier:
        if b in frontier:
            return d
        frontier = {n for z in frontier for n in adj[z]} - seen
        seen |= frontier
        d += 1
    return 99


def nearest_desk(zone):
    return min(DESKS, key=lambda name: hops(DESKS[name], zone))


def tag_id(key, slot):
    return hmac.new(key, bytes([1]) + int(slot).to_bytes(4, "big"), hashlib.sha256).digest()[:8].hex()


class Store:
    def __init__(self):
        self.zones = {z: {"zone": z, **meta, "density": 0.0, "level": 0.0, "state": "NORMAL",
                          "updated": None} for z, meta in ZONES.items()}
        self.alerts = deque(maxlen=200)
        self.bands = {}          # band_code -> registration
        self.cases = {}          # case_id -> case
        self.pending_flags = {}  # band_code -> stage-1 (silent) auto-flag watch
        self.events = deque(maxlen=100)   # PS-03 log (notifications, exit watch, ...)
        self.inflow_command = "OPEN"
        self.nodes_down = {}     # silent node id -> node(s) holding its last alarm (fail-safe)
        self.sim_time = 0.0
        self.source = "none"
        self.mode = "playback"
        self.ser = None
        self.clients = set()

    def snapshot(self):
        return {"zones": list(self.zones.values()), "alerts": list(self.alerts)[-20:],
                "cases": list(self.cases.values()), "events": list(self.events)[-20:],
                "desks": DESKS, "inflow_command": self.inflow_command,
                "nodes_down": {str(k): sorted(v) for k, v in self.nodes_down.items()},
                "sim_time": self.sim_time, "source": self.source}

    def set_zone(self, z, density, level, state):
        old = self.zones[z]["state"]
        self.zones[z].update(density=round(float(density), 3), level=round(float(level), 3),
                             state=state, updated=time.time())
        if state != old:
            self.alerts.append({"time": time.time(), "sim_time": self.sim_time, "zone": z,
                                "label": ZONES[z]["label"], "from": old, "to": state})

    def log(self, kind, text, case_id=None):
        self.events.append({"time": time.time(), "kind": kind, "text": text, "case": case_id})

    def send(self, line):
        if self.ser is not None:
            self.ser.write((line + "\n").encode())


store = Store()
app = FastAPI(
    title="Pushkaralu Crowd-Safety API",
    version="0.2.0",
    description="Operational API for the RMC Integrated Command & Control Centre (ICCC), help desks, "
                "IVR and volunteer handsets. Crowd alerts are computed on the field nodes "
                "(quorum-sensing mesh). Lost-person bands carry only rotating opaque IDs; the "
                "registry key links them to a case only when a report is filed. No facial recognition.",
)


# ---------------- models ----------------
class Zone(BaseModel):
    zone: str
    label: str
    area_m2: float
    density: float
    level: float
    state: str
    updated: float | None


class BandRegistration(BaseModel):
    band_code: str
    person: str = "child"                  # child | senior
    description: str = ""
    contact: str = ""                      # any reachable number: family, group leader, village contact
    contact_type: str = "family member"
    kiosk_zone: str = "G"


class Report(BaseModel):
    band_code: str | None = None
    description: str = ""
    channel: str = "desk"                  # desk | ivr | volunteer
    desk: str | None = None


class Found(BaseModel):
    band_code: str
    desk: str


# ---------------- PS-01 endpoints ----------------
@app.get("/", include_in_schema=False)
def dashboard():
    return FileResponse(ROOT / "dashboard" / "index.html")


@app.get("/api/v1/health", tags=["system"])
def health():
    return {"status": "ok", "source": store.source, "sim_time": store.sim_time}


@app.get("/api/v1/zones", response_model=list[Zone], tags=["crowd (PS-01)"])
def zones():
    """Current density (persons/m²), QS alert level and state per monitored zone."""
    return list(store.zones.values())


@app.get("/api/v1/alerts", tags=["crowd (PS-01)"])
def alerts(limit: int = 50):
    """Alert state transitions, newest last."""
    return list(store.alerts)[-limit:]


@app.get("/api/v1/inflow", tags=["crowd (PS-01)"])
def inflow():
    """Recommended inflow command at the road head: OPEN, HOLD (divert surplus) or CLOSE."""
    return {"command": store.inflow_command}


@app.get("/api/v1/nodes/down", tags=["crowd (PS-01)"])
def nodes_down():
    """Silent nodes whose last alarm is being held by the nodes behind them (fail-safe)."""
    return {str(k): sorted(v) for k, v in store.nodes_down.items()}


@app.post("/api/v1/nodes/{node}/clear-down", tags=["crowd (PS-01)"])
def clear_down(node: int):
    """Staff have checked a silent node on site: release the fail-safe hold on its last alarm."""
    store.send(f"CLEARDOWN {node}")
    store.log("node", f"Operator cleared the fail-safe hold for {node_label(node)}")
    return {"sent": f"CLEARDOWN {node}"}


# ---------------- PS-03 endpoints ----------------
@app.post("/api/v1/bands", tags=["lost persons (PS-03)"])
def register_band(b: BandRegistration):
    """Kiosk issues and activates a band (children and senior citizens only).
    The accompanying adult is told which desk to go to if separated."""
    return issue_band(b, os.urandom(16))


def issue_band(b: BandRegistration, key: bytes):
    """Each band gets its own secret key (written to the band at activation);
    only the registry can turn its rotating IDs back into a band code."""
    store.bands[b.band_code] = {**b.model_dump(), "key": key.hex(), "activated_at": time.time(),
                                "advice_desk": nearest_desk(b.kiosk_zone)}
    store.log("band", f"Band {b.band_code} issued at {ZONES[b.kiosk_zone]['label']} kiosk")
    return {"band_code": b.band_code, "advice": "If separated, go to the nearest help desk: "
            f"{store.bands[b.band_code]['advice_desk']}"}


@app.post("/api/v1/reports", tags=["lost persons (PS-03)"])
async def report_missing(r: Report):
    """Family reports a missing person at a help desk, by IVR (missed call) or via a volunteer.
    With a band code this starts the last-seen search and switches the band to LOST."""
    case_id = f"case-{len(store.cases) + 1:03d}"
    case = {"case": case_id, "band_code": r.band_code, "description": r.description,
            "channel": r.channel, "reported_at": time.time(), "reporter_desk": r.desk,
            "status": "searching" if r.band_code else "reported (no band)", "trail": [],
            "last_seen_zone": None, "designated_desk": None, "family_zone_hint": None,
            "exit_alert": False, "simulated": False}
    band = store.bands.get(r.band_code) if r.band_code else None
    if r.band_code and not band:
        raise HTTPException(404, "unknown band code")
    if band:
        case["description"] = case["description"] or band["description"]
        case["contact"] = band["contact"]
    store.cases[case_id] = case
    store.log("report", f"{case_id} reported via {r.channel}" + (f" (band {r.band_code})" if band else ""), case_id)
    if band:
        await start_search(case, band)
    return case


@app.post("/api/v1/found", tags=["lost persons (PS-03)"])
def found(f: Found):
    """A volunteer or police post scans a band. Links instantly to an open case (or opens one)
    and notifies the registered contact."""
    band = store.bands.get(f.band_code)
    if not band:
        raise HTTPException(404, "unknown band code")
    case = next((c for c in store.cases.values() if c["band_code"] == f.band_code
                 and c["status"] not in ("reunited", "cancelled")), None)
    if case is None:
        case = new_case(f.band_code, "found", "found")
    case["status"] = f"at {f.desk}"
    case["found_at_desk"] = f.desk
    store.log("notify", f"SMS/call to {band['contact']} ({band['contact_type']}): "
              f"{band['person']} is safe at {f.desk}", case["case"])
    store.send(f"CLEARCMD {current_id(band)}")
    return case


@app.post("/api/v1/ivr/missed-call", tags=["lost persons (PS-03)"])
def missed_call(caller: str):
    """Missed call from any phone (feature phones included): queue an IVR call-back
    that asks for the band code or a description (stub for the demo)."""
    store.log("ivr", f"Missed call from {caller[:-4] + 'XXXX'}: IVR call-back queued")
    return {"queued": True}


@app.get("/api/v1/cases", tags=["lost persons (PS-03)"])
def cases():
    return list(store.cases.values())


@app.post("/api/v1/cases/{case_id}/reunited", tags=["lost persons (PS-03)"])
def reunited(case_id: str):
    if case_id not in store.cases:
        raise HTTPException(404, "no such case")
    c = store.cases[case_id]
    c["status"] = "reunited"
    if c.get("band_code") in store.bands:
        store.send(f"CLEARCMD {current_id(store.bands[c['band_code']])}")
    return c


@app.websocket("/ws")
async def ws(socket: WebSocket):
    await socket.accept()
    store.clients.add(socket)
    try:
        while True:
            await socket.receive_text()
    except WebSocketDisconnect:
        store.clients.discard(socket)


async def broadcast():
    msg = json.dumps(store.snapshot())
    for c in list(store.clients):
        try:
            await c.send_text(msg)
        except Exception:
            store.clients.discard(c)


# ---------------- PS-03 logic ----------------
def band_slot_now(band):
    return int((time.time() - band["activated_at"]) // ROTATE_S)


def current_id(band):
    return tag_id(bytes.fromhex(band["key"]), band_slot_now(band))


def recent_ids(band):
    """IDs the band used in the last SEARCH_WINDOW_S (computed from the registry key)."""
    cur = band_slot_now(band)
    first = max(0, cur - SEARCH_WINDOW_S // ROTATE_S)
    return {tag_id(bytes.fromhex(band["key"]), s): s for s in range(first, cur + 1)}


def case_for_id(id_hex):
    for c in store.cases.values():
        band = store.bands.get(c.get("band_code"))
        if band and c["status"] not in ("reunited", "cancelled") and id_hex in recent_ids(band):
            return c
    return None


def band_for_id(id_hex):
    for code, band in store.bands.items():
        if id_hex in recent_ids(band):
            return code
    return None


def new_case(band_code, channel, status):
    band = store.bands[band_code]
    case_id = f"case-{len(store.cases) + 1:03d}"
    case = {"case": case_id, "band_code": band_code, "description": band["description"],
            "contact": band["contact"], "channel": channel, "reported_at": time.time(),
            "reporter_desk": None, "status": status, "trail": [], "last_seen_zone": None,
            "designated_desk": None, "family_zone_hint": None, "exit_alert": False,
            "simulated": False, "actions": []}
    store.cases[case_id] = case
    return case


def flag_seen(band_code, zone, rssi, now=None):
    """Stage 1: a band reports 'possible separation'. Watch silently; no humans yet."""
    now = now or time.time()
    p = store.pending_flags.setdefault(band_code, {"first": now, "zone": zone})
    p.update(last=now, live_zone=zone, rssi=rssi)


def check_pending(now=None):
    """Stage 2: escalate flags that persisted T_CONFIRM_S; drop flags that stopped
    (the band re-acquired its companions)."""
    now = now or time.time()
    for code, p in list(store.pending_flags.items()):
        if now - p["last"] > FLAG_GONE_S:
            store.pending_flags.pop(code)
            if p.get("case"):
                c = store.cases[p["case"]]
                if c["status"] == "auto-flagged":
                    c["status"] = "cancelled"
                    store.log("auto", f"{c['case']} cancelled: band is with its family again", c["case"])
            continue
        if not p.get("case") and now - p["first"] >= T_CONFIRM_S:
            escalate(code, p)


def escalate(band_code, p):
    band = store.bands[band_code]
    case = new_case(band_code, "auto (band)", "auto-flagged")
    case["reported_at"] = p["first"]
    p["case"] = case["case"]
    add_sighting(case, p["live_zone"], 0, p.get("rssi", -70), "live")
    case["separation_zone"] = p["zone"]
    case["designated_desk"] = nearest_desk(p["zone"])
    child_station = nearest_desk(p["live_zone"])
    acts = [f"{child_station}: volunteer dispatched to {ZONES[p['live_zone']]['label']} (band's live zone)",
            "All nodes: watch for the family's learned devices",
            f"Child to be brought to {case['designated_desk']} (nearest to where separation happened)"]
    case["actions"] = acts
    for a in acts:
        store.log("auto", a, case["case"])
    store.send(f"LOSTCMD {current_id(band)}")


def family_seen(zone, age_s):
    """A learned family device was heard: the station nearest the PARENTS tells
    them where to go."""
    for c in store.cases.values():
        if c["status"] in ("reunited", "cancelled"):
            continue
        c["family_zone_hint"] = {"zone": zone, "label": ZONES.get(zone, {}).get("label", zone), "age_s": age_s}
        if c["designated_desk"] and not c.get("parents_told"):
            c["parents_told"] = True
            msg = (f"{nearest_desk(zone)}: staff + zoned PA in {ZONES.get(zone, {}).get('label', zone)}, "
                   f"tell the family to go to {c['designated_desk']}")
            c.setdefault("actions", []).append(msg)
            store.log("auto", msg, c["case"])


def add_sighting(case, zone, age_s, rssi, source):
    t = time.time() - age_s
    case["trail"].append({"zone": zone, "label": ZONES.get(zone, {}).get("label", zone),
                          "time": t, "rssi": rssi, "source": source})
    case["trail"].sort(key=lambda s: s["time"])
    last = case["trail"][-1]
    case["last_seen_zone"] = last["zone"]
    case["last_seen_label"] = last["label"]
    case["last_seen_time"] = last["time"]
    if case["designated_desk"] is None or source == "buffer":
        # separation point = where the band was last seen before the report
        before = [s for s in case["trail"] if s["time"] <= case["reported_at"]] or case["trail"]
        case["separation_zone"] = before[-1]["zone"]
        case["designated_desk"] = nearest_desk(case["separation_zone"])
    if case["status"] == "searching":
        case["status"] = "located"
    if zone in EXIT_ZONES and source == "live" and not case["exit_alert"]:
        case["exit_alert"] = True
        store.log("exit", f"EXIT WATCH: {case['case']} band seen at {last['label']}: stop and check", case["case"])


async def start_search(case, band):
    ids = list(recent_ids(band))
    if store.ser is not None:
        store.send("FIND " + " ".join(ids[-64:]))
        store.send(f"LOSTCMD {current_id(band)}")
        store.log("search", f"Searching {len(ids)} rotating IDs across nodes; band set to LOST", case["case"])
    else:
        # playback mode: scripted trail so the dashboard flow can be shown without boards
        case["simulated"] = True
        for zone, age in [("C1", 1500), ("C2", 1100), ("C2", 700), ("X", 240)]:
            add_sighting(case, zone, age, -70, "buffer")
        store.log("search", "Simulated trail (no hardware connected)", case["case"])


def node_label(n):
    z = HW_NODE_ZONE.get(n)
    return f"node {n}" + (f" ({ZONES[z]['label']})" if z else "")


def node_down_report(reporter, down):
    """A node holds the last alarm of a silent successor (firmware fail-safe): tell the ICCC once
    per outage, and again when the silent node is heard (or cleared) again."""
    for n in down:
        holders = store.nodes_down.setdefault(n, set())
        if not holders:
            store.log("node", f"NODE DOWN: {node_label(n)} is silent; {node_label(reporter)} keeps its last "
                              f"alarm until it is back or cleared. Send staff to check it.")
        holders.add(reporter)
    for n in [k for k, v in store.nodes_down.items() if reporter in v and k not in down]:
        store.nodes_down[n].discard(reporter)
        if not store.nodes_down[n]:
            del store.nodes_down[n]
            store.log("node", f"{node_label(n)} back in service (or cleared by an operator)")


def handle_line(line):
    try:
        m = json.loads(line)
    except json.JSONDecodeError:
        return
    t = m.get("type")
    if t == "node":
        z = HW_NODE_ZONE.get(m["node"])
        if z:
            store.set_zone(z, m["rho"], m["A"], STATE_NAMES[m["state"]])
        for n in m.get("nbrs", []):
            nz = HW_NODE_ZONE.get(n["n"])
            if nz and n.get("age_ms", 1e9) < 5000:
                store.set_zone(nz, n["rho"], n["A"], STATE_NAMES[n["s"]])
        st = {z: ["NORMAL", "WATCH", "CRITICAL"].index(v["state"]) for z, v in store.zones.items()}
        store.inflow_command = inflow_from_states(st)
        node_down_report(m["node"], m.get("down", []))
    elif t == "hit":                                   # answer to a FIND (node buffer)
        c = case_for_id(m["id"])
        if c:
            add_sighting(c, HW_NODE_ZONE.get(m["node"], f"node{m['node']}"), m["age_s"], m["rssi"], "buffer")
    elif t == "sighting":                              # live LOST / possible-separation beacon
        zone = HW_NODE_ZONE.get(m["node"], f"node{m['node']}")
        if m.get("flags", 0) & 2:                      # band says its co-walkers are gone
            code = band_for_id(m["id"])
            if code:
                flag_seen(code, zone, m["rssi"])
        c = case_for_id(m["id"])
        if c:
            add_sighting(c, zone, 0, m["rssi"], "live")
    elif t == "family":                                # a learned family device heard at a node
        family_seen(HW_NODE_ZONE.get(m["node"], f"node{m['node']}"), m["age_s"])


def inflow_from_states(states):
    worst = max(states.get("R2", 0), states.get("G", 0))
    return ["OPEN", "HOLD", "CLOSE"][worst]


# ---------------- data sources ----------------
def load_rows(scenario):
    path = ROOT / "sim" / "results" / f"{scenario}_uncontrolled.csv"
    if not path.exists():
        raise SystemExit(f"{path} not found - run: python sim/ghat_scenario.py --scenario {scenario}")
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


async def playback(scenario, speed):
    rows = load_rows(scenario)
    names = gs.NODE_NAMES
    idx = {k: i for i, k in enumerate(names)}
    adj = np.zeros((len(names), len(names)), dtype=int)
    for a, b in gs.LINKS:
        adj[idx[a], idx[b]] = adj[idx[b], idx[a]] = 1
    store.source = f"playback:{scenario}"
    while True:
        net = QSNetwork(adj, QSParams())
        for r in rows:
            rho = np.array([r[f"rho_{k}"] for k in names])
            states, levels = net.step(rho, np.ones_like(adj, dtype=bool))
            store.sim_time = r["t"]
            st = {}
            for k, i in idx.items():
                store.set_zone(k, rho[i], levels[i], STATE_NAMES[int(states[i])])
                st[k] = int(states[i])
            store.inflow_command = inflow_from_states(st)
            await broadcast()
            await asyncio.sleep(1.0 / speed)


async def serial_reader(port, hil_scenario=None):
    import serial
    store.ser = serial.Serial(port, 115200, timeout=0.05)
    store.source = f"serial:{port}" + (f" + hil:{hil_scenario}" if hil_scenario else "")
    rows = load_rows(hil_scenario) if hil_scenario else None
    i, next_push, buf = 0, 0.0, b""
    while True:
        if rows and time.time() >= next_push:
            r = rows[i % len(rows)]
            store.sim_time = r["t"]
            for zone_name, node in ZONE_HW_NODE.items():
                store.send(f"RHO {node} {r[f'rho_{zone_name}']:.3f}")
            i += 1
            next_push = time.time() + 1.0
        buf += store.ser.read(4096)
        while b"\n" in buf:
            line, buf = buf.split(b"\n", 1)
            handle_line(line.decode(errors="ignore").strip())
        check_pending()
        await broadcast()
        await asyncio.sleep(0.1)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--mode", choices=["playback", "serial", "hil"], default="playback")
    ap.add_argument("--scenario", default="surge")
    ap.add_argument("--port", default=None)
    ap.add_argument("--speed", type=float, default=5.0, help="playback: simulated seconds per real second")
    ap.add_argument("--host", default="127.0.0.1")
    ap.add_argument("--http-port", type=int, default=8000)
    ap.add_argument("--confirm-s", type=int, default=T_CONFIRM_S,
                    help="auto-flag confirmation delay (shorten for a live demo, e.g. 30)")
    args = ap.parse_args()
    globals()["T_CONFIRM_S"] = args.confirm_s
    store.mode = args.mode
    # demo band: its key is the one flashed into the tag firmware; it is activated when
    # the server starts (power the tag on at the same moment)
    issue_band(BandRegistration(band_code="PK-4821", description="Girl, ~6 yrs, red frock, speaks Telugu",
                                contact="+91-XXXXX-X4021", contact_type="group leader (village bus)"), DEMO_KEY)

    @app.on_event("startup")
    async def start_source():
        if args.mode == "playback":
            asyncio.create_task(playback(args.scenario, args.speed))
        else:
            if not args.port:
                raise SystemExit("--port is required for serial/hil mode (e.g. COM5)")
            asyncio.create_task(serial_reader(args.port, args.scenario if args.mode == "hil" else None))

    uvicorn.run(app, host=args.host, port=args.http_port)


if __name__ == "__main__":
    main()
