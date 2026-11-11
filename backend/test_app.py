"""End-to-end checks of the PS-03 flow without hardware:
register band -> report -> node buffer hits -> trail, last-seen zone,
designated desk -> live sighting at the exit -> exit watch -> found at desk.

Run:  python test_app.py
"""

import asyncio
import json
import time

import app


def main():
    app.register_band(app.BandRegistration(band_code="PK-0001", description="Boy ~5", contact="+91-XXXXX-X1111",
                                           contact_type="feature phone", kiosk_zone="G"))
    band = app.store.bands["PK-0001"]
    band["activated_at"] = time.time() - 600          # band active for 10 min (10 slots)

    ids = app.recent_ids(band)
    assert len(ids) == 11 and len(set(ids)) == 11, "a new ID every slot"
    assert app.current_id(band) in ids

    sent = []
    app.store.ser = type("FakeSerial", (), {"write": lambda self, b: sent.append(b.decode())})()
    case = asyncio.run(app.report_missing(app.Report(band_code="PK-0001", channel="ivr")))
    assert case["status"] == "searching"
    assert any(s.startswith("FIND ") for s in sent) and any(s.startswith("LOSTCMD ") for s in sent)

    # nodes answer from their 60-min buffers: seen in the holding area 8 min ago, bathing steps 3 min ago
    old_id, new_id = list(ids)[2], list(ids)[7]
    app.handle_line(json.dumps({"type": "hit", "node": 4, "age_s": 480, "rssi": -70, "id": old_id}))
    app.handle_line(json.dumps({"type": "hit", "node": 2, "age_s": 180, "rssi": -75, "id": new_id}))
    c = app.store.cases[case["case"]]
    assert [s["zone"] for s in c["trail"]] == ["C1", "R2"]
    assert c["last_seen_zone"] == "R2" and c["status"] == "located"
    assert c["designated_desk"] == app.nearest_desk("R2")

    # a stranger's band never resolves to this case
    assert app.case_for_id("0011223344556677") is None

    # live LOST beacon heard at an exit node -> exit watch
    app.HW_NODE_ZONE[9] = "X"
    app.handle_line(json.dumps({"type": "sighting", "node": 9, "rssi": -60, "flags": 1, "id": app.current_id(band)}))
    assert c["exit_alert"] and c["last_seen_zone"] == "X"

    # volunteer scans the band at a desk -> linked instantly, contact notified, LOST cleared
    app.found(app.Found(band_code="PK-0001", desk="Desk 2 (Exit)"))
    assert c["status"] == "at Desk 2 (Exit)"
    assert any(e["kind"] == "notify" for e in app.store.events)
    assert sent[-1].startswith("CLEARCMD ")

    # ---- auto-flag: band reports its co-walkers gone; nearest station acts ----
    app.register_band(app.BandRegistration(band_code="PK-0002", description="Grandmother, 72", kiosk_zone="G"))
    b2 = app.store.bands["PK-0002"]
    t0 = time.time()
    n_cases = len(app.store.cases)
    flag = {"type": "sighting", "node": 4, "rssi": -72, "flags": 2, "id": app.current_id(b2)}
    app.handle_line(json.dumps(flag))
    app.check_pending(now=t0 + 60)
    assert len(app.store.cases) == n_cases, "stage 1 is silent: no case before T_CONFIRM_S"
    app.store.pending_flags["PK-0002"]["last"] = t0 + app.T_CONFIRM_S          # still flagging
    app.check_pending(now=t0 + app.T_CONFIRM_S + 1)
    auto = next(c for c in app.store.cases.values() if c["band_code"] == "PK-0002")
    assert auto["status"] == "auto-flagged" and auto["channel"] == "auto (band)"
    assert auto["designated_desk"] == app.nearest_desk("C1")
    assert any("volunteer dispatched" in a for a in auto["actions"])
    assert sent[-1].strip() == f"LOSTCMD {app.current_id(b2)}"
    app.handle_line(json.dumps({"type": "family", "node": 2, "age_s": 5, "rssi": -80, "hash": "a1b2c3d4"}))
    assert any("tell the family to go to" in a for a in auto["actions"])

    # a flag that stops before T_CONFIRM_S never reaches a human
    app.register_band(app.BandRegistration(band_code="PK-0003", description="Boy ~7", kiosk_zone="G"))
    b3 = app.store.bands["PK-0003"]
    app.flag_seen("PK-0003", "C2", -70, now=t0)
    app.check_pending(now=t0 + app.FLAG_GONE_S + 5)
    assert "PK-0003" not in app.store.pending_flags
    assert not any(c["band_code"] == "PK-0003" for c in app.store.cases.values())
    assert b3                                                                  # registered, never escalated

    # PS-01 node failure: R2 (node 2) holds silent C1's (node 4) alarm -> one NODE DOWN log, then back
    node = {"type": "node", "node": 2, "rho": 1.0, "A": 0.3, "state": 2, "action": 3, "nbrs": []}
    n_logs = len(app.store.events)
    for _ in range(3):
        app.handle_line(json.dumps({**node, "down": [4]}))
    assert app.store.nodes_down == {4: {2}}
    assert sum("NODE DOWN" in e["text"] for e in list(app.store.events)[n_logs:]) == 1
    app.clear_down(4)
    assert sent[-1].strip() == "CLEARDOWN 4"
    app.handle_line(json.dumps({**node, "down": []}))
    assert app.store.nodes_down == {}
    assert "back in service" in app.store.events[-1]["text"]
    print("ok")


if __name__ == "__main__":
    main()
