"""E9: software-in-the-loop (SIL) test of the ESP32 node firmware.

The node firmware's alert code (firmware/include/pk_qs.h) is compiled for the PC
together with firmware/sil/sil.cpp, which runs one virtual ESP32 per route-graph
node over a simulated radio. The same crowd traces used by the Python model drive
the virtual boards. This is the software-in-the-loop stage between the Python
model (MIL) and the real boards with a simulated sensor feed (HIL).

Tests
  equivalence   --ideal (lock-step 1 s loops, no loss): firmware vs the Python
                RouteQSNetwork, step by step (state, action, junction diversion)
  radio         realistic boards: own boot time, loop period 1.005-1.06 s with
                jitter, ESP-NOW latency 2-15 ms, 30 % message loss; 10 seeds.
                First METER/HOLD time per node vs the Python model, alerts at
                nodes that feed no hotspot, messages and airtime
  node failure  the gate (G) or the holding area (C1) dies during the surge.
                Firmware options: none; fail-safe hold (keep a silent successor's
                last alarm level for 5 min, or until it is heard again, and report
                it down); 2-hop backup (also listen to the successors' successors,
                used only while the successor in between is silent); both

Traces: compartment surge / blocked (JuPedSim, uncontrolled) and the 68-compartment
site (site_flow, uncontrolled, nominal scenarios). Open loop: the alerts do not
feed back into the crowd here (closed loop is E3/E8 in Python).

Needs a C++ compiler: on Windows `pip install ziglang` (used as `python -m ziglang c++`); on
Linux (e.g. Colab / Kaggle) the system g++. Windows Smart App Control may refuse to run a freshly
built sil.exe (unsigned); then run this script on Linux (WSL2, Colab or Kaggle): `pip install
numpy shapely` and `python esp32_sil.py` from pushkaralu/sim (the JuPedSim traces are read from results/).

Usage:  python esp32_sil.py
"""

import csv
import json
import subprocess
import sys
from pathlib import Path

import numpy as np

import ghat_scenario as gs
from qs_core import ACT_HOLD, ACT_METER, ACT_NONE, QSParams, RouteParams, RouteQSNetwork, WATCH, CRITICAL
from route_graph import compartment_graph, synthetic_site
from site_flow import SCENARIOS as SITE_SCENARIOS
from site_flow import SiteFlow, branch_density_index, divert_shares

HERE = Path(__file__).resolve().parent
RES = HERE / "results"
OUT = RES / "sil"
FW = HERE.parent / "firmware"
import os
import shutil
EXE = OUT / ("sil.exe" if os.name == "nt" else "sil")
ACT_DIVERT = 4
KIND_CODE = {"gate": 1, "junction": 2}
SEEDS = range(1, 11)
FAILURE_CONFIGS = [                       # label, SIL options
    ("none", []),
    ("hold 5 min", ["--failsafe", 300_000]),
    ("hold until restored", ["--failsafe", "forever"]),
    ("2-hop backup", ["--backup"]),
    ("2-hop backup + hold until restored", ["--backup", "--failsafe", "forever"]),
    ("2-hop backup + hold + escalate (deployed)", ["--backup", "--failsafe", "forever", "--escalate"]),
]
PROBLEM_RHO = 3.5

# ESP-NOW airtime per 25-byte broadcast at the default 1 Mbit/s: ~192 us long preamble +
# (24 MAC header + 4 FCS + 7 vendor-specific action header + 6 ESP-NOW element + 25 payload) bytes * 8 us.
AIRTIME_US = 192 + (24 + 4 + 7 + 6 + 25) * 8


def build():
    OUT.mkdir(parents=True, exist_ok=True)
    src = FW / "sil" / "sil.cpp"
    hdr = FW / "include" / "pk_qs.h"
    if EXE.exists() and EXE.stat().st_mtime > max(src.stat().st_mtime, hdr.stat().st_mtime):
        return
    cxx = ([sys.executable, "-m", "ziglang", "c++"] if os.name == "nt" or not shutil.which("g++")
           else ["g++"])
    cmd = cxx + ["-O2", "-std=c++17", "-o", str(EXE), str(src)]
    print("building:", " ".join(cmd), flush=True)
    subprocess.run(cmd, check=True)


def write_graph(g, path):
    u, v, share, walk = g.edge_arrays()
    with open(path, "w") as f:
        f.write(f"{g.n} {len(u)}\n")
        for i in range(g.n):
            f.write(f"{i} {KIND_CODE.get(g.kind[i], 0)}\n")
        for a, b, s, w in zip(u, v, share, walk):
            f.write(f"{a} {b} {s:.6f} {w:.3f}\n")


def write_trace(rho, path):
    with open(path, "w") as f:
        f.write(f"{rho.shape[0]} {rho.shape[1]}\n")
        np.savetxt(f, rho, fmt="%.5f")


_written = set()


def run_sil(name, g, rho, *args):
    gp, tp, op = OUT / f"{name}_graph.txt", OUT / f"{name}_trace.txt", OUT / f"{name}_out.txt"
    if name not in _written:                       # inputs written once per process
        write_graph(g, gp)
        write_trace(rho, tp)
        _written.add(name)
    res = subprocess.run([str(EXE), str(gp), str(tp), str(op), *map(str, args)], check=True,
                         capture_output=True, text=True)
    stats = json.loads(res.stdout)
    d = np.loadtxt(op)
    return d, stats


def per_second(d, n, T):
    """SIL tick log -> per-second state/action matrices (the last tick within each second)."""
    state = np.zeros((T, n), int)
    action = np.zeros((T, n), int)
    t = (d[:, 0] // 1000).astype(int)
    node = d[:, 1].astype(int)
    order = np.lexsort((d[:, 0], node, t))
    state[t[order], node[order]] = d[order, 2]
    action[t[order], node[order]] = d[order, 3]
    # carry forward through seconds in which a slow board had no tick
    seen = np.zeros((T, n), bool)
    seen[t, node] = True
    for k in range(1, T):
        miss = ~seen[k]
        state[k, miss] = state[k - 1, miss]
        action[k, miss] = action[k - 1, miss]
    return state, action


def python_reference(g, rho, rp=None, kills=()):
    """The Python model with perfect links, plus junction diversion fed with the same
    (message-delayed) branch densities the boards see. kills: (node, t_s) pairs; a killed
    node's last broadcast (sent just before t_s) is still heard at t_s, as on the radio."""
    T, n = rho.shape
    net = RouteQSNetwork(g, QSParams(), rp or RouteParams())
    up_all = np.ones((n, n), bool)
    u, v, base_share, _ = g.edge_arrays()
    bidx = branch_density_index(g)
    lag = np.where(bidx != v, 2, 1)                 # gate relays its compartment's density: one more hop
    kill_t = np.full(n, np.inf)
    for node, t_s in kills:
        kill_t[node] = t_s
    fh = rp is not None and (rp.failsafe_s > 0 or rp.backup)
    is_junction = np.array([k == "junction" for k in g.kind])
    state = np.zeros((T, n), int)
    action = np.zeros((T, n), int)
    for k in range(T):
        up = up_all
        dead = [d for d, t_s in kills if k >= t_s + 1]
        if dead:
            up = up_all.copy()
            up[:, dead] = False
            up[dead, :] = False
        st, _ = net.step(rho[k], up)
        act = net.action.copy()
        branch_rho = np.where(k - lag >= 0, rho[np.maximum(k - lag, 0), bidx], 0.0)
        if fh:
            # gate relaying a compartment whose board died: the gate hears the last message (sent at
            # kill_t - 1) for STALE 5 s, then reports "unknown" (firmware UNKNOWN_RHO)
            via_gate = bidx != v
            sent = np.minimum(k - 2, kill_t[bidx] - 1)          # compartment step the gate heard, relayed
            age_at_gate = (k - 1) - (sent + 1)
            known = rho[np.clip(sent, 0, T - 1).astype(int), bidx]
            gate_rho = np.where(age_at_gate <= 5, known, 99.0)
            branch_rho = np.where(via_gate & (kill_t[bidx] <= k - 1) & (k - 2 >= 0), gate_rho, branch_rho)
        share = divert_shares(g, base_share, net.edge_threat, np.where(net.edge_room_known, branch_rho, np.inf))
        diverted = np.zeros(n, bool)
        diverted[np.unique(u[share != base_share])] = True
        act[is_junction & diverted] = ACT_DIVERT
        state[k], action[k] = st, act
    return state, action


def equivalence_failures(tr):
    """Lock-step firmware vs Python with the fail-safe and 2-hop backup ON and boards killed."""
    out = []
    IDX = {k: i for i, k in enumerate(gs.NODE_NAMES)}
    g, rho = tr["compartment-surge"]
    site, srho = tr["site-surge"]
    hot = int(np.argmax(srho.max(axis=0)))                   # the worst node and the gate in front of it
    u, v, _, _ = site.edge_arrays()
    feeders = u[v == hot]
    cases = [("compartment-surge", g, rho, [(IDX["G"], 500)]),
             ("compartment-surge", g, rho, [(IDX["C1"], 500)]),
             ("compartment-surge", g, rho, [(IDX["G"], 500), (IDX["C1"], 700)]),
             ("site-surge", site, srho, [(int(f), 900) for f in feeders]),
             ("site-surge", site, srho, [(hot, 900)])]
    for (name, gg, r, kills), esc in [(c, e) for c in cases for e in (False, True)]:
        rp = RouteParams(failsafe_s=np.inf, backup=True, failsafe_escalate=esc)
        args = ["--ideal", "--backup", "--failsafe", "forever"] + (["--escalate"] if esc else [])
        for node, t_s in kills:
            args += ["--kill", f"{node}:{t_s}"]
        d, _ = run_sil(f"{name}_eqfail", gg, r, *args)
        s_fw, a_fw = per_second(d, gg.n, len(r))
        s_py, a_py = python_reference(gg, r, rp, kills)
        alive = np.ones((len(r), gg.n), bool)
        for node, t_s in kills:
            alive[t_s:, node] = False
        same = ((s_fw == s_py) & (a_fw == a_py))[alive]
        out.append({"trace": name, "killed": " + ".join(f"{gg.names[nd]}@{t}s" for nd, t in kills),
                    "options": "2-hop backup + hold until restored" + (" + escalate" if esc else ""),
                    "alive_node_seconds": int(alive.sum()),
                    "identical_pct": round(100 * float(same.mean()), 4), "differing": int((~same).sum())})
        print(out[-1], flush=True)
    return out


def first_act(action):
    """First second each node meters, holds or diverts (NaN if never)."""
    acting = action >= ACT_METER
    t = np.where(acting.any(axis=0), acting.argmax(axis=0), -1).astype(float)
    t[t < 0] = np.nan
    return t


def needed_nodes(g, rho):
    hot = np.nonzero((rho >= PROBLEM_RHO).any(axis=0))[0]
    need = np.zeros(g.n, bool)
    need[hot] = True
    for j in hot:
        for i in g.upstream(j):
            need[i] = True
    return need


def traces():
    out = {}
    g = compartment_graph()
    for sc in ["surge", "blocked"]:
        with open(RES / f"{sc}_uncontrolled.csv") as f:
            rows = list(csv.DictReader(f))
        out[f"compartment-{sc}"] = (g, np.array([[float(r[f"rho_{k}"]) for k in gs.NODE_NAMES] for r in rows]))
    site = synthetic_site()
    for sc in ["site-surge", "site-blocked"]:
        flow = SiteFlow(site, SITE_SCENARIOS[sc])
        rho = np.zeros((SITE_SCENARIOS[sc].duration_s, site.n))
        for t in range(len(rho)):
            rho[t] = flow.rho
            flow.step(t, np.ones(site.n))
        out[sc] = (site, rho)
    return out


def equivalence(name, g, rho):
    d, stats = run_sil(f"{name}_ideal", g, rho, "--ideal")
    s_fw, a_fw = per_second(d, g.n, len(rho))
    s_py, a_py = python_reference(g, rho)
    same = (s_fw == s_py) & (a_fw == a_py)
    diff = np.argwhere(~same)
    return {"trace": name, "nodes": g.n, "seconds": len(rho), "node_seconds": int(same.size),
            "identical_pct": round(100 * same.mean(), 4), "differing_node_seconds": int((~same).sum()),
            "first_difference": (f"t={diff[0][0]} s node {g.names[diff[0][1]]}" if len(diff) else ""),
            "msg_bytes": stats["msg_bytes"], "node_state_bytes": stats["node_state_bytes"]}


def radio(name, g, rho, s_py, a_py, config="as modelled", extra=()):
    T = len(rho)
    py_first = first_act(a_py)
    need = needed_nodes(g, rho)
    py_needless = int(((a_py >= ACT_METER) & ~need).sum())
    rows = []
    for seed in SEEDS:
        d, stats = run_sil(f"{name}_radio", g, rho, "--seed", seed, "--loss", 0.3, *extra)
        s_fw, a_fw = per_second(d, g.n, T)
        fw_first = first_act(a_fw)
        both = ~np.isnan(py_first) & ~np.isnan(fw_first)
        delay = fw_first[both] - py_first[both]
        acting = a_fw >= ACT_METER
        rows.append({
            "trace": name, "config": config, "seed": seed,
            "nodes_acting_python": int((~np.isnan(py_first)).sum()),
            "nodes_acting_firmware": int((~np.isnan(fw_first)).sum()),
            "missed_by_firmware": int((~np.isnan(py_first) & np.isnan(fw_first)).sum()),
            "extra_in_firmware": int((np.isnan(py_first) & ~np.isnan(fw_first)).sum()),
            "first_act_delay_median_s": float(np.median(delay)) if len(delay) else None,
            "first_act_delay_max_s": float(delay.max()) if len(delay) else None,
            "first_act_delay_min_s": float(delay.min()) if len(delay) else None,
            "action_agreement_pct": round(100 * float((np.minimum(a_fw, 3) == np.minimum(a_py, 3)).mean()), 2),
            "needless_acting_node_s": int((acting & ~need).sum()),
            "needless_acting_node_s_python": py_needless,
            "msgs_per_node_s": round(stats["sent"] / (g.n * T), 3),
            "loss_measured_pct": round(100 * stats["lost"] / max(1, stats["lost"] + stats["delivered"]), 1),
            "bytes_per_node_s": round(stats["bytes_sent"] / (g.n * T), 1),
        })
    return rows


def compartment_times(state, action, t_d):
    IDX = {k: i for i, k in enumerate(gs.NODE_NAMES)}
    road = action[:, [IDX["R2"], IDX["G"]]].max(axis=1) >= ACT_METER
    t_road = int(road.argmax()) if road.any() else None
    return t_road, (t_d - t_road) if (t_road is not None and t_d is not None) else None


def node_failure(g, rho, s_py, a_py):
    """Kill G (index 2) or C1 (index 3) 60 s after the road was first told to act."""
    import qs_alert
    IDX = {k: i for i, k in enumerate(gs.NODE_NAMES)}
    rows_csv = [{"t": float(t), **{f"rho_{k}": rho[t, i] for i, k in enumerate(gs.NODE_NAMES)}}
                for t in range(len(rho))]
    t_d = qs_alert.danger_time(rows_csv)
    t_road, _ = compartment_times(s_py, a_py, t_d)
    t_kill = t_road + 60
    out = []
    for victim in ["G", "C1"]:
        for label, opts in FAILURE_CONFIGS:
            for seed in [1, 2, 3]:
                d, _ = run_sil("compartment-surge_fail", g, rho, "--seed", seed, "--loss", 0.3,
                               *opts, "--kill", f"{IDX[victim]}:{t_kill}")
                s_fw, a_fw = per_second(d, g.n, len(rho))
                r2 = a_fw[:, IDX["R2"]]
                after = r2[t_kill:]
                # how long the road keeps acting after the failure, while the compartment is still in danger
                core = rho[:, [IDX[k] for k in ("G", "C1", "C2", "X")]].max(axis=1)
                danger = core[t_kill:] >= 3.5
                out.append({"victim": victim, "firmware_option": label, "seed": seed,
                            "killed_at_s": t_kill,
                            "danger_s_after_kill": int(danger.sum()),
                            "road_acting_s_during_danger": int(((after >= ACT_METER) & danger).sum()),
                            "road_released_at_s": (int(t_kill + np.argmax(after < ACT_METER))
                                                   if (after < ACT_METER).any() else None)})
    return out, t_kill


def write(rows, name):
    with open(RES / f"{name}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(json.dumps(rows if len(rows) < 12 else rows[:12], indent=1, default=str), flush=True)


def main():
    build()
    tr = traces()
    eq, rad = [], []
    for name, (g, rho) in tr.items():
        print(f"--- {name}: {g.n} nodes, {len(rho)} s", flush=True)
        eq.append(equivalence(name, g, rho))
        print(eq[-1], flush=True)
        s_py, a_py = python_reference(g, rho)
        rad += radio(name, g, rho, s_py, a_py)
        rad += radio(name, g, rho, s_py, a_py, "with failure handling (deployed)",
                     ["--backup", "--failsafe", "forever", "--escalate"])
        print(rad[-1], flush=True)
    write(eq, "e9_sil_equivalence")
    write(equivalence_failures(tr), "e9_sil_equivalence_failsafe")
    write(rad, "e9_sil_radio")
    summary = []
    for name, config in sorted({(r["trace"], r["config"]) for r in rad}, key=lambda x: (list(tr).index(x[0]), x[1])):
        rs = [r for r in rad if r["trace"] == name and r["config"] == config]
        summary.append({
            "trace": name, "config": config, "seeds": len(rs),
            "missed_by_firmware_total": sum(r["missed_by_firmware"] for r in rs),
            "extra_in_firmware_total": sum(r["extra_in_firmware"] for r in rs),
            "first_act_delay_median_s": float(np.median([r["first_act_delay_median_s"] for r in rs
                                                         if r["first_act_delay_median_s"] is not None])),
            "first_act_delay_worst_s": max(r["first_act_delay_max_s"] for r in rs
                                           if r["first_act_delay_max_s"] is not None),
            "first_act_earliest_s": min(r["first_act_delay_min_s"] for r in rs
                                        if r["first_act_delay_min_s"] is not None),
            "action_agreement_pct_mean": round(float(np.mean([r["action_agreement_pct"] for r in rs])), 2),
            "needless_acting_node_s_per_run": round(float(np.mean([r["needless_acting_node_s"] for r in rs])), 1),
            "needless_acting_node_s_python": rs[0]["needless_acting_node_s_python"],
            "bytes_per_node_s": rs[0]["bytes_per_node_s"],
            "airtime_per_msg_us": AIRTIME_US,
        })
    write(summary, "e9_sil_radio_summary")
    g, rho = tr["compartment-surge"]
    s_py, a_py = python_reference(g, rho)
    fail, t_kill = node_failure(g, rho, s_py, a_py)
    write(fail, "e9_sil_node_failure")


if __name__ == "__main__":
    main()
