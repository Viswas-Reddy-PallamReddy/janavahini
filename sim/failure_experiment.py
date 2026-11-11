"""E10: node (board) failures at critical places, closed loop, 68-compartment site.

For each scenario draw (site_experiment.draw_scenario, seeds 0-4) the worst place
when nothing is controlled is found (the hotspot compartment), and boards die at
the places that protect it:
  compartment   the hotspot compartment's own board
  gate          the gate in front of it
  junction      the junction feeding that gate
  road          the approach road feeding that junction
  surge road    the road from the surging Nagara (surge scenario)
  gate+junction both at once
  random 5/10/20 %  that share of all boards, anywhere
Timing: at the first alert of a failure-free run (all cases); gate and junction also
2 min before it and 5 min after it.

A dead board sends nothing and its marshals get no signal (they let pilgrims through).
Firmware options compared (firmware/include/pk_qs.h, mirrored in qs_core.RouteParams):
  none           route QS as modelled in E1-E8
  hold           a silent successor's last alarm is kept until it is heard again
  backup         also hear each successor's successors (2-hop), used while it is silent
  both           hold + backup
  both+escalate  both, and a held successor counts as CRITICAL (its crowd may still be growing
                 unseen); added after "both" left the hotspot's own dead board unprotected
  centralised    the ICCC with its network up, for reference (a dead board = no reading)

Writes results/e10/e10_runs.jsonl (resumable), results/e10_failures.csv, results/e10_summary.csv.

Usage:  python failure_experiment.py [--workers 2] [--seeds 0 1 2 3 4]
"""

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

import site_experiment as se
from qs_core import ACT_METER
from route_graph import synthetic_site
from site_flow import SiteFlow

RES = Path(__file__).resolve().parent / "results"
LOG = RES / "e10" / "e10_runs.jsonl"
OPTIONS = {
    "none": ("route-qs", {}),
    "hold": ("route-qs", {"failsafe_s": np.inf}),
    "backup": ("route-qs", {"backup": True}),
    "both": ("route-qs", {"failsafe_s": np.inf, "backup": True}),
    "both+escalate": ("route-qs", {"failsafe_s": np.inf, "backup": True, "failsafe_escalate": True}),
    "centralised": ("centralised", None),
}


def pred(g, x):
    """The node sending the largest share of its pilgrims to x."""
    u, v, s, _ = g.edge_arrays()
    e = np.nonzero(v == x)[0]
    return int(u[e[np.argmax(s[e])]])


def critical_places(g, scenario, seed):
    """Hotspot compartment of the uncontrolled run and the boards that protect it."""
    sc = se.draw_scenario(scenario, seed)
    flow = SiteFlow(g, sc)
    peak = np.zeros(g.n)
    for t in range(sc.duration_s):
        flow.step(t, np.ones(g.n))
        peak = np.maximum(peak, flow.rho)
    comps = flow.comps
    comp = int(comps[np.argmax(peak[comps])])
    gate = pred(g, comp)
    junction = pred(g, gate)
    road = pred(g, junction)
    places = {"compartment": [comp], "gate": [gate], "junction": [junction], "road": [road],
              "gate+junction": [gate, junction]}
    if sc.blocked_comp is None:
        u, v, _, _ = g.edge_arrays()
        src = [i for i in range(g.n) if g.kind[i] == "source"][sc.surge_nagara]
        surge_road = int(v[u == src][0])
        if surge_road != road:
            places["surge road"] = [surge_road]
    return places, float(peak[comp])


def first_alert(scenario, seed, hot):
    """First second any node meters/holds in the failure-free run with failure handling on."""
    r = se.run((scenario, "route-qs fail-safe", seed, hot, {"label": "no failure", "first_act": True}))
    return r["first_act_s"]


def cases(g, scenario, seed, t_first):
    places, _ = critical_places(g, scenario, seed)
    out = []
    for name, nodes in places.items():
        out.append((name, "at first alert", [(n, t_first) for n in nodes]))
        if name in ("gate", "junction"):
            out.append((name, "2 min before", [(n, max(60, t_first - 120)) for n in nodes]))
            out.append((name, "5 min after", [(n, t_first + 300) for n in nodes]))
    for pct in (5, 10, 20):
        r = np.random.default_rng([seed, pct, 10])
        nodes = r.choice(g.n, size=int(round(pct / 100 * g.n)), replace=False)
        out.append((f"random {pct}%", "at first alert", [(int(n), t_first) for n in nodes]))
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    args = ap.parse_args()
    g = synthetic_site()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in LOG.read_text().splitlines()] if LOG.exists() else []
    have = {(r["scenario"], r["seed"], r["label"], r["option"]) for r in rows}

    tasks = []
    for scenario in ["site-surge", "site-blocked"]:
        for seed in args.seeds:
            hot = se.hotspots(scenario, seed)
            t_first = first_alert(scenario, seed, hot)
            if t_first is None:
                print(f"{scenario} seed {seed}: no alert at all, skipped", flush=True)
                continue
            for place, when, kills in cases(g, scenario, seed, t_first):
                label = f"{place} | {when}"
                for opt, (method, rp) in OPTIONS.items():
                    if (scenario, seed, label, opt) in have:
                        continue
                    extra = {"label": label, "kills": kills, "option": opt, "t_first_alert": t_first}
                    if rp:
                        extra["rp"] = rp
                    tasks.append((scenario, method, seed, hot, extra))
    print(f"{len(tasks)} runs to do ({len(have)} already done)", flush=True)
    with open(LOG, "a") as f, ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
        for fut in as_completed([ex.submit(se.run, t) for t in tasks]):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()

    rows.sort(key=lambda r: (r["scenario"], r["label"], list(OPTIONS).index(r["option"]), r["seed"]))
    keys = ["scenario", "seed", "label", "option", "t_first_alert", "killed", "peak_any_density",
            "any_s_above_4", "comp_s_above_4", "held_node_s", "needless_hold_node_s", "needless_pct", "exited",
            "nodes_reporting_down"]
    with open(RES / "e10_failures.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=keys, extrasaction="ignore")
        w.writeheader()
        w.writerows(rows)
    summ = []
    for scenario in ["site-surge", "site-blocked"]:
        labels = sorted({r["label"] for r in rows if r["scenario"] == scenario})
        for label in labels:
            for opt in OPTIONS:
                rs = [r for r in rows if r["scenario"] == scenario and r["label"] == label and r["option"] == opt]
                if not rs:
                    continue
                summ.append({"scenario": scenario, "failure": label, "option": opt, "draws": len(rs),
                             "draws_safe_everywhere": sum(r["any_s_above_4"] == 0 for r in rs),
                             "s_above_4_mean": round(float(np.mean([r["any_s_above_4"] for r in rs])), 1),
                             "s_above_4_max": max(r["any_s_above_4"] for r in rs),
                             "peak_max": max(r["peak_any_density"] for r in rs),
                             "needless_pct_mean": round(float(np.mean([r["needless_pct"] for r in rs])), 1),
                             "exited_mean": round(float(np.mean([r["exited"] for r in rs]))),
                             "nodes_reporting_down_mean": round(float(np.mean([r["nodes_reporting_down"]
                                                                               for r in rs])), 1)})
    with open(RES / "e10_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for r in summ:
        print(json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
