"""E13: firmware v4 options (site scale, closed loop).

Baseline "deployed" is the firmware of E10/E12c: route-graph QS with hold + 2-hop backup +
escalation ("unknown is not safe"). The v4 options (qs_core.RouteParams):
  +slow-release   the field light rises at once but may only drop after 120 s unchanged
  +dyn-share      relayed threats follow the shares a junction actually uses after diverting,
                  smoothed with tau = 30 s (planned shares in E8-E12)
  +dead-release   a silent successor held at CRITICAL drops to WATCH (METER, never green) after
                  180 s, only if every 2-hop backup behind it is heard and calm
  v4              all three

Part A, no failures: seeds 0-19, both scenarios, perfect sensing and the realistic layout
        (sensing.CONFIGS["narrow-2%+cam10"]).
Part B, boards die at the first alert (the E10 places and 5/10/20 % at random), realistic
        sensing, seeds 0-4: deployed vs +dead-release vs v4.
Metrics as site_experiment.run, plus light changes per lit node per hour (how often the
marshals' light switches).

Writes results/e13/e13_runs.jsonl (resumable) and results/e13_v4_summary.csv.
Usage:  python v4_experiment.py [--workers 6]
"""

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

import site_experiment as se
from failure_experiment import critical_places
from route_graph import synthetic_site
from sensing import CONFIGS

RES = Path(__file__).resolve().parent / "results"
LOG = RES / "e13" / "e13_runs.jsonl"
DEPLOYED = {"failsafe_s": np.inf, "backup": True, "failsafe_escalate": True}
OPTIONS = {
    "deployed": DEPLOYED,
    "+slow-release": {**DEPLOYED, "release_dwell_s": 120.0},
    "+dyn-share": {**DEPLOYED, "dyn_share_tau_s": 30.0},
    "+dead-release": {**DEPLOYED, "dead_release_s": 180.0},
    "v4": {**DEPLOYED, "release_dwell_s": 120.0, "dyn_share_tau_s": 30.0, "dead_release_s": 180.0},
    # added after the first E13 pass: the 120 s dwell on red->amber let held crowds upstream pass
    # 4 p/m^2; keep red->amber fast and make only amber->green slow
    "+slow-amber": {**DEPLOYED, "release_dwell_s": 120.0, "release_dwell_from": "watch"},
    "v4b": {**DEPLOYED, "release_dwell_s": 120.0, "release_dwell_from": "watch", "dyn_share_tau_s": 30.0,
            "dead_release_s": 180.0},
}
SENS = {"truth": CONFIGS["truth"].as_dict(), "narrow-2%+cam10": CONFIGS["narrow-2%+cam10"].as_dict()}
SCEN = ["site-surge", "site-blocked"]


def key(r):
    return (r["part"], r["scenario"], r["seed"], r["sensing"], r["label"], r["option"])


def run_a(task):
    scenario, seed, sens, opt = task
    hot = se.hotspots(scenario, seed)
    r = se.run((scenario, "route-qs", seed, hot,
                {"sensing": SENS[sens], "rp": OPTIONS[opt], "kills": [], "label": "no failure", "option": opt}))
    return {"part": "A", **r, "sensing": sens}


def setup_b(pair):
    """Hotspots, first alert (deployed, realistic sensing) and the board-death cases."""
    scenario, seed = pair
    g = synthetic_site()
    hot = se.hotspots(scenario, seed)
    r0 = se.run((scenario, "route-qs", seed, hot, {"sensing": SENS["narrow-2%+cam10"], "rp": DEPLOYED,
                                                   "kills": [], "label": "no failure", "option": "deployed"}))
    t1 = r0["first_act_s"]
    if t1 is None:
        return []
    places, _ = critical_places(g, scenario, seed)
    cases = [(f"board dies: {k}", [(nd, t1) for nd in v]) for k, v in places.items()]
    for pct in (5, 10, 20):
        r = np.random.default_rng([seed, pct, 10])
        nodes = r.choice(g.n, size=int(round(pct / 100 * g.n)), replace=False)
        cases.append((f"board dies: random {pct}%", [(int(nd), t1) for nd in nodes]))
    return [(scenario, seed, hot, label, kills, t1, opt) for label, kills in cases
            for opt in ("deployed", "+dead-release", "v4", "v4b")]


def run_b(task):
    scenario, seed, hot, label, kills, t1, opt = task
    r = se.run((scenario, "route-qs", seed, hot, {"sensing": SENS["narrow-2%+cam10"], "rp": OPTIONS[opt],
                                                  "kills": kills, "label": label, "option": opt,
                                                  "t_first_alert": t1}))
    return {"part": "B", **r, "sensing": "narrow-2%+cam10"}


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    args = ap.parse_args()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in LOG.read_text().splitlines()] if LOG.exists() else []
    have = {key(r) for r in rows}
    with ProcessPoolExecutor(max_workers=args.workers) as ex, open(LOG, "a") as f:
        a = [(s, k, se_, o) for k in range(20) for s in SCEN for se_ in SENS for o in OPTIONS
             if ("A", s, k, se_, "no failure", o) not in have]
        b = [t for ts in ex.map(setup_b, [(s, k) for s in SCEN for k in range(5)]) for t in ts
             if ("B", t[0], t[1], "narrow-2%+cam10", t[3], t[6]) not in have]
        print(f"part A {len(a)} runs, part B {len(b)} runs to do", flush=True)
        futs = [ex.submit(run_a, t) for t in a] + [ex.submit(run_b, t) for t in b]
        for fut in as_completed(futs):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()

    summ = []
    for part in ("A", "B"):
        for s in SCEN:
            groups = sorted({(r["sensing"], r["label"]) for r in rows if r["part"] == part and r["scenario"] == s})
            for sens, label in groups:
                for opt in OPTIONS:
                    rs = [r for r in rows if r["part"] == part and r["scenario"] == s and r["sensing"] == sens
                          and r["label"] == label and r["option"] == opt]
                    if not rs:
                        continue
                    summ.append({"part": part, "scenario": s, "sensing": sens, "case": label, "option": opt,
                                 "draws": len(rs), "draws_safe": sum(r["any_s_above_4"] == 0 for r in rs),
                                 "s_above_4_max": max(r["any_s_above_4"] for r in rs),
                                 "peak_max": max(r["peak_any_density"] for r in rs),
                                 "held_node_s_mean": round(float(np.mean([r["held_node_s"] for r in rs]))),
                                 "needless_pct_mean": round(float(np.mean([r["needless_pct"] for r in rs])), 1),
                                 "exited_mean": round(float(np.mean([r["exited"] for r in rs]))),
                                 "light_changes_per_h_mean": round(float(np.mean([r["light_changes_per_h_mean"]
                                                                                  for r in rs])), 1),
                                 "light_changes_per_h_max": max(r["light_changes_per_h_max"] for r in rs)})
    with open(RES / "e13_v4_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for r in summ:
        print(json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
