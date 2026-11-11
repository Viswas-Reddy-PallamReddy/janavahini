"""E12c: failures under realistic sensing (site scale, closed loop).

E10 killed boards while every live board still saw the true density. Here the boards
sense through the realistic layout of E12 (sensing.CONFIGS["narrow-2%+cam10"]: counters
at gates and exits, a density reading every 10 s, Kalman filter), running the deployed
firmware option of E10 ("both+escalate": hold a silent successor's alarm, 2-hop backup,
unknown is not safe). Draws: seeds 0-4 of both scenarios, as in E10.

Failures, at the first alert of the failure-free run (with the same sensing):
  board dies     the hotspot compartment, its gate, junction, approach road, gate+junction,
                 and 5 / 10 / 20 % of all boards at random (the E10 places)
  camera dies    the density sensor of the compartment, gate, junction or road fails while
                 the board runs on, with two firmware behaviours:
                   naive  counters if the node has them (dead reckoning), else blind (reports 0)
                   flag   report max(last estimate, WATCH level) until the sensor is back

Writes results/e12c/e12c_runs.jsonl (resumable) and results/e12c_sensing_failures.csv.
Usage:  python sensing_failures.py [--workers 6] [--seeds 0 1 2 3 4]
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
LOG = RES / "e12c" / "e12c_runs.jsonl"
SENSING = CONFIGS["narrow-2%+cam10"].as_dict()
DEPLOYED = {"failsafe_s": np.inf, "backup": True, "failsafe_escalate": True}


def setup(args):
    """Per (scenario, seed): hotspots, first alert under sensing, and the failure cases."""
    scenario, seed = args
    g = synthetic_site()
    hot = se.hotspots(scenario, seed)
    base = {"sensing": SENSING, "rp": DEPLOYED, "kills": [], "label": "no failure", "option": "deployed"}
    r0 = se.run((scenario, "route-qs", seed, hot, base))
    t1 = r0["first_act_s"]
    tasks = []
    if t1 is None:
        return r0, tasks
    places, _ = critical_places(g, scenario, seed)
    for name, nodes in places.items():
        tasks.append((scenario, "route-qs", seed, hot,
                      {**base, "label": f"board dies: {name}", "kills": [(n, t1) for n in nodes], "t_first_alert": t1}))
    for pct in (5, 10, 20):
        r = np.random.default_rng([seed, pct, 10])
        nodes = r.choice(g.n, size=int(round(pct / 100 * g.n)), replace=False)
        tasks.append((scenario, "route-qs", seed, hot,
                      {**base, "label": f"board dies: random {pct}%", "kills": [(int(n), t1) for n in nodes],
                       "t_first_alert": t1}))
    for name in ("compartment", "gate", "junction", "road"):
        for mode in ("naive", "flag"):
            tasks.append((scenario, "route-qs", seed, hot,
                          {**base, "label": f"camera dies: {name}", "option": mode,
                           "sensor_kills": [(n, t1, mode) for n in places[name]], "t_first_alert": t1}))
    return r0, tasks


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--seeds", nargs="+", type=int, default=[0, 1, 2, 3, 4])
    args = ap.parse_args()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in LOG.read_text().splitlines()] if LOG.exists() else []
    have = {(r["scenario"], r["seed"], r["label"], r["option"]) for r in rows}
    with ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
        pairs = [(s, k) for s in ["site-surge", "site-blocked"] for k in args.seeds]
        setups = list(ex.map(setup, pairs))
        tasks = []
        with open(LOG, "a") as f:
            for r0, ts in setups:
                if (r0["scenario"], r0["seed"], r0["label"], r0["option"]) not in have:
                    rows.append(r0)
                    f.write(json.dumps(r0) + "\n")
                tasks += [t for t in ts if (t[0], t[2], t[4]["label"], t[4]["option"]) not in have]
        print(f"{len(tasks)} runs to do", flush=True)
        with open(LOG, "a") as f:
            for fut in as_completed([ex.submit(se.run, t) for t in tasks]):
                r = fut.result()
                rows.append(r)
                f.write(json.dumps(r) + "\n")
                f.flush()

    summ = []
    for scenario in ["site-surge", "site-blocked"]:
        for label in sorted({r["label"] for r in rows if r["scenario"] == scenario}):
            for opt in ["deployed", "naive", "flag"]:
                rs = [r for r in rows if r["scenario"] == scenario and r["label"] == label and r["option"] == opt]
                if not rs:
                    continue
                summ.append({"scenario": scenario, "failure": label, "firmware": opt, "draws": len(rs),
                             "draws_safe_everywhere": sum(r["any_s_above_4"] == 0 for r in rs),
                             "s_above_4_mean": round(float(np.mean([r["any_s_above_4"] for r in rs])), 1),
                             "s_above_4_max": max(r["any_s_above_4"] for r in rs),
                             "peak_max": max(r["peak_any_density"] for r in rs),
                             "needless_pct_mean": round(float(np.mean([r["needless_pct"] for r in rs])), 1),
                             "exited_mean": round(float(np.mean([r["exited"] for r in rs])))})
    with open(RES / "e12c_sensing_failures.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for r in summ:
        print(json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
