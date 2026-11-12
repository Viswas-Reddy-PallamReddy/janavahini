"""E14: police reaction delay and partial compliance.   E15: alert threshold sensitivity.

Both use the realistic sensing layout (sensing.CONFIGS["narrow-2%+cam10"]) and the deployed
firmware (route-graph QS, hold + 2-hop backup + escalation), route QS alone (30 % link loss),
20 draws of each site scenario.

E14 (part F): the light changes at once, but the crowd responds D seconds later and only in
  part: holds and meters reach a fraction c of their effect, diversion moves c of the planned
  share change. D in {0, 30, 60, 120, 180} s, c in {1.0, 0.7, 0.5}.
E15 (part T): one alert parameter changed at a time around the defaults (qs_core.QSParams):
  theta_watch 0.5 / 0.7, theta_crit 0.75 / 0.95, delta 0.75 / 0.92, rate_horizon_s 30 / 90,
  rate_floor 1.0 / 2.0. Reported as missed dangers (draws not safe) against over-alerting
  (draws with needless holds, needless share, throughput): the operating-point trade-off.

Writes results/e14/e14_runs.jsonl (resumable), results/e14_field_summary.csv, results/e15_threshold_summary.csv.
Usage:  python field_experiment.py [--workers 6] [--parts F T] [--seeds 0 1 ... 19]
"""

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from pathlib import Path

import numpy as np

import site_experiment as se
from sensing import CONFIGS

RES = Path(__file__).resolve().parent / "results"
LOG = RES / "e14" / "e14_runs.jsonl"
SENS = CONFIGS["narrow-2%+cam10"].as_dict()
DEPLOYED = {"failsafe_s": np.inf, "backup": True, "failsafe_escalate": True}
SCEN = ["site-surge", "site-blocked"]
DELAYS = [0, 30, 60, 120, 180]
COMPLY = [1.0, 0.7, 0.5]
THRESH = {"default": {},
          "theta_watch=0.5": {"theta_watch": 0.5}, "theta_watch=0.7": {"theta_watch": 0.7},
          "theta_crit=0.75": {"theta_crit": 0.75}, "theta_crit=0.95": {"theta_crit": 0.95},
          "delta=0.75": {"delta": 0.75}, "delta=0.92": {"delta": 0.92},
          "rate_horizon=30": {"rate_horizon_s": 30}, "rate_horizon=90": {"rate_horizon_s": 90},
          "rate_floor=1.0": {"rate_floor": 1.0}, "rate_floor=2.0": {"rate_floor": 2.0}}


def tasks_for(part, seeds):
    out = []
    for s in SCEN:
        for k in seeds:
            if part == "F":
                for d in DELAYS:
                    for c in COMPLY:
                        out.append(("F", s, k, f"delay={d} comply={c}",
                                    {"field": {"delay_s": d, "comply": c, "divert_comply": c}}))
            else:
                for name, qs in THRESH.items():
                    out.append(("T", s, k, name, {"qs": qs}))
    return out


def run_task(t):
    part, scenario, seed, label, extra = t
    hot = se.hotspots(scenario, seed)
    r = se.run((scenario, "route-qs", seed, hot, {"sensing": SENS, "rp": DEPLOYED, "kills": [],
                                                  "label": label, "option": part, **extra}))
    return {"part": part, **r}


def summarise(rows, part):
    out = []
    labels = list(dict.fromkeys(r["label"] for r in rows if r["part"] == part))
    for s in SCEN:
        for lab in labels:
            rs = [r for r in rows if r["part"] == part and r["scenario"] == s and r["label"] == lab]
            if not rs:
                continue
            out.append({"scenario": s, "setting": lab, "draws": len(rs),
                        "draws_safe": sum(r["any_s_above_4"] == 0 for r in rs),
                        "draws_missed": sum(r["any_s_above_4"] > 0 for r in rs),
                        "s_above_4_max": max(r["any_s_above_4"] for r in rs),
                        "peak_max": max(r["peak_any_density"] for r in rs),
                        "draws_with_needless_holds": sum(r["needless_hold_node_s"] > 0 for r in rs),
                        "needless_pct_mean": round(float(np.mean([r["needless_pct"] for r in rs])), 1),
                        "held_node_s_mean": round(float(np.mean([r["held_node_s"] for r in rs]))),
                        "exited_mean": round(float(np.mean([r["exited"] for r in rs]))),
                        "first_act_s_mean": round(float(np.mean([r["first_act_s"] for r in rs
                                                                 if r["first_act_s"] is not None] or [np.nan])), 0),
                        "light_changes_per_h_mean": round(float(np.mean([r["light_changes_per_h_mean"]
                                                                         for r in rs])), 1)})
    return out


def _write(rows, path):
    if rows:
        with open(path, "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(rows[0]))
            w.writeheader()
            w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=6)
    ap.add_argument("--parts", nargs="+", default=["F", "T"])
    ap.add_argument("--seeds", nargs="+", type=int, default=list(range(20)))
    args = ap.parse_args()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in LOG.read_text().splitlines()] if LOG.exists() else []
    have = {(r["part"], r["scenario"], r["seed"], r["label"]) for r in rows}
    todo = [t for p in args.parts for t in tasks_for(p, args.seeds) if t[:4] not in have]
    print(f"{len(todo)} runs to do ({len(have)} already done)", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as ex, open(LOG, "a") as f:
        for fut in as_completed([ex.submit(run_task, t) for t in todo]):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    _write(summarise(rows, "F"), RES / "e14_field_summary.csv")
    _write(summarise(rows, "T"), RES / "e15_threshold_summary.csv")
    for part in ("F", "T"):
        for r in summarise(rows, part):
            print(part, json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
