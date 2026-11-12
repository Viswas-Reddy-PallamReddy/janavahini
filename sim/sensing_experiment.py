"""E12: does route-graph QS survive realistic sensing? (site scale, closed loop)

E8 gave every node the true density. E12 reruns the same 20 scenario draws (seeds 0-19,
same link-loss streams) with the nodes reading their density through the sensing layer
of sensing.py: biased and noisy line counters, an absolute density reading (edge camera
or RF attenuation) and a 2-state Kalman filter per node. The configurations are
sensing.CONFIGS; "truth" must reproduce E8 exactly (checked).

Methods: route-qs (the mesh alone, 30 % link loss) and hybrid (ICCC + route-qs, backhaul
up). The ICCC sees the same sensed densities as the boards.

Outputs
  results/e12/e12_runs.jsonl            one line per run (resumable)
  results/e12_sensing_seeds.csv         every run
  results/e12_sensing_summary.csv       per scenario x sensing x method: draws safe everywhere,
                                        draws with needless holds, mean/worst peak, estimate error

Usage:  python sensing_experiment.py [--workers 6] [--seeds 0 1 ... 19] [--configs truth narrow-2%+cam10 ...]
                                     [--methods route-qs] [--scenarios site-surge] [--tag sweep]
  --tag X writes results/e12_X_seeds.csv and results/e12_X_summary.csv (all runs share the jsonl).
  E12b (sweep of the density reading):  --tag sweep --methods route-qs --scenarios site-surge
                                        --configs narrow-2%+cam10 <sensing.SWEEP names>
  E12d (100 draws):                     --tag draws100 --seeds 0..99 --configs truth narrow-2%+cam10
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

from sensing import ALL as CONFIGS
from sensing import CONFIGS as MAIN_CONFIGS
from site_experiment import SCENARIOS, hotspots, run

RES = Path(__file__).resolve().parent / "results"
METHODS = ["route-qs", "hybrid"]


def _task(scenario, method, seed, cfg):
    return (scenario, method, seed, None, {"sensing": CONFIGS[cfg].as_dict(), "label": cfg})


def run_task(task):
    """Find the hotspots inside the worker (one uncontrolled run), then run."""
    scenario, method, seed, _, extra = task
    return run((scenario, method, seed, hotspots(scenario, seed), extra))


def summary(rows):
    out = []
    present = [c for c in CONFIGS if any(r["sensing"] == c for r in rows)]
    for s in SCENARIOS:
        for cfg in present:
            for m in METHODS:
                rs = [r for r in rows if r["scenario"] == s and r["sensing"] == cfg and r["method"] == m]
                if not rs:
                    continue
                peak = np.array([r["peak_any_density"] for r in rs])
                row = {"scenario": s, "sensing": cfg, "method": m, "runs": len(rs),
                       "runs_safe_everywhere": sum(r["any_s_above_4"] == 0 for r in rs),
                       "runs_with_needless_holds": sum(r["needless_hold_node_s"] > 0 for r in rs),
                       "any_s_above_4_mean": round(float(np.mean([r["any_s_above_4"] for r in rs])), 1),
                       "any_s_above_4_max": int(max(r["any_s_above_4"] for r in rs)),
                       "peak_any_mean": round(float(peak.mean()), 2), "peak_any_max": round(float(peak.max()), 2),
                       "held_node_s_mean": round(float(np.mean([r["held_node_s"] for r in rs])), 0),
                       "needless_pct_mean": round(float(np.mean([r["needless_pct"] for r in rs])), 1),
                       "exited_mean": round(float(np.mean([r["exited"] for r in rs])), 0),
                       "est_rmse_comp_mean": round(float(np.mean([r["est_rmse_comp"] for r in rs])), 3),
                       "est_max_abs_err_dense_max": round(float(max(r["est_max_abs_err_dense"] for r in rs)), 2),
                       "blind_nodes": rs[0]["blind_nodes"], "counted_nodes": rs[0]["counted_nodes"]}
                out.append(row)
    return out


def _write(rows, path):
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    from concurrent.futures import ProcessPoolExecutor, as_completed
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seeds", nargs="+", type=int, default=list(range(20)))
    ap.add_argument("--configs", nargs="+", default=list(MAIN_CONFIGS))
    ap.add_argument("--methods", nargs="+", default=METHODS)
    ap.add_argument("--scenarios", nargs="+", default=list(SCENARIOS))
    ap.add_argument("--tag", default="sensing")
    args = ap.parse_args()
    log = RES / "e12" / "e12_runs.jsonl"
    log.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
    have = {(r["scenario"], r["sensing"], r["method"], r["seed"]) for r in rows}
    tasks = [_task(s, m, k, c) for k in args.seeds for c in args.configs for s in args.scenarios
             for m in args.methods if (s, c, m, k) not in have]
    print(f"{len(tasks)} runs to do ({len(have)} already done)", flush=True)
    with open(log, "a") as f, ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
        for fut in as_completed([ex.submit(run_task, t) for t in tasks]):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    rows = [r for r in rows if r["seed"] in args.seeds and r["sensing"] in args.configs
            and r["method"] in args.methods and r["scenario"] in args.scenarios]
    rows.sort(key=lambda r: (r["scenario"], list(CONFIGS).index(r["sensing"]), METHODS.index(r["method"]), r["seed"]))
    _write(rows, RES / f"e12_{args.tag}_seeds.csv")
    summ = summary(rows)
    _write(summ, RES / f"e12_{args.tag}_summary.csv")
    for r in summ:
        print(json.dumps({k: r[k] for k in ["scenario", "sensing", "method", "runs", "runs_safe_everywhere",
                                            "runs_with_needless_holds", "peak_any_max", "est_rmse_comp_mean",
                                            "est_max_abs_err_dense_max"]}), flush=True)


if __name__ == "__main__":
    main()
