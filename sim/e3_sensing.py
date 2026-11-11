"""E16: the JuPedSim compartment test (E3) with realistic sensing.

E3 gave every node the true density of its zone. Here the route-QS controller of E3
(variant 5: route QS, 30 % link loss, no backhaul) reads densities through the sensing layer
of sensing.py instead, per zone (R1, R2, G, C1, C2, X):
  line counters  in/out crossings of the zone (from agent zone membership, ghat_scenario.run),
                 biased per counter (beta ~ N(0, count_bias_sd)) and noisy
  density reading every obs_every_s (staggered), z = rho (1 + gamma)(1 + eps) + noise, dropouts
  estimator      sensing.kf_step, the same 2-state Kalman filter as at site scale and in the firmware
Set-ups: truth (must reproduce E3 variant 5), counters only (5 % bias, dead reckoning), the
realistic layout (only the exit corridor X is narrow enough to line-count; camera every 10 s on
every zone), and the same with RF attenuation instead of a camera.
Scenarios: surge and blocked exit, seeds 1-5 (as E3).

Writes results/e16/e16_runs.jsonl (resumable) and results/e16_e3_sensing_summary.csv.
Usage:  python e3_sensing.py [--workers 4] [--seeds 1 2 3 4 5]
"""

import argparse
import csv
import json
from concurrent.futures import ProcessPoolExecutor, as_completed
from dataclasses import replace
from pathlib import Path

import numpy as np

import ghat_scenario as gs
import qs_alert as qa
from qs_core import QSParams
from sensing import SensingConfig, kf_step

RES = Path(__file__).resolve().parent / "results"
LOG = RES / "e16" / "e16_runs.jsonl"
ZONES = gs.NODE_NAMES
CONFIGS = {
    "truth": SensingConfig(name="truth", truth=True),
    "counters-5%": SensingConfig(name="counters-5%", counters="all", density="none", count_bias_sd=0.05),
    "narrow-2%+cam10": SensingConfig(name="narrow-2%+cam10", counters="narrow", density="all",
                                     count_bias_sd=0.02, obs_every_s=10),
    "narrow-2%+rf10": SensingConfig(name="narrow-2%+rf10", counters="narrow", density="all", count_bias_sd=0.02,
                                    obs_every_s=10, obs_rel_sd=0.25, obs_bias_sd=0.10, obs_dropout=0.2),
}
NARROW = {"X"}     # the 2 m exit corridor; every other zone has a boundary at least 8 m wide


class SensedController(qa.Controller):
    """qs_alert.Controller reading densities through the sensing layer."""

    def __init__(self, method, cfg: SensingConfig, seed):
        super().__init__(method)
        self.cfg = cfg
        zones = gs.Layout().build()["zones"]
        self.area = np.array([zones[k].area for k in ZONES])
        Z = len(ZONES)
        r = np.random.default_rng([seed, 16])
        self.r = r
        if cfg.counters == "all":
            self.counted = np.ones(Z, bool)
        elif cfg.counters == "narrow":
            self.counted = np.array([k in NARROW for k in ZONES])
        else:
            self.counted = np.zeros(Z, bool)
        self.observed = np.ones(Z, bool) if cfg.density == "all" else np.zeros(Z, bool)
        self.blind = ~self.counted & ~self.observed
        self.b_in = r.normal(0, cfg.count_bias_sd, Z)
        self.b_out = r.normal(0, cfg.count_bias_sd, Z)
        self.gamma = r.normal(cfg.obs_bias_mean, cfg.obs_bias_sd, Z)
        self.phase = r.integers(0, max(1, cfg.obs_every_s), Z)
        self.state = (np.zeros(Z), np.zeros(Z), np.full(Z, 1e-4), np.zeros(Z), np.full(Z, (5 * cfg.b_walk_sd) ** 2))
        self.sq_err, self.n_err, self.max_err_dense = 0.0, 0, 0.0

    def _count(self, true, beta):
        c = true * (1 + beta) + self.r.normal(0, 1, true.shape) * self.cfg.count_noise * np.sqrt(true)
        return np.where(self.counted, np.maximum(c, 0.0), 0.0)

    def __call__(self, t, row):
        cfg = self.cfg
        true = qa.rho_vec(row)
        if cfg.truth:
            est = true
        else:
            tin = np.array([row[f"in_{k}"] for k in ZONES], float)
            tout = np.array([row[f"out_{k}"] for k in ZONES], float)
            cin, cout = self._count(tin, self.b_in), self._count(tout, self.b_out)
            due = self.observed & ((int(t) - self.phase) % max(1, cfg.obs_every_s) == 0)
            if cfg.obs_dropout > 0:
                due &= self.r.random(len(ZONES)) >= cfg.obs_dropout
            z = np.zeros(len(ZONES))
            if due.any():
                z = np.maximum(true * (1 + self.gamma) * (1 + self.r.normal(0, cfg.obs_rel_sd, len(ZONES)))
                               + self.r.normal(0, cfg.obs_abs_sd, len(ZONES)), 0.0)
            self.state = kf_step(self.state, cin, cout, self.area, self.counted, due, z, self.blind, cfg)
            est = self.state[0]
            e = est - true
            self.sq_err += float(np.sum(e ** 2))
            self.n_err += len(e)
            dense = true >= 2.0
            if dense.any():
                self.max_err_dense = max(self.max_err_dense, float(np.abs(e[dense]).max()))
        st = self.method(t, est)
        self.log.append(st.copy())
        worst = max(st[i] for i in qa.ROAD_NODES)
        if worst >= qa.CRITICAL:
            return gs.CLOSE
        if worst >= qa.WATCH:
            return gs.HOLD
        return gs.OPEN


def run_one(task):
    scenario, cfg_name, seed = task
    # E3 variant 5 exactly: route QS, 30 % link loss, rng [5, seed]
    method = qa.make_method("route-qs", QSParams(), np.random.default_rng([5, seed]), link_loss=0.3)
    ctrl = SensedController(method, CONFIGS[cfg_name], seed)
    rows, _ = gs.run(replace(gs.SCENARIOS[scenario], seed=seed), controller=ctrl)
    out = {"scenario": scenario, "sensing": cfg_name, "seed": seed, **qa.outcome(rows),
           "est_rmse": round((ctrl.sq_err / ctrl.n_err) ** 0.5, 3) if ctrl.n_err else 0.0,
           "est_max_abs_err_dense": round(ctrl.max_err_dense, 2)}
    print(json.dumps(out), flush=True)
    return out


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=4)
    ap.add_argument("--seeds", nargs="+", type=int, default=[1, 2, 3, 4, 5])
    ap.add_argument("--scenarios", nargs="+", default=["surge", "blocked"])
    args = ap.parse_args()
    LOG.parent.mkdir(parents=True, exist_ok=True)
    rows = [json.loads(x) for x in LOG.read_text().splitlines()] if LOG.exists() else []
    have = {(r["scenario"], r["sensing"], r["seed"]) for r in rows}
    # slowest first: counters-only and RF runs may jam
    order = ["counters-5%", "narrow-2%+rf10", "narrow-2%+cam10", "truth"]
    tasks = [(s, c, k) for c in order for s in args.scenarios for k in args.seeds if (s, c, k) not in have]
    print(f"{len(tasks)} runs to do", flush=True)
    with ProcessPoolExecutor(max_workers=args.workers) as ex, open(LOG, "a") as f:
        for fut in as_completed([ex.submit(run_one, t) for t in tasks]):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    # check: truth must reproduce E3 variant 5 (surge seeds in e3_surge_seeds.csv)
    e3 = RES / "e3_surge_seeds.csv"
    if e3.exists():
        ref = {int(r["seed"]): r for r in csv.DictReader(open(e3)) if r.get("variant_id") == "5"}
        for r in rows:
            if r["scenario"] == "surge" and r["sensing"] == "truth" and r["seed"] in ref:
                same = abs(r["peak_core_density"] - float(ref[r["seed"]]["peak_core_density"])) < 1e-6 and \
                    r["seconds_above_4"] == int(float(ref[r["seed"]]["seconds_above_4"]))
                print(f"CHECK truth seed {r['seed']} reproduces E3 v5: {same}", flush=True)
    summ = []
    for s in args.scenarios:
        for c in CONFIGS:
            rs = [r for r in rows if r["scenario"] == s and r["sensing"] == c]
            if not rs:
                continue
            summ.append({"scenario": s, "sensing": c, "seeds": len(rs),
                         "seeds_never_above_4": sum(r["seconds_above_4"] == 0 for r in rs),
                         "s_above_4_mean": round(float(np.mean([r["seconds_above_4"] for r in rs])), 1),
                         "s_above_4_max": max(r["seconds_above_4"] for r in rs),
                         "peak_mean": round(float(np.mean([r["peak_core_density"] for r in rs])), 2),
                         "peak_max": round(max(r["peak_core_density"] for r in rs), 2),
                         "diverted_mean": round(float(np.mean([r["pilgrims_diverted"] for r in rs]))),
                         "through_gate_mean": round(float(np.mean([r["pilgrims_through_gate"] for r in rs]))),
                         "est_rmse_mean": round(float(np.mean([r["est_rmse"] for r in rs])), 3),
                         "est_max_abs_err_dense_max": max(r["est_max_abs_err_dense"] for r in rs)})
    with open(RES / "e16_e3_sensing_summary.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(summ[0]))
        w.writeheader()
        w.writerows(summ)
    for r in summ:
        print(json.dumps(r), flush=True)


if __name__ == "__main__":
    main()
