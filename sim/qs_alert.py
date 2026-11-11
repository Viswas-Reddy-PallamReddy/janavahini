"""PS-01 experiments: quorum-sensing alerts vs baselines on the ghat scenario.

E1  alert timing (open loop): how early each method warns before the
    compartment reaches dangerous density, and whether the ROAD node (where
    police can hold inflow) is warned at all.
E2  robustness: QS with 0-50% radio link loss and the backhaul cut, against a
    centralised command-centre baseline that goes silent without backhaul.
E3  closed loop: each method controls the inflow at the road head
    (OPEN / HOLD / CLOSE). Reports peak density and seconds above 4 and 5 p/m^2.
    --e3-seeds runs several scenario seeds (arrivals, walking speeds, link loss);
    each run is its own process, saved to results/e3/ as it finishes (resumable).
E4  false alerts in normal operation.

Usage:
    python qs_alert.py --experiment all
    python qs_alert.py --experiment e1 e2       (reuse saved uncontrolled runs)
    python qs_alert.py --experiment e3 --e3-seeds 1 2 3 4 5 --workers 3
"""

import argparse
import csv
import json
from pathlib import Path

import numpy as np

import ghat_scenario as gs
from qs_core import (CRITICAL, NORMAL, WATCH, CentralisedBaseline, QSNetwork, QSParams, RouteQSNetwork,
                     ThresholdBaseline)
from route_graph import compartment_graph

RES = Path(__file__).resolve().parent / "results"
N = len(gs.NODE_NAMES)
IDX = {k: i for i, k in enumerate(gs.NODE_NAMES)}
ADJ = np.zeros((N, N), dtype=int)
for a, b in gs.LINKS:
    ADJ[IDX[a], IDX[b]] = ADJ[IDX[b], IDX[a]] = 1
UPSTREAM = {IDX[k]: [IDX[u] for u in v] for k, v in gs.UPSTREAM.items()}
DANGER = 4.0           # persons/m^2 (Green Guide moving-crowd limit)
SEVERE = 5.0
ROAD_NODES = [IDX["R2"], IDX["G"]]       # where inflow can be held
CORE_ZONES = ("G", "C1", "C2", "X")      # where a crush can happen (incl. the exit corridor)
CORE_NODES = [IDX[k] for k in CORE_ZONES]
METHODS = ["threshold", "threshold+rate", "centralised", "qs", "qs-laplacian (ablation)", "route-qs"]
ROUTE = compartment_graph()             # same node order as gs.NODE_NAMES (checked in test_route_graph)


def rho_vec(row):
    return np.array([row[f"rho_{k}"] for k in gs.NODE_NAMES])


def make_method(name, params, rng, link_loss=0.0, backhaul_cut_at=None, kill=None, failsafe=False):
    """Returns a callable (t, rho) -> states array.
    route-qs / hybrid only: kill = zone whose board dies 60 s after the road is first told to act
    (no radio, no state); failsafe = the firmware's node-failure handling (hold + 2-hop backup)."""
    if name == "threshold":
        m = ThresholdBaseline(N, params)
        return lambda t, rho: m.step(rho)[0]
    if name == "threshold+rate":
        m = ThresholdBaseline(N, params, use_rate=True)
        return lambda t, rho: m.step(rho)[0]
    if name == "centralised":
        m = CentralisedBaseline(N, params, UPSTREAM)

        def f(t, rho):
            up = backhaul_cut_at is None or t < backhaul_cut_at
            return m.step(rho, up)[0]
        return f
    if name.startswith("qs"):
        if "laplacian" in name:
            from dataclasses import replace
            params = replace(params, mode="laplacian")
        m = QSNetwork(ADJ, params)

        def f(t, rho):
            link_up = rng.random((N, N)) >= link_loss
            link_up = link_up & link_up.T
            return m.step(rho, link_up)[0]
        return f
    if name in ("route-qs", "hybrid"):
        from qs_core import RouteParams
        rp = RouteParams(failsafe_s=np.inf, backup=True) if failsafe else RouteParams()
        m = RouteQSNetwork(ROUTE, params, rp)
        cc = CentralisedBaseline(N, params, UPSTREAM) if name == "hybrid" else None
        k = {"t_kill": np.inf}

        def f(t, rho):
            link_up = rng.random((N, N)) >= link_loss
            link_up = link_up & link_up.T
            dead = kill is not None and t >= k["t_kill"]
            if dead:
                link_up[:, IDX[kill]] = False
                link_up[IDX[kill], :] = False
            st = m.step(rho, link_up)[0]
            if cc is not None:
                up = backhaul_cut_at is None or t < backhaul_cut_at
                st_cc = cc.step(rho, up)[0]
                if up:
                    st = np.maximum(st, st_cc)
            if kill is not None and np.isinf(k["t_kill"]) and max(st[i] for i in ROAD_NODES) >= WATCH:
                k["t_kill"] = t + 60
            if dead:
                st = st.copy()
                st[IDX[kill]] = NORMAL
            return st
        return f
    raise ValueError(name)


def first_time(mask, t):
    idx = np.nonzero(mask)[0]
    return float(t[idx[0]]) if len(idx) else None


def danger_time(rows, level=DANGER, sustain=10):
    """First second at which any core zone stays >= level for `sustain` s."""
    t = np.array([r["t"] for r in rows])
    core = np.array([[r[f"rho_{k}"] for k in CORE_ZONES] for r in rows]).max(axis=1)
    above = core >= level
    run = 0
    for i, a in enumerate(above):
        run = run + 1 if a else 0
        if run >= sustain:
            return float(t[i - sustain + 1])
    return None


def replay(rows, method):
    t = np.array([r["t"] for r in rows])
    states = np.array([method(ti, rho_vec(r)) for ti, r in zip(t, rows)])
    return t, states


def e1_timing(rows, params, label):
    t_d = danger_time(rows)
    out = []
    for name in METHODS:
        rng = np.random.default_rng(0)
        t, st = replay(rows, make_method(name, params, rng))
        any_crit = first_time((st[:, CORE_NODES] >= CRITICAL).any(axis=1), t)
        road_warn = first_time((st[:, ROAD_NODES] >= WATCH).any(axis=1), t)
        out.append({
            "scenario": label, "method": name, "danger_time_s": t_d,
            "first_core_CRITICAL_s": any_crit,
            "lead_time_core_s": (t_d - any_crit) if (t_d is not None and any_crit is not None) else None,
            "first_road_warning_s": road_warn,
            "lead_time_road_s": (t_d - road_warn) if (t_d is not None and road_warn is not None) else None,
        })
    return out


def _e2_seed(task):
    """One link-loss seed: first road warning time (worker re-reads the scenario CSV)."""
    name, method, loss, s = task
    rows = load_or_run(name)
    t, st = replay(rows, make_method(method, QSParams(), np.random.default_rng(100 + s), link_loss=loss))
    return first_time((st[:, ROAD_NODES] >= WATCH).any(axis=1), t)


def e2_robustness(rows, params, label, seeds=20):
    t_d = danger_time(rows)
    out = []
    losses = [0.0, 0.1, 0.3, 0.5]
    for method in ["qs", "route-qs"]:
        road = pmap(_e2_seed, [(label, method, loss, s) for loss in losses for s in range(seeds)])
        for k, loss in enumerate(losses):
            leads, warned = [], 0
            for rw in road[k * seeds:(k + 1) * seeds]:
                if rw is not None and t_d is not None:
                    warned += 1
                    leads.append(t_d - rw)
            out.append({"scenario": label, "method": f"{method} (no backhaul needed)", "link_loss": loss,
                        "road_warned_pct": 100 * warned / seeds,
                        "median_road_lead_s": float(np.median(leads)) if leads else None})
    # centralised with backhaul cut shortly before the surge builds
    rng = np.random.default_rng(0)
    t, st = replay(rows, make_method("centralised", params, rng, backhaul_cut_at=250.0))
    rw = first_time((st[:, ROAD_NODES] >= WATCH).any(axis=1), t)
    out.append({"scenario": label, "method": "centralised, backhaul cut at 250 s", "link_loss": 0.0,
                "road_warned_pct": 100.0 if rw is not None else 0.0,
                "median_road_lead_s": (t_d - rw) if (rw is not None and t_d is not None) else None})
    return out


class Controller:
    """Maps road/gate node alert states to an inflow command."""

    def __init__(self, method):
        self.method = method
        self.log = []

    def __call__(self, t, row):
        st = self.method(t, rho_vec(row))
        self.log.append(st.copy())
        worst = max(st[i] for i in ROAD_NODES)
        if worst >= CRITICAL:
            return gs.CLOSE
        if worst >= WATCH:
            return gs.HOLD
        return gs.OPEN


def outcome(rows):
    core = np.array([[r[f"rho_{k}"] for k in CORE_ZONES] for r in rows]).max(axis=1)
    return {
        "peak_core_density": float(core.max()),
        "seconds_above_4": int((core >= DANGER).sum()),
        "seconds_above_5": int((core >= SEVERE).sum()),
        "pilgrims_diverted": int(rows[-1]["diverted"]),
        "pilgrims_through_gate": int(rows[-1]["gate_cum"]),
    }


WORKERS = 2   # parallel processes; RAM on the dev machine (~0.7 GB free) is the limit, not cores

# (label, method, kwargs for make_method, rng seed); method None = no controller
E3_VARIANTS = [
    ("no system", None, {}, 0),
    ("threshold (current practice)", "threshold", {}, 0),
    ("centralised ICCC, backhaul up", "centralised", {}, 0),
    ("centralised ICCC, backhaul cut at 250 s", "centralised", {"backhaul_cut_at": 250.0}, 0),
    ("QS mesh, 30% link loss, no backhaul", "qs", {"link_loss": 0.3}, 5),
    ("route QS, 30% link loss, no backhaul", "route-qs", {"link_loss": 0.3}, 5),
    ("hybrid ICCC + route QS, backhaul up", "hybrid", {"link_loss": 0.3}, 5),
    ("hybrid, backhaul cut mid-surge (520 s)", "hybrid", {"link_loss": 0.3, "backhaul_cut_at": 520.0}, 5),
    ("route QS, gate board G dies, no failure handling", "route-qs", {"link_loss": 0.3, "kill": "G"}, 5),
    ("route QS, gate board G dies, failure handling", "route-qs",
     {"link_loss": 0.3, "kill": "G", "failsafe": True}, 5),
    ("route QS, holding-area board C1 dies, no failure handling", "route-qs", {"link_loss": 0.3, "kill": "C1"}, 5),
    ("route QS, holding-area board C1 dies, failure handling", "route-qs",
     {"link_loss": 0.3, "kill": "C1", "failsafe": True}, 5),
]


def pmap(fn, items, workers=WORKERS):
    """Order-preserving parallel map over processes (each task carries its own seed,
    so results do not depend on the number of workers)."""
    from concurrent.futures import ProcessPoolExecutor
    items = list(items)
    if workers <= 1 or len(items) <= 1:
        return [fn(x) for x in items]
    with ProcessPoolExecutor(max_workers=min(workers, len(items))) as ex:
        return list(ex.map(fn, items))


def _e3_one(task):
    scenario_name, i = task
    label, method, kwargs, seed = E3_VARIANTS[i]
    ctrl = Controller(make_method(method, QSParams(), np.random.default_rng(seed), **kwargs)) if method else None
    rows, _ = gs.run(gs.SCENARIOS[scenario_name], controller=ctrl)
    gs.save_rows(rows, RES / f"{scenario_name}_closed_{label.split(',')[0].split(' ')[0]}_{i}.csv")
    o = {"scenario": scenario_name, "variant": label, **outcome(rows)}
    print(o, flush=True)
    return o


# slowest first so the pool stays busy: a jam (no control, late control) is the slowest case
# to simulate; the centralised variant, which keeps the crowd small, is the fastest
E3_ORDER = [0, 1, 3, 4, 2]


def e3_closed_loop(scenario_name, only=None):
    idx = [i for i in E3_ORDER if only is None or i in only]
    return pmap(_e3_one, [(scenario_name, i) for i in idx])


# ---- E3 over several seeds -------------------------------------------------
E3_DIR = RES / "e3"
E3_SEED_ORDER = [0, 5, 4, 2, 6, 7, 8, 9, 10, 11]   # per seed: uncontrolled (slowest, a jam) first
E3_DEFERRED = [1, 3]              # checked against the uncontrolled run first (see e3_multi)


def _e3_tag(scenario_name, i, seed):
    return f"{scenario_name}_s{seed}_v{i}"


def _e3_controller(i, seed):
    label, method, kwargs, vseed = E3_VARIANTS[i]
    if method is None:
        return None
    return Controller(make_method(method, QSParams(), np.random.default_rng([vseed, seed]), **kwargs))


def _e3_result(scenario_name, i, seed, rows, note=""):
    return {"scenario": scenario_name, "seed": seed, "variant_id": i, "variant": E3_VARIANTS[i][0],
            **outcome(rows), "note": note}


def e3_task(scenario_name, i, seed):
    """One closed-loop run in this process; writes rows CSV + result JSON."""
    from dataclasses import replace
    sc = replace(gs.SCENARIOS[scenario_name], seed=seed)
    rows, _ = gs.run(sc, controller=_e3_controller(i, seed))
    E3_DIR.mkdir(parents=True, exist_ok=True)
    tag = _e3_tag(scenario_name, i, seed)
    gs.save_rows(rows, E3_DIR / f"{tag}.csv")
    (E3_DIR / f"{tag}.json").write_text(json.dumps(_e3_result(scenario_name, i, seed, rows)))


def _load_rows(path):
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def _never_acts(i, seed, uncontrolled_rows):
    """True if the controller replayed on the uncontrolled run never leaves OPEN.
    Then the closed-loop run is exactly the uncontrolled run: the scenario's random
    stream does not depend on the controller, and nothing differs until the first
    non-OPEN command, which never comes."""
    ctrl = _e3_controller(i, seed)
    return all(ctrl(r["t"], r) == gs.OPEN for r in uncontrolled_rows)


def e3_multi(scenario_name, seeds, workers=3, variants=None):
    """Runs every (variant, seed) as a separate process, at most `workers` at once,
    printing each result as it lands. Finished runs (JSON present) are skipped."""
    import subprocess
    import sys
    import time
    variants = set(range(len(E3_VARIANTS)) if variants is None else variants)
    E3_DIR.mkdir(parents=True, exist_ok=True)

    def done(i, s):
        return (E3_DIR / f"{_e3_tag(scenario_name, i, s)}.json").exists()

    need_unc = bool(variants & set(E3_DEFERRED))
    queue = [(i, s) for s in seeds for i in E3_SEED_ORDER if i in variants or (i == 0 and need_unc)]
    deferred = [(i, s) for s in seeds for i in E3_DEFERRED if i in variants]
    running = {}
    t0 = time.time()

    def report(i, s):
        r = json.loads((E3_DIR / f"{_e3_tag(scenario_name, i, s)}.json").read_text())
        note = f" ({r['note']})" if r["note"] else ""
        print(f"[{(time.time() - t0) / 60:6.1f} min] seed {s} | {r['variant']}: peak {r['peak_core_density']:.2f}, "
              f">=4 for {r['seconds_above_4']} s, >=5 for {r['seconds_above_5']} s, "
              f"diverted {r['pilgrims_diverted']}{note}", flush=True)

    while queue or deferred or running:
        for i, s in [d for d in deferred if done(0, d[1])]:       # uncontrolled run of that seed is ready
            deferred.remove((i, s))
            if done(i, s):
                report(i, s)
                continue
            unc = _load_rows(E3_DIR / f"{_e3_tag(scenario_name, 0, s)}.csv")
            if _never_acts(i, s, unc):
                (E3_DIR / f"{_e3_tag(scenario_name, i, s)}.json").write_text(json.dumps(
                    _e3_result(scenario_name, i, s, unc, "never left OPEN: identical to no system")))
                report(i, s)
            else:
                queue.insert(0, (i, s))
        for (i, s), proc in list(running.items()):
            if proc.poll() is not None:
                del running[(i, s)]
                if proc.returncode == 0 and done(i, s):
                    report(i, s)
                else:
                    print(f"run {_e3_tag(scenario_name, i, s)} failed (exit {proc.returncode})", flush=True)
        while queue and len(running) < workers:
            i, s = queue.pop(0)
            if done(i, s):
                report(i, s)
                continue
            running[(i, s)] = subprocess.Popen([sys.executable, __file__, "--e3-task", scenario_name, str(i), str(s)])
        if running or deferred:
            time.sleep(5)

    results = [json.loads(p.read_text()) for p in sorted(E3_DIR.glob(f"{scenario_name}_s*_v*.json"))]
    results.sort(key=lambda r: (r["variant_id"], r["seed"]))
    write(results, f"e3_{scenario_name}_seeds")
    write(e3_summary(results), f"e3_{scenario_name}_summary")
    return results


def e3_summary(results):
    out = []
    for i in sorted({r["variant_id"] for r in results}):
        rs = [r for r in results if r["variant_id"] == i]
        row = {"variant": E3_VARIANTS[i][0], "runs": len(rs)}
        for k in ["peak_core_density", "seconds_above_4", "seconds_above_5", "pilgrims_diverted",
                  "pilgrims_through_gate"]:
            x = np.array([r[k] for r in rs], float)
            row[f"{k}_mean"] = round(float(x.mean()), 2)
            row[f"{k}_sd"] = round(float(x.std(ddof=1)), 2) if len(x) > 1 else 0.0
            row[f"{k}_min"] = round(float(x.min()), 2)
            row[f"{k}_max"] = round(float(x.max()), 2)
        row["runs_never_above_4"] = int(sum(r["seconds_above_4"] == 0 for r in rs))
        out.append(row)
    return out


def e5_gamma_sweep(rows_by_scenario, params, normal_rows):
    """Sensitivity of the QS per-hop decay gamma: earlier upstream warning vs
    more WATCH alerts in normal operation."""
    from dataclasses import replace
    out = []
    for g in [0.7, 0.8, 0.9, 0.95]:
        p = replace(params, gamma=g)
        row = {"gamma": g}
        for name, rows in rows_by_scenario.items():
            t_d = danger_time(rows)
            t, st = replay(rows, make_method("qs", p, np.random.default_rng(0)))
            core = first_time((st[:, CORE_NODES] >= CRITICAL).any(axis=1), t)
            road = first_time((st[:, ROAD_NODES] >= WATCH).any(axis=1), t)
            row[f"{name}_core_lead_s"] = (t_d - core) if core is not None else None
            row[f"{name}_road_lead_s"] = (t_d - road) if road is not None else None
        t, st = replay(normal_rows, make_method("qs", p, np.random.default_rng(0)))
        row["normal_WATCH_events_per_hour"] = float(
            np.sum(np.diff(np.r_[np.zeros((1, N)), (st >= WATCH).astype(int)], axis=0) == 1) / (len(normal_rows) / 3600))
        out.append(row)
    return out


def e4_false_alerts(rows, params):
    hours = len(rows) / 3600
    out = []
    for name in METHODS:
        t, st = replay(rows, make_method(name, params, np.random.default_rng(0)))
        crit_events = int(np.sum(np.diff(np.r_[np.zeros((1, N)), (st >= CRITICAL).astype(int)], axis=0) == 1))
        watch_events = int(np.sum(np.diff(np.r_[np.zeros((1, N)), (st >= WATCH).astype(int)], axis=0) == 1))
        out.append({"method": name, "CRITICAL_events_per_hour": crit_events / hours,
                    "WATCH_events_per_hour": watch_events / hours,
                    "peak_density_in_run": float(max(rho_vec(r).max() for r in rows))})
    return out


def load_or_run(name):
    path = RES / f"{name}_uncontrolled.csv"
    if not path.exists():
        rows, _ = gs.run(gs.SCENARIOS[name], verbose=True)
        gs.save_rows(rows, path)
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def write(rows, name):
    RES.mkdir(exist_ok=True)
    with open(RES / f"{name}.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    print(json.dumps(rows, indent=1, default=str))


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--experiment", nargs="+", default=["all"])
    ap.add_argument("--e3-variants", nargs="+", type=int, default=None,
                    help="run only these E3 variant indices (0 none, 1 threshold, 2 centralised, 3 centralised cut, 4 QS, "
                         "5 route QS, 6 hybrid, 7 hybrid cut, 8-11 route QS with a dead board G/C1 without/with "
                         "failure handling)")
    ap.add_argument("--e3-seeds", nargs="+", type=int, default=None,
                    help="run E3 over these scenario seeds, one process per run (includes variant 5, route QS)")
    ap.add_argument("--e3-scenario", default="surge")
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--e3-task", nargs=3, default=None, help=argparse.SUPPRESS)
    args = ap.parse_args()
    if args.e3_task:
        name, i, seed = args.e3_task
        e3_task(name, int(i), int(seed))
        return
    ex = set(args.experiment)
    run_all = "all" in ex
    params = QSParams()

    if run_all or ex & {"e1", "e2"}:
        e1, e2 = [], []
        for name in ["surge", "blocked"]:
            rows = load_or_run(name)
            if run_all or "e1" in ex:
                e1 += e1_timing(rows, params, name)
            if run_all or "e2" in ex:
                e2 += e2_robustness(rows, params, name)
        if e1:
            write(e1, "e1_alert_timing")
        if e2:
            write(e2, "e2_robustness")
    if run_all or "e4" in ex:
        write(e4_false_alerts(load_or_run("normal"), params), "e4_false_alerts")
    if run_all or "e5" in ex:
        rows_by = {n: load_or_run(n) for n in ["surge", "blocked"]}
        write(e5_gamma_sweep(rows_by, params, load_or_run("normal")), "e5_gamma_sweep")
    if (run_all or "e3" in ex) and args.e3_seeds:
        e3_multi(args.e3_scenario, args.e3_seeds, args.workers, args.e3_variants)
    elif run_all or "e3" in ex:
        write(e3_closed_loop("surge", only=args.e3_variants), "e3_closed_loop")


if __name__ == "__main__":
    main()
