"""E8: site-scale test of over-alerting (68 compartments, 20 junctions, 10 Nagaras).

Scenarios (site_flow.py): a day-1 surge at one Nagara, and a partly blocked exit
at one compartment. Methods control the flow at Nagaras, roads, junctions and
gates:
  none                    no control
  threshold               each node acts on its own density only
  qs-gradient             radio-hop QS (decays 10 % per hop), 30 % link loss
  centralised             command centre sees all, warns every upstream node
  centralised-cut         same, backhaul cut at 300 s (before the surge)
  route-qs                alerts follow the route graph upstream: share-weighted,
                          METER/HOLD, junction diversion; 30 % link loss
  route-qs ablations      no-share / with-horizon (hold only near the hotspot) / no-meter / no-divert
  route-qs fail-safe      route-qs with the firmware's node-failure handling on (hold a silent
                          successor's last alarm until it is back + 2-hop backup)
  hybrid                  ICCC and route-qs together: each node takes the stronger action of the two
                          while the backhaul is up; route-qs alone without it
  hybrid-cut              hybrid with the backhaul cut at 300 s (as centralised-cut)
Actions from alert states: WATCH -> METER (50 % flow), CRITICAL -> HOLD (10 %).

Metrics
  comp_s_above_4 / 5      seconds any compartment is above 4 / 5 persons/m^2
  any_s_above_4           seconds ANY node (incl. roads, where holds push people) is above 4
  needless_hold_node_s    node-seconds of METER/HOLD at nodes that feed none of the
                          places that exceed 3.5 persons/m^2 when nothing is controlled
                          (over-alerting; successful prevention is NOT counted as needless)
  held_node_s             all node-seconds of METER/HOLD
  exited                  pilgrims who completed the visit (throughput)

Seeds: seed 0 is the nominal scenario of site_flow.SCENARIOS. Seeds >= 1 draw a
random variant (which Nagara surges, how hard and when; base demand; dwell time;
which compartment exit is blocked, how badly and when) and a fresh link-loss stream.
Each finished run is appended to results/e8/e8_runs.jsonl (resumable); the summary
gives mean / sd / min / max per scenario and method over the seeds.

Usage:  python site_experiment.py [--workers 2] [--seeds 0 1 2 ...]
"""

import argparse
import csv
import json
from dataclasses import replace
from pathlib import Path

import numpy as np

from qs_core import (ACT_HOLD, ACT_METER, ACT_NONE, CRITICAL, WATCH, CentralisedBaseline, QSNetwork, QSParams,
                     RouteParams, RouteQSNetwork, ThresholdBaseline)
from route_graph import synthetic_site
from sensing import Sensing, SensingConfig
from site_flow import CONTROL, SCENARIOS, SiteFlow, branch_density_index, divert_shares

RES = Path(__file__).resolve().parent / "results"
LINK_LOSS = 0.3
PROBLEM_RHO = 3.5
CONTROLLED_KINDS = {"source", "road", "junction", "gate"}
CTRL = np.array([CONTROL[a] for a in range(4)])     # action -> flow multiplier

METHODS = ["none", "threshold", "qs-gradient", "centralised", "centralised-cut",
           "route-qs", "route-qs no-share", "route-qs with-horizon", "route-qs no-meter", "route-qs no-divert",
           "route-qs fail-safe", "hybrid", "hybrid-cut"]
FAILSAFE = {"failsafe_s": np.inf, "backup": True}


def state_to_action(state):
    return np.where(state >= CRITICAL, ACT_HOLD, np.where(state >= WATCH, ACT_METER, ACT_NONE))


def draw_scenario(scenario, seed):
    """Seed 0: the nominal scenario. Otherwise a random variant of it."""
    sc = SCENARIOS[scenario]
    if seed == 0:
        return sc
    r = np.random.default_rng([seed, 8])
    sc = replace(sc, base_rate=float(r.uniform(5.0, 7.0)), dwell_s=float(r.uniform(240, 360)))
    if sc.blocked_comp is None:
        start = int(r.uniform(300, 900))
        sc = replace(sc, surge_nagara=int(r.integers(10)), surge_rate=float(r.uniform(35, 60)),
                     surge_start=start, surge_end=start + int(r.uniform(900, 1500)))
    else:
        sc = replace(sc, blocked_comp=int(r.integers(68)), blocked_factor=float(r.uniform(0.01, 0.1)),
                     blocked_at=int(r.uniform(200, 600)))
    return sc


def hotspots(scenario, seed=0):
    """Nodes that exceed PROBLEM_RHO when nothing is controlled: the places that
    genuinely need protecting. A hold is needless if the node feeds none of them."""
    g = synthetic_site()
    sc = draw_scenario(scenario, seed)
    flow = SiteFlow(g, sc)
    hit = np.zeros(g.n, bool)
    for t in range(sc.duration_s):
        flow.step(t, np.ones(g.n))
        hit |= flow.rho >= PROBLEM_RHO
    return np.nonzero(hit)[0].tolist()


def run(task):
    """task: (scenario, method, seed, hotspots[, extra]); extra may hold
    kills: [(node, t_s), ...] boards that die at t_s (no radio, no signal to their marshals),
    rp: RouteParams overrides (e.g. FAILSAFE), label: free text copied to the output,
    sensing: a SensingConfig as a dict (E12); without it the nodes see the true density,
    sensor_kills: [(node, t_s, mode), ...] density sensors that fail at t_s while the board
    runs on (mode "naive" or "flag", see sensing.Sensing.sensor_fault)."""
    scenario, method, seed, hot = task[:4]
    extra = task[4] if len(task) > 4 else {}
    g = synthetic_site()
    sc = draw_scenario(scenario, seed)
    flow = SiteFlow(g, sc)
    n = g.n
    rng = np.random.default_rng(seed)
    p = QSParams()
    controlled = np.array([k in CONTROLLED_KINDS for k in g.kind])
    feeds = np.zeros((n, n), bool)                    # feeds[i, j]: i sends pilgrims towards j
    for j in range(n):
        for i in g.upstream(j):
            feeds[i, j] = True
    hot = np.array(hot, int)
    needed = (feeds[:, hot].any(axis=1) if len(hot) else np.zeros(n, bool))
    needed[hot] = True                                 # nodes whose holds can protect a real hotspot
    bidx = branch_density_index(g)
    base_share = g.edge_arrays()[2]
    is_junction = np.array([k == "junction" for k in g.kind])
    eu, ev = g.edge_arrays()[:2]
    kill_t = np.full(n, np.inf)
    for node, t_s in extra.get("kills", []):
        kill_t[node] = t_s
    last_branch_rho = np.zeros(len(eu))
    sens = Sensing(g, SensingConfig(**extra["sensing"]), seed) if "sensing" in extra else None
    est = np.zeros(n)                                  # sensed density; the site starts empty
    err_sq_comp = 0.0
    err_max = 0.0
    err_max_dense = 0.0                                # largest error where the true density >= 2

    divert = True
    m_cc = None
    if method == "threshold":
        m = ThresholdBaseline(n, p)
    elif method == "qs-gradient":
        m = QSNetwork(g.radio_adjacency(), p)
    elif method.startswith("centralised"):
        m = CentralisedBaseline(n, p, g.upstream_map())
    elif method.startswith("route-qs") or method.startswith("hybrid"):
        rp = RouteParams()
        if "fail-safe" in method:
            rp = replace(rp, **FAILSAFE)
        if "rp" in extra:
            rp = replace(rp, **extra["rp"])
        if "no-share" in method:
            rp = replace(rp, use_share=False)
        if "with-horizon" in method:
            rp = replace(rp, use_horizon=True)
        if "no-meter" in method:
            rp = replace(rp, meter=False)
        divert = "no-divert" not in method
        m = RouteQSNetwork(g, p, rp)
        if method.startswith("hybrid"):
            m_cc = CentralisedBaseline(n, p, g.upstream_map())
    failure_handling = method not in ("none", "threshold", "qs-gradient") and hasattr(m, "rp") and         (m.rp.failsafe_s > 0 or m.rp.backup)
    ever_down = np.zeros(n, bool)
    first_act = None

    comp_above4 = comp_above5 = any_above4 = 0
    needless = held = 0
    peak_comp = peak_any = 0.0
    light_changes = np.zeros(n, int)
    prev_light = np.zeros(n, int)
    lit = np.zeros(n, bool)
    for t in range(sc.duration_s):
        diverting = np.zeros(n, bool)
        rho = flow.rho
        meas = est if sens is not None else rho        # what the boards measure (E12) or the truth
        dead = kill_t <= t
        seen = np.where(dead, 0.0, meas)               # a dead board reports nothing
        share = None
        if method == "none":
            action = np.zeros(n, int)
        elif method == "threshold":
            action = state_to_action(m.step(seen)[0])
        elif method == "qs-gradient":
            up = rng.random((n, n)) >= LINK_LOSS
            up = up & up.T
            up[:, dead] = False
            up[dead, :] = False
            action = state_to_action(m.step(seen, up)[0])
        elif method.startswith("centralised"):
            backhaul = method == "centralised" or t < 300
            action = state_to_action(m.step(seen, backhaul)[0])
        else:
            up = rng.random((n, n)) >= LINK_LOSS
            up = up & up.T
            up[:, dead] = False
            up[dead, :] = False
            m.step(seen, up)
            action = m.action.copy()
            if m_cc is not None:
                backhaul = method == "hybrid" or t < 300
                cc = state_to_action(m_cc.step(seen, backhaul)[0])
                if backhaul:
                    action = np.maximum(action, cc)
            ever_down[np.unique(m.v[:m.m0][m.down_report])] = True     # nodes reported DOWN to staff
            if divert:
                # what a junction knows about the room behind each branch: a gate relays its
                # compartment's density. Old firmware: a dead compartment board reads as empty (0),
                # a dead gate keeps its last value. Failure handling: unknown (no room).
                branch_rho = np.where(dead[bidx], 0.0, meas[bidx])
                branch_rho = np.where(dead[ev], last_branch_rho, branch_rho)
                if failure_handling:
                    branch_rho = np.where(dead[bidx] | dead[ev], np.inf, branch_rho)
                last_branch_rho = np.where(dead[ev], last_branch_rho, branch_rho)
                share = divert_shares(g, base_share, m.edge_threat,
                                      np.where(m.edge_room_known, branch_rho, np.inf))
                m.set_shares(share)                           # used only with dyn_share_tau_s > 0
                diverted = np.zeros(n, bool)
                diverted[np.unique(g.edge_arrays()[0][share != base_share])] = True
                action[is_junction & diverted] = ACT_NONE     # diverting instead of holding
                diverting = is_junction & diverted
        action = np.where(controlled & ~dead, action, ACT_NONE)     # a dead board's marshals get no signal
        # the light the marshals see: red HOLD, amber METER or DIVERT, green otherwise
        light = np.where(action >= ACT_HOLD, 2, np.where((action == ACT_METER) | diverting, 1, 0))
        light = np.where(controlled & ~dead, light, 0)
        light_changes += light != prev_light
        lit |= light > 0
        prev_light = light
        control = CTRL[action]
        edge_flow = flow.step(t, control, share)

        rho = flow.rho
        if sens is not None:
            for node, t_s, mode in extra.get("sensor_kills", []):
                if t == t_s:
                    sens.sensor_fault([node], mode)
            est = sens.update(t, edge_flow, flow.last_arrivals, flow.last_exit_out, rho)
            e = est - rho
            err_sq_comp += float(np.mean(e[flow.comps] ** 2))
            err_max = max(err_max, float(np.abs(e).max()))
            dense = rho >= 2.0
            if dense.any():
                err_max_dense = max(err_max_dense, float(np.abs(e[dense]).max()))
        comp_rho = rho[flow.comps]
        peak_comp = max(peak_comp, float(comp_rho.max()))
        peak_any = max(peak_any, float(rho.max()))
        comp_above4 += comp_rho.max() >= 4.0
        comp_above5 += comp_rho.max() >= 5.0
        any_above4 += rho.max() >= 4.0
        acting = action >= ACT_METER
        if first_act is None and acting.any():
            first_act = t
        held += int(acting.sum())
        needless += int((acting & ~needed).sum())
    out = {"scenario": scenario, "method": method, "seed": seed, "peak_compartment_density": round(peak_comp, 2),
           "comp_s_above_4": int(comp_above4), "comp_s_above_5": int(comp_above5),
           "peak_any_density": round(peak_any, 2), "any_s_above_4": int(any_above4),
           "held_node_s": held, "needless_hold_node_s": needless,
           "needless_pct": round(100 * needless / held, 1) if held else 0.0,
           "exited": int(flow.exited)}
    hours = sc.duration_s / 3600
    out["light_changes_per_h_mean"] = round(float(light_changes[lit].mean() / hours), 1) if lit.any() else 0.0
    out["light_changes_per_h_max"] = round(float(light_changes.max() / hours), 1)
    if sens is not None:
        out = {"sensing": sens.cfg.name, **out,
               "est_rmse_comp": round((err_sq_comp / sc.duration_s) ** 0.5, 3),
               "est_max_abs_err": round(err_max, 2), "est_max_abs_err_dense": round(err_max_dense, 2),
               "blind_nodes": int(sens.blind.sum()), "counted_nodes": int(sens.counted.sum())}
        if "kills" in extra or "sensor_kills" in extra:
            out.update({"label": extra.get("label", ""), "option": extra.get("option", method),
                        "t_first_alert": extra.get("t_first_alert"),
                        "killed": " ".join(f"{g.names[nd]}@{t_s}" for nd, t_s in extra.get("kills", [])),
                        "sensor_killed": " ".join(f"{g.names[nd]}@{t_s}:{md}"
                                                  for nd, t_s, md in extra.get("sensor_kills", [])),
                        "nodes_reporting_down": int(ever_down.sum()), "first_act_s": first_act})
        print(json.dumps(out), flush=True)
        return out
    if extra:
        out = {"label": extra.get("label", ""), "option": extra.get("option", method),
               "t_first_alert": extra.get("t_first_alert"),
               "killed": " ".join(f"{g.names[nd]}@{t_s}" for nd, t_s in extra.get("kills", [])),
               **out, "nodes_reporting_down": int(ever_down.sum()), "first_act_s": first_act}
    print(json.dumps(out), flush=True)
    return out


SUMMARY_KEYS = ["peak_compartment_density", "comp_s_above_4", "peak_any_density", "any_s_above_4",
                "held_node_s", "needless_hold_node_s", "needless_pct", "exited"]


def summary(rows):
    out = []
    for sc in SCENARIOS:
        for m in METHODS:
            rs = [r for r in rows if r["scenario"] == sc and r["method"] == m]
            if not rs:
                continue
            row = {"scenario": sc, "method": m, "runs": len(rs),
                   "runs_safe_everywhere": sum(r["any_s_above_4"] == 0 for r in rs),
                   "runs_with_needless_holds": sum(r["needless_hold_node_s"] > 0 for r in rs)}
            for k in SUMMARY_KEYS:
                x = np.array([r[k] for r in rs], float)
                row[f"{k}_mean"] = round(float(x.mean()), 2)
                row[f"{k}_sd"] = round(float(x.std(ddof=1)), 2) if len(x) > 1 else 0.0
                row[f"{k}_min"] = round(float(x.min()), 2)
                row[f"{k}_max"] = round(float(x.max()), 2)
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
    ap.add_argument("--workers", type=int, default=2)
    ap.add_argument("--seeds", nargs="+", type=int, default=[0])
    args = ap.parse_args()
    RES.mkdir(exist_ok=True)
    if args.seeds == [0]:                         # nominal scenarios only: the original E8 table
        hot = {s: hotspots(s) for s in SCENARIOS}
        tasks = [(s, m, 0, hot[s]) for s in SCENARIOS for m in METHODS]
        if args.workers <= 1:
            rows = [run(t) for t in tasks]
        else:
            with ProcessPoolExecutor(max_workers=args.workers) as ex:
                rows = list(ex.map(run, tasks))
        _write(rows, RES / "e8_site_scale.csv")
        return

    log = RES / "e8" / "e8_runs.jsonl"
    log.parent.mkdir(exist_ok=True)
    rows = [json.loads(x) for x in log.read_text().splitlines()] if log.exists() else []
    have = {(r["scenario"], r["method"], r["seed"]) for r in rows}
    hot = {(s, k): hotspots(s, k) for s in SCENARIOS for k in args.seeds}
    tasks = [(s, m, k, hot[(s, k)]) for k in args.seeds for s in SCENARIOS for m in METHODS
             if (s, m, k) not in have]
    print(f"{len(tasks)} runs to do ({len(have)} already done)", flush=True)
    with open(log, "a") as f, ProcessPoolExecutor(max_workers=max(1, args.workers)) as ex:
        for fut in as_completed([ex.submit(run, t) for t in tasks]):
            r = fut.result()
            rows.append(r)
            f.write(json.dumps(r) + "\n")
            f.flush()
    rows = [r for r in rows if r["seed"] in args.seeds]
    rows.sort(key=lambda r: (r["scenario"], METHODS.index(r["method"]), r["seed"]))
    _write(rows, RES / "e8_site_scale_seeds.csv")
    _write(summary(rows), RES / "e8_site_scale_summary.csv")
    for r in summary(rows):
        print(json.dumps({k: r[k] for k in ["scenario", "method", "runs", "runs_safe_everywhere",
                                            "runs_with_needless_holds", "any_s_above_4_mean",
                                            "needless_pct_mean", "exited_mean"]}), flush=True)


if __name__ == "__main__":
    main()
