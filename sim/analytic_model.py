"""E11: mathematical model of the PS-01 system, checked against the simulations.

M1  Compartment fluid model (one ODE, closed loop with the alert rules)
      dN/dt = I(t) - Q(N),   rho_C2 = min(N / A_eff, rho_jam)
      I(t)   = arrivals delayed by the road walk D, cut to the hold rate (HOLD) or 0 (CLOSE)
      Q(N)   = Q0 / (1 + (N / N50)^k)   outflow that FALLS as the compartment fills
               (bathers going in and leaving cross each other: counter-flow gridlock)
    Q, A_eff fitted on JuPedSim seeds 1-2 (uncontrolled); predictions tested on seeds 3-5
    and on the controlled runs (E3), which were never used for fitting.
    Alert rules are the real ones (qs_core): route QS = leaky level A, relayed hop by hop
    (one step per hop, plus the expected wait for a lost message); ICCC = no leak, no hops.
M2  Radio: effective loss, stale-link probability, extra delay over h hops
      P(stale) = q^(k+1);  E[extra delay] = h q / (1 - q)  (geometric retries, 1 msg/s)
    checked by Monte Carlo of RouteQSNetwork on the compartment surge (300 seeds per q).
M3  Reliability: probability the road is blind to its compartment when a fraction f of
    boards are dead (chain compartment -> gate -> junction -> road):
      no protection  1 - (1 - f)^2        2-hop backup  f^2
    checked by Monte Carlo on the 68-compartment route graph.
M4  Alarm stability: a state can drop only after MIN_DWELL steps below the release
    threshold, so a node changes state at most once per MIN_DWELL+1 s (<= 600 per hour);
    measured state changes per node-hour in the SIL logs and E3 road commands.
M5  Radio capacity: ESP-NOW airtime per neighbourhood; LoRa SF7 airtime per message
    (Semtech AN1200.13 formula) and how many 1-Hz long-hop nodes one channel can carry.

Writes results/e11_analytic_vs_sim.csv (predicted vs measured, every row) and
results/e11_fluid_fit.json.

Usage:  python analytic_model.py
"""

import csv
import json
import math
from pathlib import Path

import numpy as np

import ghat_scenario as gs
from qs_core import CRITICAL, WATCH, QSParams, RhoHistory, RouteParams, RouteQSNetwork, StateMachine
from route_graph import compartment_graph, synthetic_site

RES = Path(__file__).resolve().parent / "results"
E3 = RES / "e3"
P = QSParams()
SC = gs.SCENARIOS["surge"]
D_ROAD = 48.0                 # road 60 m at ~1.25 m/s
RHO_JAM = 6.05                # highest C2 density JuPedSim reaches (jam)


def load(path):
    with open(path) as f:
        return [{k: float(v) for k, v in r.items()} for r in csv.DictReader(f)]


def comp_occupancy(rows):
    return np.array([r["rho_C1"] * 300 + r["rho_C2"] * 300 + r["rho_X"] * 16 for r in rows])


# ---------------- M1 fluid model ----------------
def fit_fluid(train):
    """Fit Q(N) (outflow law) and A_eff from uncontrolled runs."""
    Ns, Qs, rN, rC2 = [], [], [], []
    for rows in train:
        g = np.array([r["gate_cum"] for r in rows])
        N = comp_occupancy(rows)
        out = g - N                                 # people who finished and left
        for a in range(100, len(rows) - 60, 30):
            b = a + 60
            Ns.append(N[a:b].mean())
            Qs.append(max(0.0, (out[b] - out[a]) / (b - a)))
        c2 = np.array([r["rho_C2"] for r in rows])
        keep = c2 < 0.95 * RHO_JAM
        rN += list(N[keep])
        rC2 += list(c2[keep])
    Ns, Qs = np.array(Ns), np.array(Qs)
    best = None
    for Q0 in np.arange(1.2, 2.01, 0.05):
        for N50 in np.arange(100, 2001, 25):
            for k in np.arange(0.5, 4.01, 0.25):
                err = np.mean((Q0 / (1 + (Ns / N50) ** k) - Qs) ** 2)
                if best is None or err < best[0]:
                    best = (err, Q0, N50, k)
    rN, rC2 = np.array(rN), np.array(rC2)
    sel = rN > 200
    A_eff = float(np.sum(rN[sel] ** 2) / np.sum(rN[sel] * rC2[sel]))   # least squares rho = N / A
    return {"Q0": round(best[1], 3), "N50": float(best[2]), "k": float(best[3]),
            "fit_rmse_p_per_s": round(math.sqrt(best[0]), 3), "A_eff_m2": round(A_eff, 1),
            "windows": len(Ns)}


def rho_c2(N, fit):
    return min(N / fit["A_eff_m2"], RHO_JAM)


def fluid_run(fit, method, T=1500, loss_q=0.51, hops=2):
    """Closed loop: fluid compartment + the alert rule of `method` driving the road command.
    method: 'none' | 'route-qs' | 'centralised'. Returns per-second rho_C2 and commands."""
    lam = lambda t: SC.surge_rate if SC.surge_start <= t < SC.surge_end else SC.base_rate
    N = 48.0                                     # steady pre-surge occupancy (measured)
    sent = np.zeros(T + int(D_ROAD) + 1)          # arrivals released at the road head each second
    hist = RhoHistory(1, P, 1.0)
    sm = StateMachine(1, P)
    A = 0.0
    lag = hops + hops * loss_q / (1 - loss_q)     # relay steps + expected waits for lost messages
    L_hist = []
    rhos, cmds = [], []
    cmd = gs.OPEN
    for t in range(T):
        rate = lam(t)
        if cmd == gs.HOLD:
            rate = min(rate, SC.hold_rate)
        elif cmd == gs.CLOSE:
            rate = 0.0
        sent[t] = rate
        inflow = sent[t - int(D_ROAD)] if t >= D_ROAD else SC.base_rate
        Q = fit["Q0"] / (1 + (N / fit["N50"]) ** fit["k"])
        N = max(0.0, N + inflow - Q)
        r = rho_c2(N, fit)
        s = hist.push(np.array([r]))[0]
        if method == "route-qs":
            A = P.delta * A + (1 - P.delta) * s
            L_hist.append(A)
            j = len(L_hist) - 1 - lag
            heard = np.interp(j, np.arange(len(L_hist)), L_hist) if j >= 0 else 0.0
            st = sm.update(np.array([heard]))[0]
        elif method == "centralised":
            st = sm.update(np.array([s]))[0]
        else:
            st = 0
        cmd = gs.CLOSE if st >= CRITICAL else (gs.HOLD if st >= WATCH else gs.OPEN)
        rhos.append(r)
        cmds.append(cmd)
    return np.array(rhos), np.array(cmds)


def measured_e3(variant, seeds):
    out = []
    for s in seeds:
        p = E3 / f"surge_s{s}_v{variant}.csv"
        if p.exists():
            rows = load(p)
            c2 = np.array([r["rho_C2"] for r in rows])
            core = np.array([[r[f"rho_{k}"] for k in ("G", "C1", "C2", "X")] for r in rows]).max(axis=1)
            t4 = int(np.argmax(core >= 4.0)) if (core >= 4.0).any() else None
            cmd = np.array([r["command"] for r in rows])
            out.append({"peak": core.max(), "s_above_4": int((core >= 4).sum()), "t_first_4": t4,
                        "first_hold": int(np.argmax(cmd > 0)) if (cmd > 0).any() else None,
                        "switches_per_h": float(np.sum(np.diff(cmd) != 0) / (len(cmd) / 3600)),
                        "c2_peak": c2.max()})
    return out


def m1(rows_out):
    unc = {s: load(E3 / f"surge_s{s}_v0.csv") for s in range(1, 6) if (E3 / f"surge_s{s}_v0.csv").exists()}
    fit = fit_fluid([unc[s] for s in (1, 2)])
    (RES / "e11_fluid_fit.json").write_text(json.dumps(fit, indent=1))
    print("fluid fit", fit, flush=True)
    test = [3, 4, 5]
    for method, variant, label in [("none", 0, "no control"), ("route-qs", 5, "route QS (30% link loss)"),
                                   ("centralised", 2, "ICCC, network up")]:
        rho, cmd = fluid_run(fit, method)
        meas = measured_e3(variant, test)
        if not meas:
            continue
        t4 = int(np.argmax(rho >= 4)) if (rho >= 4).any() else None
        mp = [m["peak"] for m in meas]
        rows_out.append({"model": "M1 fluid", "quantity": f"peak density, {label}",
                         "predicted": round(float(rho.max()), 2),
                         "measured": f"{np.mean(mp):.2f} (range {min(mp):.2f}-{max(mp):.2f})",
                         "measured_by": "JuPedSim E3 seeds 3-5 (not used for fitting)"})
        ms = [m["s_above_4"] for m in meas]
        rows_out.append({"model": "M1 fluid", "quantity": f"seconds above 4 p/m2, {label}",
                         "predicted": int((rho >= 4).sum()),
                         "measured": f"{np.mean(ms):.0f} (range {min(ms)}-{max(ms)})",
                         "measured_by": "JuPedSim E3 seeds 3-5"})
        if method == "none":
            mt = [m["t_first_4"] for m in meas if m["t_first_4"] is not None]
            rows_out.append({"model": "M1 fluid", "quantity": "time density first reaches 4 p/m2, no control (s)",
                             "predicted": t4, "measured": f"{np.mean(mt):.0f} (range {min(mt)}-{max(mt)})",
                             "measured_by": "JuPedSim E3 seeds 3-5"})
        else:
            fh = int(np.argmax(cmd > 0)) if (cmd > 0).any() else None
            mh = [m["first_hold"] for m in meas if m["first_hold"] is not None]
            rows_out.append({"model": "M1 fluid", "quantity": f"first road hold (s), {label}",
                             "predicted": fh, "measured": f"{np.mean(mh):.0f} (range {min(mh)}-{max(mh)})",
                             "measured_by": "JuPedSim E3 seeds 3-5"})
    # tipping point: the largest steady arrival rate the compartment can absorb
    Ngrid = np.linspace(0, 4000, 4001)
    Qg = fit["Q0"] / (1 + (Ngrid / fit["N50"]) ** fit["k"])
    rows_out.append({"model": "M1 fluid", "quantity": "max sustainable arrival rate (p/s) = max Q(N)",
                     "predicted": round(float(Qg.max()), 2),
                     "measured": "1.2 p/s normal day stays steady; 4.5 p/s surge runs away",
                     "measured_by": "JuPedSim normal / surge scenarios"})


# ---------------- M2 radio ----------------
def m2(rows_out):
    rows = load(RES / "surge_uncontrolled.csv")
    rho = np.array([[r[f"rho_{k}"] for k in gs.NODE_NAMES] for r in rows])
    g = compartment_graph()
    R2 = gs.NODE_NAMES.index("R2")
    n = g.n

    def first_road(loss, seed):
        net = RouteQSNetwork(g, P, RouteParams())
        rng = np.random.default_rng(seed)
        for t in range(len(rho)):
            up = rng.random((n, n)) >= loss
            st, _ = net.step(rho[t], up & up.T)
            if net.action[R2] >= 2:
                return t
        return None

    t0 = first_road(0.0, 0)
    hops = 3                                         # C2 -> C1 -> G -> R2
    for loss in [0.1, 0.3, 0.5]:
        q = 1 - (1 - loss) ** 2                      # both directions drawn, both must be up
        d = np.array([first_road(loss, s) - t0 for s in range(300)], float)
        rows_out.append({"model": "M2 radio", "quantity": f"extra road-warning delay, link loss {loss:.0%} "
                                                           f"(effective message loss {q:.0%}), mean s",
                         "predicted": round(hops * q / (1 - q), 2),
                         "measured": f"{d.mean():.2f} (p95 {np.percentile(d, 95):.0f}, max {d.max():.0f})",
                         "measured_by": "Monte Carlo, Python model, 300 seeds"})
        rows_out.append({"model": "M2 radio", "quantity": f"P(link stale > 5 s), effective loss {q:.0%}",
                         "predicted": f"{q ** 6:.2e}", "measured": "", "measured_by": ""})
    sil = RES / "e9_sil_radio_summary.csv"
    if sil.exists():
        with open(sil) as f:
            r = [x for x in csv.DictReader(f) if x["trace"] == "compartment-surge" and x["config"] == "as modelled"]
        if r:
            q = 0.3
            rows_out.append({"model": "M2 radio", "quantity": "firmware: first-action delay vs lock-step model, "
                                                               "30% message loss, async boards (s)",
                             "predicted": f"+{hops * q / (1 - q):.2f} from losses, minus up to {hops} s "
                                          f"(boards forward within ms, not at the next 1-s tick)",
                             "measured": f"median {r[0]['first_act_delay_median_s']}, range "
                                         f"{r[0]['first_act_earliest_s']} to +{r[0]['first_act_delay_worst_s']}",
                             "measured_by": "SIL E9, 10 seeds"})


# ---------------- M3 reliability ----------------
def m3(rows_out):
    g = synthetic_site()
    u, v, _, _ = g.edge_arrays()
    comps = [i for i in range(g.n) if g.kind[i] == "compartment"]
    pred = {int(b): int(a) for a, b in zip(u, v)}             # every gate / compartment has one feeder

    def blind(dead, backup):
        """Fraction of compartments (alive) whose road node (alive) cannot hear them."""
        bad = tot = 0
        for c in comps:
            gate = pred[c]
            junc = pred[gate]
            road = pred[junc]
            if dead[c] or dead[road]:
                continue
            tot += 1
            if backup:   # road hears junction, or gate via backup; junction hears gate or c via backup
                ok = (not dead[junc]) or (not dead[gate])
            else:
                ok = not dead[junc] and not dead[gate]
            bad += not ok
        return bad / tot

    rng = np.random.default_rng(11)
    for f in [0.05, 0.10, 0.20]:
        for backup, name, pred_v in [(False, "no protection", 1 - (1 - f) ** 2), (True, "2-hop backup", f ** 2)]:
            meas = np.mean([blind(rng.random(g.n) < f, backup) for _ in range(2000)])
            rows_out.append({"model": "M3 reliability", "quantity": f"P(road blind to its compartment), "
                                                                     f"{f:.0%} boards dead, {name}",
                             "predicted": round(pred_v, 4), "measured": round(float(meas), 4),
                             "measured_by": "Monte Carlo on the 68-compartment route graph, 2000 draws"})


# ---------------- M4 stability ----------------
def m4(rows_out):
    bound = 3600 / (P.min_dwell_steps + 1)
    worst = 0.0
    for p in (RES / "sil").glob("*_radio_out.txt"):
        d = np.loadtxt(p)
        for node in np.unique(d[:, 1]):
            s = d[d[:, 1] == node, 2]
            ch = np.sum(np.diff(s) != 0) / (len(s) / 3600)
            worst = max(worst, ch)
    rows_out.append({"model": "M4 stability", "quantity": "alert state changes per node-hour (upper bound vs worst "
                                                           "node seen)",
                     "predicted": f"<= {bound:.0f}", "measured": round(worst, 1),
                     "measured_by": "SIL E9 logs, every node, last radio seed of each trace"})
    meas = measured_e3(5, range(1, 6))
    if meas:
        sw = [m["switches_per_h"] for m in meas]
        rows_out.append({"model": "M4 stability", "quantity": "road command changes per hour, route QS closed loop",
                         "predicted": f"<= {bound:.0f}", "measured": f"{np.mean(sw):.1f} (max {max(sw):.1f})",
                         "measured_by": "JuPedSim E3, 5 seeds"})
    e4 = RES / "e4_false_alerts.csv"
    if e4.exists():
        with open(e4) as f:
            r = [x for x in csv.DictReader(f) if x["method"] == "route-qs"]
        if r:
            rows_out.append({"model": "M4 stability", "quantity": "false alerts per hour on a normal day (route QS)",
                             "predicted": "0 (s = rho/4 < 0.6 while rho < 2.4 and below the rate floor)",
                             "measured": f"WATCH {r[0]['WATCH_events_per_hour']}, CRITICAL "
                                         f"{r[0]['CRITICAL_events_per_hour']}",
                             "measured_by": "E4, normal scenario"})


# ---------------- M5 capacity ----------------
def lora_airtime_ms(payload, sf=7, bw=125e3, cr=1, preamble=8, header=True, de=False):
    """Semtech AN1200.13 time-on-air."""
    tsym = (2 ** sf) / bw * 1000
    h = 0 if header else 1
    n = 8 + max(math.ceil((8 * payload - 4 * sf + 28 + 16 - 20 * h) / (4 * (sf - 2 * int(de)))) * (cr + 4), 0)
    return (preamble + 4.25) * tsym + n * tsym


def m5(rows_out):
    espnow_us = 192 + (24 + 4 + 7 + 6 + 25) * 8
    for k in [10, 50, 100]:
        rows_out.append({"model": "M5 capacity", "quantity": f"ESP-NOW channel busy, {k} boards in range, 1 msg/s",
                         "predicted": f"{100 * k * espnow_us / 1e6:.1f}%", "measured": "", "measured_by": ""})
    t = lora_airtime_ms(25)
    rows_out.append({"model": "M5 capacity", "quantity": "LoRa SF7/125 kHz airtime per 25-byte message (ms)",
                     "predicted": round(t, 1), "measured": "", "measured_by": ""})
    # pure-ALOHA throughput peaks at 1/(2e) = 18.4% channel use
    per_node = t / 1000
    rows_out.append({"model": "M5 capacity", "quantity": "LoRa long-hop boards per channel at 1 msg/s (ALOHA 18%)",
                     "predicted": int(0.184 / per_node), "measured": "", "measured_by": ""})
    rows_out.append({"model": "M5 capacity", "quantity": "LoRa long-hop boards per channel, 1 msg per 10 s or on "
                                                          "state change",
                     "predicted": int(0.184 / (per_node / 10)), "measured": "", "measured_by": ""})
    sil = RES / "e9_sil_radio_summary.csv"
    if sil.exists():
        with open(sil) as f:
            r = list(csv.DictReader(f))
        rows_out.append({"model": "M5 capacity", "quantity": "bytes per board per second",
                         "predicted": 25.0, "measured": r[0]["bytes_per_node_s"],
                         "measured_by": "SIL E9 (loop period slightly over 1 s)"})


def main():
    out = []
    for f in (m1, m2, m3, m4, m5):
        f(out)
        print(f"{f.__name__} done", flush=True)
    with open(RES / "e11_analytic_vs_sim.csv", "w", newline="") as fh:
        w = csv.DictWriter(fh, fieldnames=["model", "quantity", "predicted", "measured", "measured_by"])
        w.writeheader()
        w.writerows(out)
    for r in out:
        print(f"{r['model']:15s} | {r['quantity'][:80]:80s} | pred {r['predicted']} | meas {r['measured']}")


if __name__ == "__main__":
    main()
