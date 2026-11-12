"""Equivalence test: the firmware estimator (firmware/include/pk_sense.h, float32) against the
reference sensing.kf_step (float64), on the same inputs.

Six nodes cover every branch: counters + readings, counters only (dead reckoning), readings
only, blind, counters + readings with a FLAGGED sensor fault at t = 1500 s, and readings only
with a NAIVE fault at t = 2000 s. Inputs: an hour of biased, noisy in/out counts around a
surge, and noisy density readings every 10-60 s with dropouts.

Pass: the estimates agree within 0.01 persons/m^2 at every second (float32 vs float64), and
the alert-relevant crossings (rho >= 2.4 WATCH, >= 3.4 CRITICAL at rho_crit 4) agree in
>= 99.9 % of node-seconds.
Needs g++ (WSL). Usage: python test_sense_port.py
"""

import subprocess
import tempfile
from pathlib import Path

import numpy as np

from sensing import Sensing, SensingConfig, kf_step

ROOT = Path(__file__).resolve().parent.parent
T = 3600
NODES = [  # area, counted, observed, fault (t, mode)
    (600.0, True, True, None),
    (600.0, True, False, None),
    (7500.0, False, True, None),
    (1000.0, False, False, None),
    (600.0, True, True, (1500, "flag")),
    (7500.0, False, True, (2000, "naive")),
]
# boundary width (m) -> physics-based random walk 1.22 x width / area; 0 = legacy fixed walk (last node)
WIDTH = [20.0, 20.0, 30.0, 25.0, 20.0, 0.0]


def make_inputs(seed=7):
    r = np.random.default_rng(seed)
    n = len(NODES)
    area = np.array([a for a, *_ in NODES])
    t = np.arange(T)
    q_in = 4.0 + 6.0 * ((t > 600) & (t < 1800)) + 2.0 * np.sin(t / 300)
    q_out = 4.5 + 1.0 * np.sin(t / 200)
    N = np.zeros(n)
    cin = np.zeros((T, n)); cout = np.zeros((T, n)); due = np.zeros((T, n), bool); z = np.zeros((T, n))
    every = np.array([10, 30, 60, 10, 20, 10])
    for k in range(T):
        tin = r.poisson(np.maximum(q_in[k], 0), n).astype(float)
        tout = np.minimum(r.poisson(np.maximum(q_out[k], 0), n).astype(float), N + tin)
        N += tin - tout
        cin[k] = np.maximum(tin * 1.03 + r.normal(0, 0.3 * np.sqrt(tin)), 0)     # 3 % biased counters
        cout[k] = np.maximum(tout * 0.99 + r.normal(0, 0.3 * np.sqrt(tout)), 0)
        due[k] = (k % every == 0) & (r.random(n) > 0.2)
        z[k] = np.maximum(N / area * (1 + r.normal(0, 0.1, n)) + r.normal(0, 0.05, n), 0)
    return cin, cout, due, z


def reference(cin, cout, due, z, cfg):
    n = len(NODES)
    area = np.array([a for a, *_ in NODES])
    counted = np.array([c for _, c, _, _ in NODES])
    observed = np.array([o for *_, o, _ in NODES])
    state = (np.zeros(n), np.zeros(n), np.full(n, 1e-4), np.zeros(n), np.full(n, (5 * cfg.b_walk_sd) ** 2))
    flagged = np.zeros(n, bool)
    frozen = np.zeros(n)
    from route_graph import PEAK_SPECIFIC_FLOW
    w = np.array(WIDTH)
    rw = np.where(w > 0, PEAK_SPECIFIC_FLOW * w / np.maximum(area, 1e-9), cfg.rw_sd)
    out = np.zeros((T, n))
    for k in range(T):
        for i, (*_, fault) in enumerate(NODES):
            if fault and fault[0] == k:                        # as Sensing.sensor_fault
                observed[i] = False
                if fault[1] == "flag":
                    flagged[i] = True
                    frozen[i] = max(state[0][i], Sensing.WATCH_RHO)
        blind = ~counted & ~observed
        state = kf_step(state, cin[k], cout[k], area, counted, due[k] & observed, z[k], blind, cfg, rw=rw)
        rho = np.where(flagged, np.maximum(state[0], frozen), state[0])
        state = (rho, *state[1:])
        out[k] = rho
    return out


def firmware(cin, cout, due, z):
    with tempfile.TemporaryDirectory() as d:
        d = Path(d)
        exe = d / "sense_check"
        subprocess.run(["g++", "-O2", "-std=c++17", "-o", str(exe), str(ROOT / "firmware/sil/sense_check.cpp")],
                       check=True)
        with open(d / "in.txt", "w") as f:
            f.write(f"{T} {len(NODES)}\n")
            for (a, c, o, _), w in zip(NODES, WIDTH):
                f.write(f"{a} {int(c)} {int(o)} {w}\n")
            for k in range(T):
                for i, (*_, fault) in enumerate(NODES):
                    fl = 0
                    if fault and fault[0] == k:
                        fl = 2 if fault[1] == "flag" else 1
                    f.write(f"{cin[k, i]:.6f} {cout[k, i]:.6f} {int(due[k, i])} {z[k, i]:.6f} {fl}\n")
        subprocess.run([str(exe), str(d / "in.txt"), str(d / "out.txt")], check=True)
        return np.loadtxt(d / "out.txt")


def test_firmware_matches_reference():
    cfg = SensingConfig()
    cin, cout, due, z = make_inputs()
    # the firmware reads the inputs at 6 decimals; give the reference the same rounding
    cin, cout, z = (np.round(x, 6) for x in (cin, cout, z))
    ref = reference(cin, cout, due, z, cfg)
    fw = firmware(cin, cout, due, z)
    diff = np.abs(ref - fw)
    level = lambda x: np.digitize(x, [2.4, 3.4])
    agree = float((level(ref) == level(fw)).mean())
    print(f"max |firmware - reference| = {diff.max():.2e} p/m^2 (per node: {np.round(diff.max(axis=0), 6).tolist()}); "
          f"alert-level agreement {100 * agree:.3f} %; final estimates {np.round(fw[-1], 3).tolist()}")
    assert diff.max() < 0.01, diff.max()
    assert agree >= 0.999, agree
    return diff.max(), agree


if __name__ == "__main__":
    test_firmware_matches_reference()
    print("firmware estimator matches sensing.kf_step")
