"""Calibrate the pedestrian model's time gap T against Weidmann's fundamental
diagram (RiMEA test 4 setup) and report bottleneck flow (RiMEA test 12).

The collision-free speed model sets speed from headway s as v = min(v0, (s - 2r)/T),
so T is the one parameter that shapes the fundamental diagram. The chosen T is
written to results/calibration.json and picked up by ghat_scenario.py.

Usage:  python calibrate.py
"""

import json
import sys

import numpy as np

from rimea_tests import RES, test4, test12, weidmann_speed

DENSITIES = (0.5, 1.0, 2.0, 3.0, 4.0)


def weidmann_capacity():
    """Peak specific flow of Weidmann's planar FD, max over rho of rho * v(rho)."""
    r = np.linspace(0.2, 5.3, 2000)
    return float(np.max(r * weidmann_speed(r)))


def select(results):
    """Lowest FD error among settings whose bottleneck flow is at least the
    Weidmann capacity (a bottleneck that clogs below free-walkway capacity is
    unrealistic; T = 0.7 does this in the sweep)."""
    cap = weidmann_capacity()
    ok = [r for r in results if r["bottleneck_specific_flow"] >= cap and r["rimea12_pass"]]
    best = min(ok or results, key=lambda r: r["fd_rmse_m_s"])
    return best, cap


def write(results):
    best, cap = select(results)
    out = {"chosen_time_gap": best["time_gap"],
           "criterion": f"min RMSE vs Weidmann FD subject to bottleneck flow >= Weidmann capacity ({cap:.2f} p/(m s))",
           "sweep": results}
    (RES / "calibration.json").write_text(json.dumps(out, indent=1))
    print("chosen time_gap:", best["time_gap"], "|", out["criterion"])


def main():
    RES.mkdir(exist_ok=True)
    if "--reselect" in sys.argv:          # re-apply the criterion to a saved sweep
        write(json.loads((RES / "calibration.json").read_text())["sweep"])
        return
    results = []
    for T in (1.0, 0.7, 0.5, 0.35):
        fd = test4(densities=DENSITIES, time_gap=T)
        rmse = float(np.sqrt(np.mean([(r["measured_speed"] - r["weidmann_speed"]) ** 2 for r in fd])))
        bn = test12(time_gap=T)
        res = {"time_gap": T, "fd_rmse_m_s": round(rmse, 3),
               "bottleneck_specific_flow": bn["specific_flow_p_per_m_s"], "rimea12_pass": bn["pass"],
               "fd": fd}
        print({k: v for k, v in res.items() if k != "fd"})
        results.append(res)
    write(results)


if __name__ == "__main__":
    main()
