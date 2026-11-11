"""Verification of the pedestrian model against RiMEA guideline test cases,
plus validation of the fundamental diagram against Weidmann (1993).

RiMEA (Richtlinie fuer Mikroskopische Entfluchtungsanalysen) is the de facto
international benchmark for pedestrian simulators; Vadere runs it on every
code change and JuPedSim's verification framework uses it.

Tests implemented:
  Test 1  - one person, 1.33 m/s, 40 m corridor: travel time must be 26-34 s.
  Test 4  - fundamental diagram (speed vs density) in a corridor, compared
            with Weidmann's planar FD (v0 = 1.34 m/s, rho_max = 5.4 /m^2,
            gamma = 1.913 /m^2).
  Test 12 - bottleneck: congestion must form in front of the bottleneck,
            not at the corridor exit after it.

Usage:  python rimea_tests.py   (writes results/rimea_*.csv and .png)
"""

import csv
from pathlib import Path

import jupedsim as jps
import numpy as np
from shapely import box
from shapely.ops import unary_union

RES = Path(__file__).resolve().parent / "results"
DT = 0.01


def model():
    return jps.CollisionFreeSpeedModel()


def weidmann_speed(rho, v0=1.34, rho_max=5.4, gamma=1.913):
    rho = np.asarray(rho, dtype=float)
    return v0 * (1 - np.exp(-gamma * (1 / rho - 1 / rho_max)))


def test1():
    corridor = box(0, 0, 42, 2)
    sim = jps.Simulation(model=model(), geometry=corridor, dt=DT)
    exit_id = sim.add_exit_stage(box(40, 0, 42, 2))
    j = sim.add_journey(jps.JourneyDescription([exit_id]))
    sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(
        journey_id=j, stage_id=exit_id, position=(0.5, 1.0), desired_speed=1.33, radius=0.2))
    # time to cover 40 m (from x=0.5 to the exit area starting at x=40, i.e. 39.5 m, then
    # scaled to exactly 40 m so the check matches the guideline wording)
    while sim.agent_count() > 0 and sim.elapsed_time() < 100:
        sim.iterate()
    t = sim.elapsed_time() * 40 / 39.5
    return {"test": "RiMEA 1 - speed in corridor", "measured": round(t, 2),
            "required": "26-34 s", "pass": 26 <= t <= 34}


def test4(densities=(0.5, 1.0, 1.5, 2.0, 3.0, 4.0, 5.0), length=60.0, width=4.0, time_gap=1.0, workers=1):
    """Agents placed at a given density in a corridor walk towards a far exit.
    Mean speed is measured in a 10 m window in the middle during 10-30 s.
    Densities are independent simulations, run in parallel."""
    tasks = [(rho, length, width, time_gap) for rho in densities]
    if workers <= 1 or len(tasks) <= 1:          # inline: no process pool at all
        return [_fd_point(t) for t in tasks]
    from concurrent.futures import ProcessPoolExecutor
    with ProcessPoolExecutor(max_workers=min(workers, len(tasks))) as ex:
        return list(ex.map(_fd_point, tasks))


def _positions(sim):
    ags = list(sim.agents())
    ids = np.fromiter((a.id for a in ags), np.int64, len(ags))
    xy = np.array([a.position for a in ags]).reshape(-1, 2)
    return ids, xy


def _inside(xy, b):
    """Strictly inside an axis-aligned box (minx, miny, maxx, maxy), like shapely contains."""
    return (xy[:, 0] > b[0]) & (xy[:, 0] < b[2]) & (xy[:, 1] > b[1]) & (xy[:, 1] < b[3])


def _fd_point(task):
    rho, length, width, time_gap = task
    corridor = box(0, 0, length + 200, width)
    sim = jps.Simulation(model=model(), geometry=corridor, dt=DT)
    exit_id = sim.add_exit_stage(box(length + 198, 0, length + 200, width))
    j = sim.add_journey(jps.JourneyDescription([exit_id]))
    n = int(rho * length * width)
    spacing = 1 / np.sqrt(rho)
    # fill the corridor with a lattice of the target density, keeping a
    # wall clearance the model requires (> radius)
    ny = max(1, int(round(width * np.sqrt(rho))))
    ys = np.linspace(0.25, width - 0.25, ny)
    dx = 1 / (rho * (width / ny)) if ny else spacing
    pos = [(x, y) for x in np.arange(dx / 2, length, dx) for y in ys][:n]
    rng = np.random.default_rng(1)
    for p in pos:
        sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=j, stage_id=exit_id, position=p,
            desired_speed=float(rng.normal(1.34, 0.1)), radius=0.15, time_gap=time_gap))
    meas = (length / 2 - 5, 0, length / 2 + 5, width)
    meas_area = 10 * width
    speeds, dens = [], []
    # agent ids are a process-wide counter (they keep growing across simulations),
    # so index the previous-position array by id relative to this simulation's first id
    last = np.full((2 * len(pos) + 16, 2), np.nan)
    base = None
    while sim.elapsed_time() < 30:
        sim.iterate(100)
        ids, xy = _positions(sim)
        if base is None:
            base = int(ids.min())
        ids = ids - base
        if sim.elapsed_time() >= 10:
            inside = _inside(xy, meas)
            prev = last[ids[inside]]
            v = np.hypot(xy[inside, 0] - prev[:, 0], xy[inside, 1] - prev[:, 1])
            v = v[~np.isnan(v)]
            if len(v):
                speeds.append(np.mean(v))
                dens.append(inside.sum() / meas_area)
        last[:] = np.nan
        last[ids] = xy
    return {"target_density": rho, "measured_density": round(float(np.mean(dens)), 2),
            "measured_speed": round(float(np.mean(speeds)), 3),
            "weidmann_speed": round(float(weidmann_speed(np.mean(dens))), 3)}


def test12(n=150, bottleneck_w=1.0, time_gap=1.0):
    """Room (10x10) -> bottleneck corridor (1 m x 4 m) -> room 2 -> exit.
    Pass if density at the bottleneck entrance >> density at the corridor exit."""
    room1 = box(0, 0, 10, 10)
    corr = box(10, 5 - bottleneck_w / 2, 14, 5 + bottleneck_w / 2)
    room2 = box(14, 0, 24, 10)
    geo = unary_union([room1, corr, room2])
    sim = jps.Simulation(model=model(), geometry=geo, dt=DT)
    exit_id = sim.add_exit_stage(box(22, 4, 24, 6))
    wp = sim.add_waypoint_stage((11, 5), 0.5)
    jd = jps.JourneyDescription([wp, exit_id])
    jd.set_transition_for_stage(wp, jps.Transition.create_fixed_transition(exit_id))
    j = sim.add_journey(jd)
    for p in jps.distribute_by_number(polygon=box(0.5, 0.5, 8, 9.5), number_of_agents=n,
                                      distance_to_agents=0.45, distance_to_polygon=0.2, seed=3):
        sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(
            journey_id=j, stage_id=wp, position=p, desired_speed=1.34, radius=0.2, time_gap=time_gap))
    before = (8, 3, 10, 7)             # in front of the bottleneck (8 m^2)
    after = (14, 3, 16, 7)             # at the corridor exit (8 m^2)
    d_before, d_after = [], []
    seen = np.zeros(2 * n + 16, bool)
    base = None                                     # ids are process-wide: index relative to the first
    while sim.agent_count() > 0 and sim.elapsed_time() < 300:
        sim.iterate(100)
        ids, xy = _positions(sim)
        if base is None:
            base = int(ids.min())
        ids = ids - base
        d_before.append(_inside(xy, before).sum() / 8.0)
        d_after.append(_inside(xy, after).sum() / 8.0)
        seen[ids[xy[:, 0] > 14]] = True
    t_total = sim.elapsed_time()
    flow = seen.sum() / max(t_total, 1e-9) / bottleneck_w
    mb, ma = float(np.max(d_before)), float(np.max(d_after))
    return {"test": "RiMEA 12 - bottleneck", "max_density_before": round(mb, 2),
            "max_density_after": round(ma, 2), "specific_flow_p_per_m_s": round(flow, 2),
            "required": "congestion before bottleneck, none after",
            "pass": mb > 2 * ma and ma < 2.0}


def main():
    RES.mkdir(exist_ok=True)
    # run the parallel test first: on Windows, starting worker processes after the
    # parent has created a JuPedSim simulation breaks the pool (WinError 6)
    fd = test4()
    summary = [test1()]
    print(summary[-1])
    for r in fd:
        print(r)
    rmse = float(np.sqrt(np.mean([(r["measured_speed"] - r["weidmann_speed"]) ** 2 for r in fd])))
    summary.append({"test": "RiMEA 4 - fundamental diagram vs Weidmann", "measured": f"RMSE {rmse:.3f} m/s",
                    "required": "qualitative agreement (speed falls with density)",
                    "pass": all(fd[i]["measured_speed"] >= fd[i + 1]["measured_speed"] - 0.02
                                for i in range(len(fd) - 1))})
    print(summary[-1])
    summary.append(test12())
    print(summary[-1])

    with open(RES / "rimea_summary.csv", "w", newline="") as f:
        keys = sorted({k for s in summary for k in s})
        w = csv.DictWriter(f, fieldnames=keys)
        w.writeheader()
        w.writerows(summary)
    with open(RES / "rimea_fundamental_diagram.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(fd[0]))
        w.writeheader()
        w.writerows(fd)

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    r = np.linspace(0.3, 5.3, 100)
    plt.figure(figsize=(5, 4))
    plt.plot(r, weidmann_speed(r), label="Weidmann (1993)")
    plt.plot([x["measured_density"] for x in fd], [x["measured_speed"] for x in fd], "o", label="JuPedSim (our model)")
    plt.xlabel("Density (persons/m²)")
    plt.ylabel("Speed (m/s)")
    plt.title("RiMEA test 4: fundamental diagram")
    plt.legend()
    plt.tight_layout()
    plt.savefig(RES / "rimea_fundamental_diagram.png", dpi=150)


if __name__ == "__main__":
    main()
