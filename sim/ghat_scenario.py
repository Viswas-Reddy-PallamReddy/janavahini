"""JuPedSim model of a bathing-ghat compartment (Kotilingala-style, assumed
dimensions until RTIH shares the model-compartment specs).

Layout (metres, flow left -> right):
    approach road  x 0-60,  8 m wide           zones R1 (0-30), R2 (30-60)
    entry gate     x 60-62, 3 m wide           node G (the 2015 failure point)
    compartment    x 62-92, 20 m deep          zones C1 holding (62-77), C2 steps (77-92)
    river edge     x ~90                        pilgrims dwell here to bathe
    exit corridor  from compartment top edge   node X, width configurable

Six fixed nodes measure, every second: density (persons/m^2), crowd pressure
P = rho * Var(speed) (Johansson et al. 2008) and gate flow. An optional
controller is called every second with those readings; it returns an inflow
command (OPEN / HOLD / CLOSE) applied at the road head, modelling police
diverting pilgrims to other ghats.

Usage:
    python ghat_scenario.py --scenario surge --out results/surge.csv
"""

import argparse
import csv
from dataclasses import dataclass, field
from pathlib import Path

import numpy as np
from shapely import box
from shapely.ops import unary_union

NODE_NAMES = ["R1", "R2", "G", "C1", "C2", "X"]
# radio links between neighbouring nodes (ESP-NOW range comfortably covers these)
LINKS = [("R1", "R2"), ("R2", "G"), ("G", "C1"), ("C1", "C2"), ("C2", "X"), ("C1", "X")]
# for the centralised baseline: which nodes feed (are upstream of) each node
UPSTREAM = {"C1": ["G", "R2", "R1"], "C2": ["C1", "G", "R2", "R1"], "G": ["R2", "R1"], "R2": ["R1"],
            "X": ["C2", "C1", "G", "R2", "R1"]}

OPEN, HOLD, CLOSE = 0, 1, 2
CALIBRATION = Path(__file__).resolve().parent / "results" / "calibration.json"


def calibrated_time_gap(default=1.0):
    """Time gap chosen by calibrate.py (fit to Weidmann's fundamental diagram)."""
    if CALIBRATION.exists():
        import json
        return float(json.loads(CALIBRATION.read_text())["chosen_time_gap"])
    return default


@dataclass
class Layout:
    road_len: float = 60.0
    road_w: float = 8.0
    gate_w: float = 3.0
    comp_len: float = 30.0
    comp_depth: float = 20.0
    exit_w: float = 2.0
    exit_x: float = 80.0
    exit_len: float = 16.0

    def build(self):
        g0 = self.road_len
        cy = self.road_w / 2
        road = box(0, 0, self.road_len, self.road_w)
        gate = box(g0 - 0.5, cy - self.gate_w / 2, g0 + 2.5, cy + self.gate_w / 2)
        c0 = g0 + 2
        y0 = cy - self.comp_depth / 2
        comp = box(c0, y0, c0 + self.comp_len, y0 + self.comp_depth)
        ex = box(self.exit_x, y0 + self.comp_depth - 0.5, self.exit_x + self.exit_w,
                 y0 + self.comp_depth + self.exit_len)
        area = unary_union([road, gate, comp, ex])
        zones = {
            "R1": box(0, 0, self.road_len / 2, self.road_w),
            "R2": box(self.road_len / 2, 0, self.road_len, self.road_w),
            "G": box(g0 - 6, 0, g0 + 2, self.road_w),   # gate approach (where 2015 surge formed)
            "C1": box(c0, y0, c0 + self.comp_len / 2, y0 + self.comp_depth),
            "C2": box(c0 + self.comp_len / 2, y0, c0 + self.comp_len, y0 + self.comp_depth),
            "X": box(self.exit_x, y0 + self.comp_depth, self.exit_x + self.exit_w,
                     y0 + self.comp_depth + 8),
        }
        geo = dict(
            area=area, zones=zones, gate_x=g0 + 1,
            spawn=box(0.5, 0.5, 4.0, self.road_w - 0.5),
            river=box(c0 + self.comp_len - 4, y0 + 1, c0 + self.comp_len - 0.5, y0 + self.comp_depth - 1),
            river_wp=(c0 + self.comp_len - 2, cy),
            gate_wp=(g0 + 3, cy),
            exit_poly=box(self.exit_x + 0.1, y0 + self.comp_depth + self.exit_len - 2,
                          self.exit_x + self.exit_w - 0.1, y0 + self.comp_depth + self.exit_len - 0.2),
            exit_entry_wp=(self.exit_x + self.exit_w / 2, y0 + self.comp_depth - 1.5),
        )
        return geo


@dataclass
class Scenario:
    name: str = "normal"
    duration_s: float = 1500.0
    base_rate: float = 1.5            # arrivals per second
    surge_rate: float = 1.5
    surge_start: float = 300.0
    surge_end: float = 900.0
    dwell_s: tuple = (60.0, 180.0)    # bathing time range
    exit_block_at: float | None = None
    exit_block_speed: float = 0.15    # desired speed of agents in a blocked exit
    hold_rate: float = 0.5            # inflow while HOLD (rest diverted)
    seed: int = 1
    layout: Layout = field(default_factory=Layout)


SCENARIOS = {
    "normal": Scenario(name="normal", base_rate=1.2, surge_rate=1.2),
    "surge": Scenario(name="surge", base_rate=1.2, surge_rate=4.5),          # day-1 muhurtham surge
    "blocked": Scenario(name="blocked", base_rate=1.5, surge_rate=1.5, exit_block_at=300.0),
}


def arrival_rate(sc: Scenario, t):
    return sc.surge_rate if sc.surge_start <= t < sc.surge_end else sc.base_rate


def run(sc: Scenario, controller=None, dt=0.05, frames_every_s=None, verbose=False):
    """Run one scenario. Returns (rows, frames).

    rows: one dict per second with density/pressure per node, gate flow,
    agents present, unserved demand and the inflow command.
    """
    # imported here so the zone/route constants of this module stay usable even
    # where the JuPedSim native library cannot load (e.g. blocked by Windows policy)
    import jupedsim as jps

    rng = np.random.default_rng(sc.seed)
    time_gap = calibrated_time_gap()
    geo = sc.layout.build()
    sim = jps.Simulation(model=jps.CollisionFreeSpeedModel(), geometry=geo["area"], dt=dt)

    gate_wp = sim.add_waypoint_stage(geo["gate_wp"], 1.5)
    river_wp = sim.add_waypoint_stage(geo["river_wp"], 3.0)
    exit_wp = sim.add_waypoint_stage(geo["exit_entry_wp"], 1.5)
    exit_id = sim.add_exit_stage(geo["exit_poly"])
    journey = jps.JourneyDescription([gate_wp, river_wp, exit_wp, exit_id])
    journey.set_transition_for_stage(gate_wp, jps.Transition.create_fixed_transition(river_wp))
    journey.set_transition_for_stage(river_wp, jps.Transition.create_fixed_transition(exit_wp))
    journey.set_transition_for_stage(exit_wp, jps.Transition.create_fixed_transition(exit_id))
    journey_id = sim.add_journey(journey)

    zones = geo["zones"]
    zone_names = list(zones)
    zone_area = np.array([zones[k].area for k in zone_names])
    # every zone and the river are axis-aligned boxes: strict bounds checks give the same
    # answer as shapely's contains_xy (which excludes the boundary), much faster
    zone_bounds = np.array([zones[k].bounds for k in zone_names])            # (Z, 4)
    river_b = np.array(geo["river"].bounds)
    exit_b = np.array(zones["X"].bounds)
    steps_per_s = int(round(1.0 / dt))
    bathing_until = {}            # agent id -> time their dwell ends
    desired = {}                  # agent id -> normal desired speed
    speed_cat = {}                # agent id -> 0 walking, 1 bathing, 2 blocked (setter only on change)
    cap = 4096                    # per-agent arrays indexed by agent id, grown as needed
    last_xy = np.full((cap, 2), np.nan)
    crossed = np.zeros(cap, bool)
    pending = 0.0                 # arrivals that could not enter (road head full) or were diverted
    diverted = 0.0
    command = OPEN
    rows, frames = [], []
    sx0, sy0, sx1, sy1 = geo["spawn"].bounds

    t_end = int(sc.duration_s)
    for sec in range(t_end):
        t = float(sec)
        # --- arrivals for this second ---
        lam = arrival_rate(sc, t)
        if command == HOLD:
            diverted += max(0.0, lam - sc.hold_rate)
            lam = min(lam, sc.hold_rate)
        elif command == CLOSE:
            diverted += lam
            lam = 0.0
        n_new = rng.poisson(lam) + int(pending)
        pending -= int(pending)
        failures = 0
        for i in range(n_new):
            if failures >= 3:
                # road head is full: the rest keep queueing (same meaning, O(1) per second
                # instead of retrying every queued arrival during a jam)
                pending += n_new - i
                break
            placed = False
            for _try in range(15):
                p = (rng.uniform(sx0, sx1), rng.uniform(sy0, sy1))
                if not list(sim.agents_in_range(p, 0.5)):
                    v = float(np.clip(rng.normal(1.25, 0.2), 0.6, 1.8))  # Indian crowd mean ~1.2 m/s
                    aid = sim.add_agent(jps.CollisionFreeSpeedModelAgentParameters(
                        journey_id=journey_id, stage_id=gate_wp, position=p,
                        desired_speed=v, radius=0.2, time_gap=time_gap))
                    desired[aid] = v
                    speed_cat[aid] = 0
                    placed = True
                    break
            if placed:
                failures = 0
            else:
                pending += 1
                failures += 1
        # --- advance one second ---
        sim.iterate(steps_per_s)

        # --- one pass over the agents ---
        agents = list(sim.agents())
        n = len(agents)
        ids = np.empty(n, np.int64)
        xy = np.empty((n, 2))
        for k, a in enumerate(agents):
            ids[k] = a.id
            xy[k] = a.position
        if n and ids.max() >= cap:
            grow = int(ids.max()) * 2 + 1 - cap
            last_xy = np.vstack([last_xy, np.full((grow, 2), np.nan)])
            crossed = np.concatenate([crossed, np.zeros(grow, bool)])
            cap += grow

        # --- bathing dwell and exit blockage (behaviour layer) ---
        blocked = sc.exit_block_at is not None and t >= sc.exit_block_at
        in_river = _in_box(xy, river_b)
        in_exit = _in_box(xy, exit_b)
        for k in range(n):
            aid = int(ids[k])
            if aid not in bathing_until and in_river[k]:
                bathing_until[aid] = t + rng.uniform(*sc.dwell_s)
            if aid in bathing_until and t < bathing_until[aid]:
                cat = 1                                 # standing in the river
            elif blocked and in_exit[k]:
                cat = 2
            else:
                cat = 0
            if speed_cat[aid] != cat:                   # the setter is expensive: only on change
                speed_cat[aid] = cat
                agents[k].model.desired_speed = (desired[aid], 0.05, sc.exit_block_speed)[cat]

        # --- sensing at the nodes ---
        row = {"t": t, "agents": n, "pending": pending, "diverted": diverted, "command": command}
        prev = last_xy[ids]
        speed = np.hypot(xy[:, 0] - prev[:, 0], xy[:, 1] - prev[:, 1])   # NaN for new agents
        crossed[ids[xy[:, 0] > geo["gate_x"]]] = True
        row["gate_cum"] = int(crossed.sum())
        last_xy[:] = np.nan
        last_xy[ids] = xy
        inside_all = ((xy[None, :, 0] > zone_bounds[:, None, 0]) & (xy[None, :, 0] < zone_bounds[:, None, 2])
                      & (xy[None, :, 1] > zone_bounds[:, None, 1]) & (xy[None, :, 1] < zone_bounds[:, None, 3]))
        for z, name in enumerate(zone_names):
            inside = inside_all[z]
            rho = inside.sum() / zone_area[z]
            v = speed[inside]
            v = v[~np.isnan(v)]
            row[f"rho_{name}"] = float(rho)
            row[f"P_{name}"] = float(rho * np.var(v)) if len(v) > 2 else 0.0
            row[f"v_{name}"] = float(np.mean(v)) if len(v) else 0.0
        rows.append(row)

        if frames_every_s and sec % frames_every_s == 0:
            frames.append((t, xy.copy()))

        if controller is not None:
            command = controller(t, row)
        if verbose and sec % 100 == 0:
            print(f"t={t:6.0f}s agents={row['agents']:4d} rho_G={row['rho_G']:.2f} "
                  f"rho_C1={row['rho_C1']:.2f} rho_C2={row['rho_C2']:.2f} cmd={command}")
    return rows, frames


def _in_box(xy, b):
    """Strictly inside an axis-aligned box (minx, miny, maxx, maxy)."""
    return (xy[:, 0] > b[0]) & (xy[:, 0] < b[2]) & (xy[:, 1] > b[1]) & (xy[:, 1] < b[3])


def save_rows(rows, path):
    path = Path(path)
    path.parent.mkdir(parents=True, exist_ok=True)
    with open(path, "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)


def main():
    ap = argparse.ArgumentParser()
    ap.add_argument("--scenario", choices=list(SCENARIOS), default="surge")
    ap.add_argument("--duration", type=float, default=None)
    ap.add_argument("--out", default=None)
    args = ap.parse_args()
    sc = SCENARIOS[args.scenario]
    if args.duration:
        sc.duration_s = args.duration
    rows, _ = run(sc, verbose=True)
    out = args.out or Path(__file__).resolve().parent / "results" / f"{sc.name}_uncontrolled.csv"
    save_rows(rows, out)
    peak = {k: max(r[f"rho_{k}"] for r in rows) for k in NODE_NAMES}
    print("peak density per zone:", {k: round(v, 2) for k, v in peak.items()})
    print("saved", out)


if __name__ == "__main__":
    main()
