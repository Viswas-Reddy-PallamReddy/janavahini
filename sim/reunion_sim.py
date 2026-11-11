"""PS-03 reunion-time simulation: how long until a separated child (or elder)
is back with the family, under today's practice and under each layer of our
design. Discrete-time (1 min) Monte-Carlo on a grid of ghat zones.

Site: 12 x 5 zones of ~40 m (a 480 m x 200 m stretch of ghats), help desks in
6 zones, one BLE node per zone (detection per zone visit ~90 %, Basalamah2016).
Walking speed in crowd ~0.6 m/s -> ~1.1 min per zone.

Strategies
  A  PA + paper/desk registers (today): every 10-min announcement cycle the
     separated pair is linked with probability p_link_manual.
  B  Band + registry: when the child is brought to any desk its band is scanned
     and linked to the family's report instantly (and vice versa).
  C  B + last-seen trail (BLE band): on report, node buffers give the child's
     last-seen zone; volunteers there and in neighbouring zones search, which
     raises the finding rate.
  D  C + converge-at-desk: at issuance families are told "if separated, go to
     the help desk nearest to where it happened"; finders take the child to the
     desk nearest the separation zone (known from the tag trail).
  E  D + co-walker family zone (families whose phone the tag learned): if the
     child is found while the family's address is still valid (p_loc from
     companion_sim), a volunteer walks the child to the family's current zone.
  F  D + AUTO-FLAG (families whose phone the tag learned): the band flags
     "missing" by itself when its co-walkers vanish and the flag survives the
     confirmation delay (p_auto and delay from companion_sim). Then
       - the station nearest the child dispatches a volunteer to the band's
         live zone (finds the child at rate 1/3 min) -> designated desk;
       - nodes across the region look for the family's learned devices; if
         found (p_loc_esc), the station nearest the PARENTS sends staff / a
         zoned PA there (reaches them w.p. p_reach per 3-min cycle) and tells
         them which desk to go to.
     Without auto-flag, or if the parents are not located, behaviour is as D.

Behavioural assumptions (stated in the proposal; vary them in sensitivity runs)
  parent notices after U(2,10) min; searches nearby U(10,30) min, then reports
  at the nearest desk (in D/E: goes to the designated desk with prob 0.7);
  child stays put or drifts (moves to a neighbouring zone w.p. 0.3/min);
  bystanders bring a lost child to a desk at rate 1/20 min; alerted volunteers
  in a zone find the child at rate 1/4 min.

Usage:  python reunion_sim.py [--runs 3000]
"""

import argparse
import csv
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "results"
COLS, ROWS = 12, 5
DESKS = [(1, 0), (5, 0), (9, 0), (3, 4), (7, 4), (11, 4)]
MIN_PER_ZONE = 40 / 0.6 / 60


def dist(a, b):
    return abs(a[0] - b[0]) + abs(a[1] - b[1])


def nearest_desk(z):
    return min(DESKS, key=lambda d: dist(d, z))


def neighbours(z):
    x, y = z
    out = [(x + dx, y + dy) for dx, dy in ((1, 0), (-1, 0), (0, 1), (0, -1))]
    return [(a, b) for a, b in out if 0 <= a < COLS and 0 <= b < ROWS]


def episode(rng, strategy, has_ble=True, has_phone=False, p_loc=0.0, p_link_manual=0.5,
            p_detect=0.9, compliance=0.7, max_min=360,
            p_auto=0.0, esc_min=5.0, p_loc_esc=0.0, p_reach=0.6, reach_cycle_min=3.0):
    z0 = (int(rng.integers(0, COLS)), int(rng.integers(0, ROWS)))
    child, parent = z0, z0
    t_notice = rng.uniform(2, 10)
    t_search_end = t_notice + rng.uniform(10, 30)
    child_desk = None              # desk where the child is waiting
    parent_goal = None             # desk the parent walks to
    parent_at_desk = None
    reported = False
    alerted = set()
    trail_known = strategy >= "C" and has_ble and rng.random() < p_detect
    designated = nearest_desk(z0)
    goes_designated = strategy >= "D" and rng.random() < compliance
    found_t = None
    auto = strategy == "F" and has_phone and rng.random() < p_auto
    parents_located = auto and rng.random() < p_loc_esc
    volunteer_rate = 1 / 3
    p_reach_min = 1 - (1 - p_reach) ** (1 / reach_cycle_min)

    for t in range(max_min):
        # ---- auto-flag: station actions ----
        auto_active = auto and t >= esc_min
        if auto_active:
            trail_known = True                   # live LOST beacons give the zone
            if child_desk is None:
                alerted = {child} | set(neighbours(child))   # volunteer follows the live zone
            if parents_located and parent_goal is None and rng.random() < p_reach_min:
                parent_goal = designated         # staff near the parents: "go to Desk N"
                t_parent_arrives = t + dist(parent, designated) * MIN_PER_ZONE
        # ---- child ----
        if child_desk is None:
            if rng.random() < 0.3:
                child = neighbours(child)[rng.integers(len(neighbours(child)))]
            rate = 1 / 20 + ((volunteer_rate if auto_active else 1 / 4) if child in alerted else 0)
            if rng.random() < 1 - np.exp(-rate):
                found_t = t
                if strategy == "E" and has_phone and rng.random() < p_loc:
                    # volunteer walks the child to the family's current zone
                    return t + dist(child, parent) * MIN_PER_ZONE
                target = designated if (strategy >= "D" and trail_known) else nearest_desk(child)
                child_desk = target
                t_child_arrives = t + dist(child, target) * MIN_PER_ZONE
        # ---- parent ----
        if parent_at_desk is None:
            if t >= t_notice and goes_designated and parent_goal is None:
                parent_goal = designated
                t_parent_arrives = t + dist(parent, designated) * MIN_PER_ZONE
            elif t >= t_search_end and parent_goal is None:
                parent_goal = nearest_desk(parent)
                t_parent_arrives = t + dist(parent, parent_goal) * MIN_PER_ZONE
            elif parent_goal is None and t >= t_notice and rng.random() < 0.4:
                nb = [n for n in neighbours(parent) if dist(n, z0) <= 1]
                parent = nb[rng.integers(len(nb))] if nb else parent
            if parent_goal is not None and t >= t_parent_arrives:
                parent, parent_at_desk = parent_goal, parent_goal
                reported = True
                if trail_known:                         # last-seen zone -> alert volunteers
                    alerted = {child} | set(neighbours(child))
        # ---- reunion ----
        child_waiting = child_desk is not None and t >= t_child_arrives
        if child_waiting and parent_at_desk == child_desk:
            return float(t)
        if child_waiting and reported:
            # B+: the band scan links child and report instantly; A: announcement cycles
            linked = strategy >= "B" or (t % 10 == 0 and rng.random() < p_link_manual)
            if linked:
                return t + dist(parent_at_desk, child_desk) * MIN_PER_ZONE
    return float(max_min)


def summarise(times):
    t = np.array(times)
    return {"median_min": float(np.median(t)), "p90_min": float(np.percentile(t, 90)),
            "within_30_min_pct": float(100 * np.mean(t <= 30)), "within_60_min_pct": float(100 * np.mean(t <= 60))}


def _batch(task):
    """n episodes with one parameter set and its own seed. `share` (optional): each
    family has a learnable device with this probability."""
    n, seed, kw, share = task
    rng = np.random.default_rng(seed)
    kw = dict(kw)
    strategy = kw.pop("strategy")
    if share is None:
        return [episode(rng, strategy, **kw) for _ in range(n)]
    return [episode(rng, strategy, has_phone=rng.random() < share, **kw) for _ in range(n)]


def escalation_inputs(density=2.0, p_dev=0.5, t_confirm=180):
    """p_auto, escalation delay (min) and p_loc_esc from companion_sim results."""
    path = OUT / "companion_escalation.csv"
    if not path.exists():
        return 0.7, 5.0, 0.5
    with open(path) as f:
        for r in csv.DictReader(f):
            if float(r["density"]) == density and float(r["p_adult_has_device"]) == p_dev \
                    and int(float(r["t_confirm_s"])) == t_confirm:
                return (float(r["auto_detected_pct_of_device_families"]) / 100,
                        float(r["median_escalation_delay_s"]) / 60,
                        float(r["family_locatable_at_escalation_pct"]) / 100)
    raise SystemExit("no matching companion_escalation row")


def main():
    from concurrent.futures import ProcessPoolExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--runs", type=int, default=3000)
    ap.add_argument("--seed", type=int, default=3)
    ap.add_argument("--p-loc", type=float, default=0.1,
                    help="chance the family phone is still locatable when the child is found "
                         "(companion_sim: 0-14 %% depending on density)")
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    configs = [
        ("A: PA + manual registers (today)", "A", False, False),
        ("B: QR band + registry", "B", False, False),
        ("C: BLE band + last-seen trail", "C", True, False),
        ("D: C + converge-at-desk", "D", True, False),
        ("E: D + co-walker family zone (family has phone)", "E", True, True),
    ]
    p_auto, esc_min, p_loc_esc = escalation_inputs()
    print(f"auto-flag inputs (density 2, device share 0.5, confirm 3 min): p_auto={p_auto:.2f} "
          f"delay={esc_min:.1f} min p_loc_esc={p_loc_esc:.2f}")
    configs.append(("F: D + auto-flag + nearest-station action (family has phone)", "F", True, True))
    auto = dict(p_auto=p_auto, esc_min=esc_min, p_loc_esc=p_loc_esc)

    # every batch of episodes is an independent job with its own seed -> run them in parallel
    jobs = [(args.runs, dict(strategy=s, has_ble=b, has_phone=ph, p_loc=args.p_loc, **auto), None)
            for _, s, b, ph in configs]
    shares = [0.3, 0.6, 0.9]
    jobs += [(args.runs, dict(strategy="F", has_ble=True, **auto), sh) for sh in shares]
    fs_cfg = [(tc, pr) for tc in [0, 180, 300] for pr in [0.3, 0.6, 0.9]]
    fs_inputs = {tc: escalation_inputs(t_confirm=tc) for tc in [0, 180, 300]}
    jobs += [(args.runs // 3, dict(strategy="F", has_ble=True, has_phone=True, p_auto=fs_inputs[tc][0],
                                   esc_min=fs_inputs[tc][1], p_loc_esc=fs_inputs[tc][2], p_reach=pr), None)
             for tc, pr in fs_cfg]
    sens_cfg = [(c, pl) for c in [0.3, 0.5, 0.7, 0.9] for pl in [0.3, 0.5, 0.8]]
    for c, pl in sens_cfg:
        jobs += [(args.runs // 3, dict(strategy="A", p_link_manual=pl), None),
                 (args.runs // 3, dict(strategy="D", has_ble=True, compliance=c), None)]
    seeds = np.random.SeedSequence(args.seed).spawn(len(jobs))
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        results = iter(ex.map(_batch, [(n, s, kw, sh) for (n, kw, sh), s in zip(jobs, seeds)]))

        rows = []
        for label, *_ in configs:
            row = {"strategy": label, **summarise(next(results))}
            rows.append(row)
            print({k: (round(v, 1) if isinstance(v, float) else v) for k, v in row.items()})

        # whole population: only families whose devices the band learned can be auto-flagged
        mix = []
        for share in shares:
            mix.append({"share_of_families_with_learnable_device": share, **summarise(next(results))})
            print("population mix", mix[-1])
        fs_results = [next(results) for _ in fs_cfg]
        sens_results = [(next(results), next(results)) for _ in sens_cfg]
    with open(OUT / "reunion_autoflag_population.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(mix[0]))
        w.writeheader()
        w.writerows(mix)

    # sensitivity of F: confirmation delay (via companion_sim) and how readily staff reach parents
    fsens = []
    for (tc, p_reach), times in zip(fs_cfg, fs_results):
        pa, em, pl = fs_inputs[tc]
        fsens.append({"t_confirm_s": tc, "p_auto": pa, "escalation_delay_min": em, "p_loc_esc": pl,
                      "p_reach_per_3min": p_reach, **summarise(times)})
    with open(OUT / "reunion_autoflag_sensitivity.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(fsens[0]))
        w.writeheader()
        w.writerows(fsens)
    print("F sensitivity: median", min(s["median_min"] for s in fsens), "-", max(s["median_min"] for s in fsens))
    with open(OUT / "reunion_time.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    # sensitivity: does the converge-at-desk gain survive pessimistic assumptions?
    sens = []
    for (compliance, p_link), (ta, td) in zip(sens_cfg, sens_results):
        a, d = summarise(ta), summarise(td)
        sens.append({"family_compliance": compliance, "manual_link_prob_per_cycle": p_link,
                     "A_median": a["median_min"], "D_median": d["median_min"],
                     "A_within_30_pct": a["within_30_min_pct"], "D_within_30_pct": d["within_30_min_pct"]})
    with open(OUT / "reunion_sensitivity.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(sens[0]))
        w.writeheader()
        w.writerows(sens)
    print("sensitivity: D median ranges", min(s["D_median"] for s in sens), "-", max(s["D_median"] for s in sens),
          "| A median ranges", min(s["A_median"] for s in sens), "-", max(s["A_median"] for s in sens))

    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    fig, ax = plt.subplots(figsize=(8, 4))
    labels = [r["strategy"].split(":")[0] for r in rows]
    ax.bar(labels, [r["median_min"] for r in rows], label="median")
    ax.scatter(labels, [r["p90_min"] for r in rows], color="black", zorder=3, label="90th percentile")
    ax.set_ylabel("Minutes to reunion")
    ax.set_title("PS-03: time to reunite after separation")
    ax.legend()
    for i, r in enumerate(rows):
        ax.annotate(r["strategy"].split(": ", 1)[1], (i, 2), rotation=90, ha="center", va="bottom", fontsize=8, color="white")
    fig.tight_layout()
    fig.savefig(OUT / "reunion_time.png", dpi=150)


if __name__ == "__main__":
    main()
