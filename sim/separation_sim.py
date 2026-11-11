"""PS-03 Monte-Carlo: child-guardian separation detection in a dense crowd.

Compares a naive RSSI threshold against our filtered state machine
(NEAR -> DRIFTING -> SEPARATED -> LOST, with guardian cancel), and estimates
how often fixed scanners/volunteer phones can give a last-seen zone.

Radio model (parameters from the literature survey):
  RSSI = P0 - 10 n log10(d) - body_loss - X_sigma
  - P0 = -59 dBm @ 1 m, n = 2.0 (outdoor, near free-space when LOS is clear: Cotton2011)
  - each body blocking the line of sight: ~10 dB for the first, +5 dB for the next
    ones (Januszkiewicz2018); occasional deep outdoor body-to-body fades up to 40 dB
    (Cotton2011)
  - log-normal shadowing + fast fading, sigma = 6 dB (Leith2020: confounders as big
    as a ~2 m move)
  - packet loss from advertising collisions at high tag density, ~0.2-0.33
    (Nikodem2019), and phone scan duty cycle (Kindt2022)

Usage:  python separation_sim.py            (writes results/separation_*.csv/png)
"""

import argparse
import csv
from dataclasses import dataclass
from pathlib import Path

import numpy as np

OUT = Path(__file__).resolve().parent / "results"


@dataclass
class Radio:
    p0: float = -59.0          # dBm at 1 m
    n: float = 2.0             # path-loss exponent
    sigma: float = 6.0         # dB, shadowing + fast fading
    sensitivity: float = -95.0
    body_first_db: float = 10.0
    body_next_db: float = 5.0
    body_width_m: float = 0.45
    deep_fade_p: float = 0.03  # chance per packet of a deep body-to-body fade
    deep_fade_db: float = 30.0
    adv_interval_s: float = 0.3
    scan_duty: float = 0.5     # guardian phone/tag scan duty cycle
    collision_p_max: float = 0.3


def rssi_samples(rng, d, rho, radio: Radio, n_pkts):
    """RSSI for n_pkts packets over distance d (m) at crowd density rho (p/m^2).

    Returns an array with np.nan for lost packets.
    """
    return rssi_matrix(rng, np.array([d], dtype=float), rho, radio, n_pkts)[0]


def rssi_matrix(rng, d, rho, radio: Radio, n_pkts):
    """Vectorised rssi_samples: one row of n_pkts packets per distance in d."""
    d = np.maximum(np.asarray(d, dtype=float), 0.3)[:, None]
    shape = (len(d), n_pkts)
    # expected bodies in a corridor of body width between child and guardian
    lam = np.broadcast_to(rho * np.maximum(d - 0.6, 0.0) * radio.body_width_m, shape)
    k = rng.poisson(lam)
    body = np.where(k > 0, radio.body_first_db + radio.body_next_db * np.minimum(k - 1, 6), 0.0)
    deep = (rng.random(shape) < radio.deep_fade_p * min(1.0, rho / 2)) * radio.deep_fade_db
    r = radio.p0 - 10 * radio.n * np.log10(d) - body - deep + rng.normal(0, radio.sigma, shape)
    collision_p = radio.collision_p_max * min(1.0, rho / 4)
    lost = (r < radio.sensitivity) | (rng.random(shape) < collision_p) | (rng.random(shape) > radio.scan_duty)
    r[lost] = np.nan
    return r


def trajectory(rng, duration_s, sep_time_s, dt=1.0):
    """Child-guardian distance over time. Together (0.5-2.5 m, with brief
    excursions) until sep_time_s, then drifting apart at 0.3-0.9 m/s."""
    t = np.arange(0, duration_s, dt)
    d = np.empty_like(t)
    cur = 1.0
    drift = rng.uniform(0.3, 0.9)
    for i, ti in enumerate(t):
        if sep_time_s is not None and ti >= sep_time_s:
            cur = cur + drift * dt + rng.normal(0, 0.3)
        else:
            cur = np.clip(cur + rng.normal(0, 0.4), 0.5, 2.5)
            # child briefly steps a few metres away (normal behaviour, not lost)
            if rng.random() < 0.01:
                cur = rng.uniform(3, 5)
        d[i] = max(cur, 0.3)
    return t, d


@dataclass
class Detector:
    tau_dbm: float = -80.0       # ~10 m equivalent in clear LOS for this radio
    margin_db: float = 12.0      # body-shadowing margin (Januszkiewicz2018)
    drift_persist_s: float = 20  # DRIFTING must persist this long -> SEPARATED
    cancel_window_s: float = 60  # guardian can cancel within this window
    loss_window_s: int = 5
    kalman_q: float = 1.0
    kalman_r: float = 36.0


def naive_detector(windows, tau, avg_s=5):
    """Typical anti-loss-tag logic: alarm when the mean RSSI over the last
    `avg_s` seconds is below tau, or nothing was heard in that span."""
    heard = ~np.isnan(windows)
    cnt = np.cumsum(np.r_[0, heard.sum(axis=1)])
    tot = np.cumsum(np.r_[0.0, np.where(heard, windows, 0.0).sum(axis=1)])
    i = np.arange(len(windows))
    lo = np.maximum(0, i - avg_s + 1)
    n = cnt[i + 1] - cnt[lo]                       # packets heard in the last avg_s seconds
    s = tot[i + 1] - tot[lo]
    with np.errstate(invalid="ignore", divide="ignore"):
        return (n == 0) | (s / n < tau)


def ours_detector(rng, windows, true_d, det: Detector, p_cancel=0.9):
    """Returns per-second state codes: 0 NEAR, 1 DRIFTING, 2 SEPARATED, 3 LOST."""
    x, P = -65.0, 25.0
    states, state, drift_t, sep_t = [], 0, 0.0, None
    recent_heard = []
    for t, w in enumerate(windows):
        heard = w[~np.isnan(w)]
        recent_heard.append(len(heard))
        recent_heard = recent_heard[-det.loss_window_s:]
        P += det.kalman_q
        for z in heard:                        # 1-D Kalman on RSSI
            K = P / (P + det.kalman_r)
            x += K * (z - x)
            P *= (1 - K)
        silent = sum(recent_heard) == 0 and len(recent_heard) == det.loss_window_s
        weak = x < det.tau_dbm - det.margin_db
        if state == 0:
            if weak or silent:
                state, drift_t = 1, 0.0
        elif state == 1:
            if weak or silent:
                drift_t += 1
                if drift_t >= det.drift_persist_s:
                    state, sep_t = 2, t
            else:
                state = 0
        elif state == 2:
            together = true_d[t] < 3.0
            if together and rng.random() < p_cancel / det.cancel_window_s * 3:
                state, drift_t = 0, 0.0          # guardian taps "with me"
            elif not together and rng.random() < 0.7 / 10:
                state = 3                        # guardian confirms "lost" early
            elif t - sep_t >= det.cancel_window_s:
                state = 3                        # no cancel -> escalate
        states.append(state)
    return np.array(states)


def run_episode(rng, rho, radio, det, separate: bool, duration_s=1800):
    sep_time = rng.uniform(300, 1200) if separate else None
    t, d = trajectory(rng, duration_s, sep_time)
    pkts_per_s = int(round(1 / radio.adv_interval_s))
    windows = rssi_matrix(rng, d, rho, radio, pkts_per_s)        # (seconds, packets)
    naive = naive_detector(windows, det.tau_dbm)
    ours = ours_detector(rng, windows, d, det)
    truth_sep = d > 10.0
    res = {}
    # false alarms: alerts while the child is actually close (< 3 m for buzz, < 5 m for LOST)
    close3, close5 = d < 3.0, d < 5.0
    res["naive_false_alarm_events"] = int(np.sum(np.diff(np.r_[0, naive & close3].astype(int)) == 1))
    res["ours_buzz_false_events"] = int(np.sum(np.diff(np.r_[0, (ours >= 2) & close3].astype(int)) == 1))
    res["ours_lost_false_events"] = int(np.sum(np.diff(np.r_[0, (ours == 3) & close5].astype(int)) == 1))
    if separate and truth_sep.any():
        t_true = int(np.argmax(truth_sep))
        after = slice(t_true, None)
        n_idx = np.nonzero(naive[after])[0]
        b_idx = np.nonzero(ours[after] >= 2)[0]
        l_idx = np.nonzero(ours[after] == 3)[0]
        res["naive_delay_s"] = float(n_idx[0]) if len(n_idx) else np.nan
        res["ours_buzz_delay_s"] = float(b_idx[0]) if len(b_idx) else np.nan
        res["ours_lost_delay_s"] = float(l_idx[0]) if len(l_idx) else np.nan
    return res


def sighting_coverage(rng, rho, node_spacing_m, n_phones_per_ha, radio, minutes=5, trials=400):
    """Probability that a lost child (random walk) is heard by at least one fixed
    node or volunteer phone within `minutes` (last-seen zone available)."""
    hits = 0
    area_side = 200.0
    xs = np.arange(node_spacing_m / 2, area_side, node_spacing_m)
    nodes = np.array([(x, y) for x in xs for y in xs])
    n_phones = max(1, int(n_phones_per_ha * (area_side ** 2) / 1e4))
    for _ in range(trials):
        pos = rng.uniform(0, area_side, 2)
        phones = rng.uniform(0, area_side, (n_phones, 2))
        found = False
        for _s in range(minutes * 60):
            pos = np.clip(pos + rng.normal(0, 0.6, 2), 0, area_side)
            phones = np.clip(phones + rng.normal(0, 0.8, phones.shape), 0, area_side)
            scanners = np.vstack([nodes, phones])
            dists = np.linalg.norm(scanners - pos, axis=1)
            near = dists[dists < 40]
            # LOST mode: burst advertising (~10 pkt/s) -> any one packet from any scanner suffices
            if len(near) and np.any(~np.isnan(rssi_matrix(rng, near, rho, radio, 10))):
                found = True
                break
        hits += found
    return hits / trials


def _density(task):
    """All episodes at one crowd density (own seed, so worker count does not matter)."""
    rho, episodes, seed = task
    rng = np.random.default_rng(seed)
    radio, det = Radio(), Detector()
    agg = {k: [] for k in ["naive_delay_s", "ours_buzz_delay_s", "ours_lost_delay_s"]}
    fa = {"naive": 0, "buzz": 0, "lost": 0}
    hours = 0.0
    for e in range(episodes):
        r = run_episode(rng, rho, radio, det, separate=e % 2 == 0)
        hours += 0.5
        fa["naive"] += r["naive_false_alarm_events"]
        fa["buzz"] += r["ours_buzz_false_events"]
        fa["lost"] += r["ours_lost_false_events"]
        for k in agg:
            if k in r:
                agg[k].append(r[k])
    row = {
        "density_p_per_m2": rho,
        "naive_false_alarms_per_family_hour": fa["naive"] / hours,
        "ours_guardian_buzz_false_per_family_hour": fa["buzz"] / hours,
        "ours_network_LOST_false_per_family_hour": fa["lost"] / hours,
        "naive_median_detect_s": float(np.nanmedian(agg["naive_delay_s"])),
        "ours_median_buzz_s": float(np.nanmedian(agg["ours_buzz_delay_s"])),
        "ours_median_LOST_s": float(np.nanmedian(agg["ours_lost_delay_s"])),
        "ours_missed_pct": float(100 * np.mean(np.isnan(agg["ours_buzz_delay_s"]))),
        "family_hours_simulated": hours,
    }
    print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}, flush=True)
    return row


def _coverage(task):
    spacing, phones, seed = task
    c = sighting_coverage(np.random.default_rng(seed), 4.0, spacing, phones, Radio())
    return {"node_spacing_m": spacing, "volunteer_phones_per_ha": phones, "p_last_seen_within_5min": c}


def main():
    from concurrent.futures import ProcessPoolExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--episodes", type=int, default=200)
    ap.add_argument("--seed", type=int, default=7)
    ap.add_argument("--workers", type=int, default=3)
    ap.add_argument("--skip-coverage", action="store_true")
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    densities = [0.5, 2.0, 4.0, 6.0]
    cov_cfg = [(s, p) for s in [30, 50, 80] for p in [0, 5, 20]]
    seeds = np.random.SeedSequence(args.seed).spawn(len(densities) + len(cov_cfg))

    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        rows = list(ex.map(_density, [(rho, args.episodes, s) for rho, s in zip(densities, seeds)]))
        cov_rows = [] if args.skip_coverage else list(
            ex.map(_coverage, [(sp, ph, s) for (sp, ph), s in zip(cov_cfg, seeds[len(densities):])]))

    with open(OUT / "separation_detection.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)

    if cov_rows:
        print(*cov_rows, sep="\n")
        with open(OUT / "sighting_coverage.csv", "w", newline="") as f:
            w = csv.DictWriter(f, fieldnames=list(cov_rows[0]))
            w.writeheader()
            w.writerows(cov_rows)

    plot(rows)


def plot(rows):
    import matplotlib
    matplotlib.use("Agg")
    import matplotlib.pyplot as plt
    rho = [r["density_p_per_m2"] for r in rows]
    fig, ax = plt.subplots(1, 2, figsize=(10, 4))
    ax[0].plot(rho, [r["naive_false_alarms_per_family_hour"] for r in rows], "o-", label="Naive RSSI threshold")
    ax[0].plot(rho, [r["ours_guardian_buzz_false_per_family_hour"] for r in rows], "s-", label="Ours: guardian buzz")
    ax[0].plot(rho, [r["ours_network_LOST_false_per_family_hour"] for r in rows], "^-", label="Ours: network LOST alert")
    ax[0].set_xlabel("Crowd density (persons/m²)")
    ax[0].set_ylabel("False alarms per family-hour")
    ax[0].set_yscale("symlog", linthresh=0.1)
    ax[0].legend()
    ax[0].set_title("False alarms")
    ax[1].plot(rho, [r["naive_median_detect_s"] for r in rows], "o-", label="Naive")
    ax[1].plot(rho, [r["ours_median_buzz_s"] for r in rows], "s-", label="Ours: buzz")
    ax[1].plot(rho, [r["ours_median_LOST_s"] for r in rows], "^-", label="Ours: LOST")
    ax[1].set_xlabel("Crowd density (persons/m²)")
    ax[1].set_ylabel("Median delay after child >10 m away (s)")
    ax[1].legend()
    ax[1].set_title("Detection delay")
    fig.tight_layout()
    fig.savefig(OUT / "separation_detection.png", dpi=150)


if __name__ == "__main__":
    main()
