"""PS-03 co-walker inference: can a child's tag learn which nearby devices
belong to its family (who wear nothing), and use that to detect separation
and to locate the family afterwards?

Model
- A family (child + 1-3 adults) walks together through a crowd; each adult
  carries a Bluetooth-advertising device with probability p_dev (phone with
  Bluetooth on, earbuds, smartwatch). Feature phones / no device = invisible.
- Bystanders fill a 60 m x 60 m window around the child at crowd density rho;
  a fraction p_bystander of them advertise. They move independently.
- Every device rotates its BLE address every 15 min (Android CDD requirement /
  Bluetooth SIG default), at a random phase.
- The tag scans 1 s in every 5 s (radio model as in separation_sim.py) and
  keeps an exponentially-weighted companion score per address:
      score <- a*score + (1-a)*[heard with RSSI >= rssi_near]
  Imprinting: devices within 2 m when the band is issued start at 0.8.
  Rotation re-link: when a companion address vanishes and a new address
  appears in the same scan with RSSI within 6 dB, the score transfers.
- Possible separation: the tag had a stable companion set and hears none of
  it for `sep_after_s`.
- Family locate: when the child is found (5-20 min after separating), is a
  companion address still valid (not yet rotated), so nodes can search for it?

Variant "fixed-node": the tag only beacons; nodes record which addresses share
the child's node zone (~30 m) each minute. Approximated by counting any address
heard (no proximity threshold) at 1-min resolution.

Usage:  python companion_sim.py [--runs 30]
"""

import argparse
import csv
from pathlib import Path

import numpy as np

from separation_sim import Radio

OUT = Path(__file__).resolve().parent / "results"
SCAN_EVERY_S = 5
ROTATE_S = 900
WINDOW = 60.0          # m, local window simulated around the child


def rssi_max(rng, d, rho, radio: Radio, n_pkts=2):
    """Vectorised radio model: strongest received RSSI per device over n_pkts
    packets, NaN if none received. Same model as separation_sim.rssi_samples."""
    d = np.maximum(d, 0.3)[:, None]
    lam = rho * np.maximum(d - 0.6, 0.0) * radio.body_width_m
    k = rng.poisson(np.broadcast_to(lam, (len(d), n_pkts)))
    body = np.where(k > 0, radio.body_first_db + radio.body_next_db * np.minimum(k - 1, 6), 0.0)
    deep = (rng.random(k.shape) < radio.deep_fade_p * min(1.0, rho / 2)) * radio.deep_fade_db
    r = radio.p0 - 10 * radio.n * np.log10(d) - body - deep + rng.normal(0, radio.sigma, k.shape)
    lost = (r < radio.sensitivity) | (rng.random(k.shape) < radio.collision_p_max * min(1.0, rho / 4)) \
        | (rng.random(k.shape) > radio.scan_duty)
    best = np.where(lost, -np.inf, r).max(axis=1)
    best[np.isneginf(best)] = np.nan                   # nothing received from this device
    return best


N_PAYLOAD_CLASSES = 30   # advertising-payload "shape" (OS, adv types, TX power) that survives rotation


def run(rng, rho, p_dev, p_bystander, variant="tag", duration_s=7200, sep_time_s=None,
        rssi_near=-80.0, alpha_window_s=600, sep_after_s=120, comp_threshold=0.4, strong=0.6,
        arm_after_s=600, radio=None):
    radio = radio or Radio(adv_interval_s=0.5, scan_duty=1.0, collision_p_max=0.2)
    n_adults = int(rng.integers(1, 4))
    n_fam = int((rng.random(n_adults) < p_dev).sum())
    n_by = int(rho * WINDOW * WINDOW * p_bystander)
    n = n_fam + n_by
    is_fam = np.zeros(n, bool)
    is_fam[:n_fam] = True
    pos = np.vstack([rng.uniform(-1.5, 1.5, (n_fam, 2)), rng.uniform(-WINDOW / 2, WINDOW / 2, (n_by, 2))])
    ang = rng.uniform(0, 2 * np.pi, n)
    vel = np.c_[np.cos(ang), np.sin(ang)] * 0.7
    vel[is_fam] = 0
    phase = rng.uniform(0, ROTATE_S, n)
    pclass = rng.integers(0, N_PAYLOAD_CLASSES, n)
    addr = np.arange(n)
    next_addr = n
    addr_class = {int(i): int(pclass[i]) for i in range(n)}
    remembered = set()        # last stable companion set (kept after separation for the family search)
    comp_since = {}           # address -> time it became a companion (carried over on re-link)

    a = 1 - SCAN_EVERY_S / alpha_window_s
    score, last_rssi = {}, {}
    for i in np.nonzero(np.linalg.norm(pos, axis=1) < 2.0)[0]:     # imprinting at issuance
        score[int(addr[i])] = 0.8

    sep = sep_time_s
    fam_offset, fam_vel = np.zeros(2), None
    stable_since, last_comp_heard = None, 0.0
    sep_flag_time, false_flags, flagged = None, 0, False
    prec, rec = [], []
    locate_ok = None
    found_time = sep + rng.uniform(300, 1200) if sep is not None else None
    minute_presence = {}
    episodes = []             # [start, end, together_at_start] of each separation-flag episode
    loc_valid = []

    for t in np.arange(0, duration_s, SCAN_EVERY_S):
        together = sep is None or t < sep
        # --- movement (positions relative to the child) ---
        if together:
            pos[is_fam] = np.clip(pos[is_fam] + rng.normal(0, 0.3, (n_fam, 2)), -2.5, 2.5)
        else:
            if fam_vel is None:
                g = rng.uniform(0, 2 * np.pi)
                fam_vel = 0.6 * np.array([np.cos(g), np.sin(g)])
            fam_offset = fam_offset + fam_vel * SCAN_EVERY_S
            pos[is_fam] += rng.normal(0, 0.1, (n_fam, 2))
        by = ~is_fam
        pos[by] = (pos[by] + vel[by] * SCAN_EVERY_S + rng.normal(0, 0.5, (n_by, 2)) + WINDOW / 2) % WINDOW - WINDOW / 2
        # --- address rotation ---
        rot = ((t + phase) % ROTATE_S) < SCAN_EVERY_S
        k = int(rot.sum())
        addr[rot] = np.arange(next_addr, next_addr + k)
        for i in np.nonzero(rot)[0]:
            addr_class[int(addr[i])] = int(pclass[i])
        next_addr += k
        # --- scan ---
        eff = pos + np.where(is_fam[:, None], fam_offset, 0.0)
        dist = np.linalg.norm(eff, axis=1)
        cand = np.nonzero(dist <= 45)[0]
        r = rssi_max(rng, dist[cand], rho, radio)
        ok = ~np.isnan(r)
        heard = {int(addr[i]): float(v) for i, v in zip(cand[ok], r[ok])}

        if variant == "tag":
            appeared = [ad for ad in heard if ad not in score and ad not in last_rssi]
            for ad in [ad for ad, s in score.items() if s >= comp_threshold and ad not in heard and ad in last_rssi]:
                # rotation re-link: a brand-new address, same payload class, similar RSSI
                cands = [x for x in appeared if addr_class[x] == addr_class[ad]
                         and abs(heard[x] - last_rssi[ad]) <= 6.0]
                if len(cands) == 1:                               # only when unambiguous
                    score[cands[0]] = score[ad]
                    if ad in comp_since:
                        comp_since[cands[0]] = comp_since[ad]
                    if ad in remembered:
                        remembered.add(cands[0])
                    appeared.remove(cands[0])
            for ad in list(score):
                score[ad] = a * score[ad] + (1 - a) * (ad in heard and heard[ad] >= rssi_near)
                if score[ad] < 0.01 and ad not in heard:
                    del score[ad]
            for ad, v in heard.items():
                if ad not in score and v >= rssi_near:
                    score[ad] = 1 - a
            last_rssi = heard
            companions = {ad for ad, s in score.items() if s >= comp_threshold}
            for ad in companions:
                comp_since.setdefault(ad, t)
            for ad in list(comp_since):
                if ad not in score:
                    del comp_since[ad]
        else:
            minute = int(t // 60)
            for ad in heard:
                minute_presence.setdefault(ad, set()).add(minute)
            recent = set(range(max(0, minute - 10), minute + 1))
            need = 0.8 * len(recent)
            companions = {ad for ad, ms in minute_presence.items() if len(ms & recent) >= need}

        fam_addrs = set(int(x) for x in addr[is_fam])
        if t >= 600 and together:
            if companions:
                prec.append(len(companions & fam_addrs) / len(companions))
            if fam_addrs:
                rec.append(len(companions & fam_addrs) / len(fam_addrs))
        if companions & heard.keys():
            last_comp_heard = t
            if variant == "tag":
                # armed only by companions that have stayed >= arm_after_s (strangers rarely do)
                strong_set = {ad for ad in companions if score[ad] >= strong and t - comp_since[ad] >= arm_after_s}
                remembered = strong_set or remembered
            else:
                remembered = set(companions)
        armed = remembered if variant == "tag" else (companions or remembered)
        stable_since = (stable_since if stable_since is not None else t) if armed else None
        watch = companions | remembered
        silent = stable_since is not None and t - stable_since >= 300 and t - last_comp_heard >= sep_after_s \
            and not (watch & heard.keys())
        if silent and not flagged:
            flagged = True
            episodes.append([t, None, together])
            if together:
                false_flags += 1
            elif sep_flag_time is None:
                sep_flag_time = t - sep
        if not silent and flagged:
            episodes[-1][1] = t
            flagged = False
        # is a remembered family address still valid (not rotated) right now?
        loc_valid.append(bool(remembered & fam_addrs))
        if found_time is not None and locate_ok is None and t >= found_time:
            # nodes can search for a remembered family address only if it has not rotated since
            locate_ok = bool(remembered & fam_addrs)
    hours_together = (sep if sep is not None else duration_s) / 3600
    return {"has_device": n_fam > 0,
            "precision": float(np.mean(prec)) if prec else np.nan,
            "recall": float(np.mean(rec)) if rec else np.nan,
            "false_sep_flags_per_h": false_flags / hours_together,
            "sep_detect_delay_s": sep_flag_time,
            "family_locatable_when_found": locate_ok,
            "episodes": episodes, "loc_valid": loc_valid, "sep": sep, "duration": duration_s,
            "hours_together": hours_together}


def escalation(r, t_confirm, node_detect=0.9, rng=None):
    """Two-stage escalation applied to one run: a flag only becomes a station
    action if it persists t_confirm seconds without the companions returning.
    Returns (false escalations, detection delay or None, family locatable at escalation)."""
    false_esc, delay, loc = 0, None, None
    for start, end, together in r["episodes"]:
        lasted = (end if end is not None else r["duration"]) - start
        if lasted < t_confirm:
            continue
        if together:
            false_esc += 1
        elif delay is None:
            delay = start + t_confirm - r["sep"]
            k = min(int((start + t_confirm) // SCAN_EVERY_S), len(r["loc_valid"]) - 1)
            loc = r["loc_valid"][k] and (rng is None or rng.random() < node_detect)
    return false_esc, delay, loc


def _config(task):
    """All runs of one (variant, density, device share) configuration, with its own
    seed so results do not depend on how many worker processes are used."""
    variant, rho, p_dev, runs, seed = task
    rng = np.random.default_rng(seed)
    res = [run(rng, rho, p_dev, 0.3, variant, sep_time_s=float(rng.uniform(1800, 4800)))
           for _ in range(runs)]
    esc_rows = []
    if variant == "tag":
        hours_all = sum(r["hours_together"] for r in res)
        for tc in [0, 180, 300]:
            out = [escalation(r, tc, rng=rng) for r in res]
            wd_out = [o for o, r in zip(out, res) if r["has_device"]]
            dets = [o[1] for o in wd_out if o[1] is not None]
            locs = [o[2] for o in wd_out if o[1] is not None]
            fe = sum(o[0] for o in out) / hours_all
            esc_rows.append({
                "density": rho, "p_adult_has_device": p_dev, "t_confirm_s": tc,
                "families_with_device_pct": 100 * len(wd_out) / len(res),
                "auto_detected_pct_of_device_families": 100 * len(dets) / len(wd_out) if wd_out else np.nan,
                "median_escalation_delay_s": float(np.median(dets)) if dets else np.nan,
                "false_escalations_per_family_hour_all_families": fe,
                "volunteer_checks_per_1000_bands_per_hour": 1000 * fe,
                "family_locatable_at_escalation_pct": 100 * float(np.mean(locs)) if locs else np.nan})
    wd = [r for r in res if r["has_device"]]
    det = [r["sep_detect_delay_s"] for r in wd if r["sep_detect_delay_s"] is not None]
    loc = [r["family_locatable_when_found"] for r in wd if r["family_locatable_when_found"] is not None]
    mean = lambda key: float(np.nanmean([r[key] for r in wd])) if wd else np.nan  # noqa: E731
    row = {"variant": variant, "density": rho, "p_adult_has_device": p_dev,
           "families_with_detectable_device_pct": 100 * len(wd) / len(res),
           "companion_precision": mean("precision"), "companion_recall": mean("recall"),
           "separation_detected_pct": 100 * len(det) / len(wd) if wd else np.nan,
           "median_detect_delay_s": float(np.median(det)) if det else np.nan,
           "false_flags_per_family_hour": mean("false_sep_flags_per_h"),
           "false_flags_per_hour_no_device_families": float(np.mean(
               [r["false_sep_flags_per_h"] for r in res if not r["has_device"]] or [np.nan])),
           "family_locatable_pct": 100 * float(np.mean(loc)) if loc else np.nan}
    print({k: (round(v, 2) if isinstance(v, float) else v) for k, v in row.items()}, flush=True)
    return row, esc_rows


def main():
    from concurrent.futures import ProcessPoolExecutor
    ap = argparse.ArgumentParser()
    ap.add_argument("--runs", type=int, default=30)
    ap.add_argument("--seed", type=int, default=11)
    ap.add_argument("--workers", type=int, default=3)
    args = ap.parse_args()
    OUT.mkdir(exist_ok=True)
    configs = [(v, rho, p) for v in ["tag", "fixed-node"] for rho in [0.5, 2.0, 4.0] for p in [0.2, 0.5, 0.8]]
    seeds = np.random.SeedSequence(args.seed).spawn(len(configs))
    tasks = [(v, rho, p, args.runs, s) for (v, rho, p), s in zip(configs, seeds)]
    with ProcessPoolExecutor(max_workers=args.workers) as ex:
        results = list(ex.map(_config, tasks))
    rows = [r for r, _ in results]
    esc_rows = [e for _, es in results for e in es]
    with open(OUT / "companion_inference.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(rows[0]))
        w.writeheader()
        w.writerows(rows)
    with open(OUT / "companion_escalation.csv", "w", newline="") as f:
        w = csv.DictWriter(f, fieldnames=list(esc_rows[0]))
        w.writeheader()
        w.writerows(esc_rows)


if __name__ == "__main__":
    main()
