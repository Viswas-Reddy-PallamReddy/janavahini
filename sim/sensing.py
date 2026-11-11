"""Sensing layer for the site-scale model (E12): what the nodes actually measure.

E1-E11 give every node the true density of its zone ("perfect sensing", report §2.7).
Here each node estimates its density from imperfect sensors instead.

Line counters (one per boundary they cover)
    counted = true crossings x (1 + beta) + noise,   noise ~ N(0, count_noise * sqrt(true))
    beta is a fixed bias per counter, drawn once per run (beta ~ N(0, count_bias_sd)).
    A counter on edge i -> j is shared: it is "out" for i and "in" for j.
    Placement:
      "all"     every route edge, every Nagara entry and every exit (the idealised layout)
      "narrow"  only boundaries at most narrow_width_m wide: junction -> gate, gate ->
                compartment, compartment -> exit, and exit outflow. Wide roads, junctions
                and Nagara entries (25-40 m) cannot be line-counted without lanes.
      "none"
Density sensor (an absolute reading: edge camera wired to the node, or RF link attenuation)
    every obs_every_s seconds (staggered per node):
    z = rho x (1 + gamma_i) x (1 + eps),   gamma_i ~ N(0, obs_bias_sd) fixed per node,
                                           eps ~ N(0, obs_rel_sd), plus N(0, obs_abs_sd)
    missing with probability obs_dropout. Placement: "all", "uncounted" (only nodes the
    counters do not fully cover) or "none".

Estimator, per node (scalar operations only, so it ports to the firmware header):
    a 2-state Kalman filter, x = [rho, b], b = drift of the counted net flux (persons/m^2/s)
      predict   rho <- rho + (in_counted - out_counted) / A - b ;   b <- b
      update    with z when a reading arrives: corrects rho (kills accumulated drift) and
                learns b, so the counters stay corrected between readings
    Nodes whose boundaries are not all counted have no flux model: rho is a random walk,
    corrected by z (a first-order filter on the density sensor).
    Nodes with neither sensor are blind: they report 0.
"""

from dataclasses import asdict, dataclass

import numpy as np


@dataclass(frozen=True)
class SensingConfig:
    name: str = "truth"
    truth: bool = False                 # bypass everything: the node sees the true density
    counters: str = "all"               # "all" | "narrow" | "none"
    narrow_width_m: float = 6.0
    count_bias_sd: float = 0.02         # per-counter systematic bias (2 % = a good commercial counter)
    count_noise: float = 0.3            # random miscounts, x sqrt(crossings) per second
    density: str = "all"                # "all" | "uncounted" | "none"
    obs_every_s: int = 60
    obs_rel_sd: float = 0.10            # 10 %: an edge camera; RF attenuation is noisier
    obs_bias_sd: float = 0.05
    obs_abs_sd: float = 0.05
    obs_dropout: float = 0.0
    b_walk_sd: float = 2e-5             # how fast the counter drift may change (p/m^2/s per sqrt(s))
    rw_sd: float = 0.01                 # density change between readings at uncounted nodes (p/m^2/s)

    def as_dict(self):
        return asdict(self)


class Sensing:
    def __init__(self, graph, cfg: SensingConfig, seed: int):
        self.cfg = cfg
        u, v, _, _ = graph.edge_arrays()
        self.u, self.v = np.asarray(u), np.asarray(v)
        self.n = graph.n
        self.area = np.asarray(graph.area, float)
        width = np.asarray(graph.width, float)
        kind = np.asarray(graph.kind)
        rng = np.random.default_rng([seed, 12])          # independent of the link-loss stream
        self.rng = rng

        m = len(self.u)
        src = kind == "source"
        ext = kind == "exit"
        if cfg.counters == "all":
            self.c_edge = np.ones(m, bool)
            self.c_src, self.c_exit = src.copy(), ext.copy()
        elif cfg.counters == "narrow":
            bw = np.minimum(width[self.u], width[self.v])
            self.c_edge = bw <= cfg.narrow_width_m
            self.c_src = src & (width <= cfg.narrow_width_m)
            self.c_exit = ext & (width <= cfg.narrow_width_m)
        else:
            self.c_edge = np.zeros(m, bool)
            self.c_src = self.c_exit = np.zeros(self.n, bool)
        # a node has a flux model only if every one of its boundaries is counted
        uncounted_in = np.bincount(self.v, weights=~self.c_edge, minlength=self.n) > 0
        uncounted_out = np.bincount(self.u, weights=~self.c_edge, minlength=self.n) > 0
        self.counted = ~uncounted_in & ~uncounted_out & (~src | self.c_src) & (~ext | self.c_exit)
        if cfg.counters == "none":
            self.counted[:] = False
        if cfg.density == "all":
            self.observed = np.ones(self.n, bool)
        elif cfg.density == "uncounted":
            self.observed = ~self.counted
        else:
            self.observed = np.zeros(self.n, bool)
        self.blind = ~self.counted & ~self.observed

        self.beta_e = rng.normal(0, cfg.count_bias_sd, m)
        self.beta_src = rng.normal(0, cfg.count_bias_sd, self.n)
        self.beta_exit = rng.normal(0, cfg.count_bias_sd, self.n)
        self.gamma = rng.normal(0, cfg.obs_bias_sd, self.n)
        self.phase = rng.integers(0, max(1, cfg.obs_every_s), self.n)

        # Kalman state: the site starts empty and that is known
        self.rho = np.zeros(self.n)
        self.b = np.zeros(self.n)
        self.P00 = np.full(self.n, 1e-4)
        self.P01 = np.zeros(self.n)
        self.P11 = np.full(self.n, (5 * cfg.b_walk_sd) ** 2)

    def _count(self, true, beta, active):
        c = true * (1 + beta) + self.rng.normal(0, 1, true.shape) * self.cfg.count_noise * np.sqrt(np.maximum(true, 0))
        return np.where(active, np.maximum(c, 0.0), 0.0)

    def update(self, t, edge_flow, arrivals, exit_out, rho_true):
        """One second. edge_flow, arrivals, exit_out: the true crossings of the step that just
        ran; rho_true: the true density after it. Returns the estimate the nodes report."""
        cfg = self.cfg
        if cfg.truth:
            self.rho = np.asarray(rho_true, float).copy()
            return self.rho
        n = self.n
        # ---- predict
        ce = self._count(edge_flow, self.beta_e, self.c_edge)
        cin = np.bincount(self.v, weights=ce, minlength=n) + self._count(arrivals, self.beta_src, self.c_src)
        cout = np.bincount(self.u, weights=ce, minlength=n) + self._count(exit_out, self.beta_exit, self.c_exit)
        k = self.counted
        a = self.area
        q_rho = (cfg.count_noise ** 2 * (cin + cout) + 1e-6) / a ** 2
        self.rho = np.where(k, self.rho + (cin - cout) / a - self.b, self.rho)
        P00 = np.where(k, self.P00 - 2 * self.P01 + self.P11 + q_rho, self.P00 + cfg.rw_sd ** 2)
        P01 = np.where(k, self.P01 - self.P11, 0.0)
        P11 = np.where(k, self.P11 + cfg.b_walk_sd ** 2, self.P11)
        # ---- correct, where a density reading arrives
        due = self.observed & ((t - self.phase) % max(1, cfg.obs_every_s) == 0)
        if cfg.obs_dropout > 0:
            due &= self.rng.random(n) >= cfg.obs_dropout
        if due.any():
            z = (np.asarray(rho_true) * (1 + self.gamma) * (1 + self.rng.normal(0, cfg.obs_rel_sd, n))
                 + self.rng.normal(0, cfg.obs_abs_sd, n))
            z = np.maximum(z, 0.0)
            R = (cfg.obs_rel_sd * np.maximum(z, 0.5)) ** 2 + cfg.obs_abs_sd ** 2
            S = P00 + R
            K0, K1 = P00 / S, P01 / S
            y = z - self.rho
            self.rho = np.where(due, self.rho + K0 * y, self.rho)
            self.b = np.where(due & k, self.b + K1 * y, self.b)
            nP00 = (1 - K0) * P00
            nP01 = (1 - K0) * P01
            nP11 = P11 - K1 * P01
            P00 = np.where(due, nP00, P00)
            P01 = np.where(due, nP01, P01)
            P11 = np.where(due, nP11, P11)
        self.P00, self.P01, self.P11 = P00, P01, P11
        self.rho = np.where(self.blind, 0.0, np.maximum(self.rho, 0.0))
        return self.rho


# The sensing set-ups compared in E12 (sensing_experiment.py)
CONFIGS = {
    "truth": SensingConfig(name="truth", truth=True),
    # counters everywhere, no absolute reading: in - out dead reckoning (the review's "ghost crowd")
    "counters-2%": SensingConfig(name="counters-2%", counters="all", density="none", count_bias_sd=0.02),
    "counters-5%": SensingConfig(name="counters-5%", counters="all", density="none", count_bias_sd=0.05),
    # counters everywhere + a camera reading every 60 s, fused by the Kalman filter
    "counters-5%+cam60": SensingConfig(name="counters-5%+cam60", counters="all", density="all",
                                       count_bias_sd=0.05, obs_every_s=60),
    # realistic placement: counters only at gates/exits; edge cameras everywhere, every 10 s
    "narrow-2%+cam10": SensingConfig(name="narrow-2%+cam10", counters="narrow", density="all",
                                     count_bias_sd=0.02, obs_every_s=10),
    "narrow-5%+cam10": SensingConfig(name="narrow-5%+cam10", counters="narrow", density="all",
                                     count_bias_sd=0.05, obs_every_s=10),
    # same, but the absolute reading is RF link attenuation: noisier, more biased, gaps
    "narrow-2%+rf10": SensingConfig(name="narrow-2%+rf10", counters="narrow", density="all", count_bias_sd=0.02,
                                    obs_every_s=10, obs_rel_sd=0.25, obs_bias_sd=0.10, obs_dropout=0.2),
    # cameras only (no counters), every 10 s
    "cam10-only": SensingConfig(name="cam10-only", counters="none", density="all", obs_every_s=10),
    # counters at gates/exits only and nothing else: roads, junctions and Nagaras are blind
    "narrow-only": SensingConfig(name="narrow-only", counters="narrow", density="none", count_bias_sd=0.02),
}
