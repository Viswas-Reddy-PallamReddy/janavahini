"""Quorum-sensing (QS) decentralised crowd-alert algorithm and baselines.

Each node i keeps an "autoinducer" level A_i, produced from local crowd signals
and leaking over time. Like a signal molecule spreading from a dense colony,
the level a node *receives* from its radio neighbours decays with each hop:

    gradient mode (default):
        A_i(t+1) = delta*A_i(t) + (1-delta) * max(s_i(t), gamma * max_j A_j(t))
    laplacian mode (ablation, consensus-style averaging):
        A_i(t+1) = delta*A_i(t) + (1-delta)*s_i(t) + D * sum_j w_ij (A_j - A_i)

With the (1-delta) scaling, A_i -> s_i for an isolated node, so thresholds stay
in units of "fraction of critical density". In gradient mode a hotspot keeps
its full level and its neighbours see gamma, gamma^2, ... of it: upstream nodes
get an early, graded warning. Averaging (laplacian) instead pulls the hotspot
down towards calmer neighbours, the failure mode noted in the literature
survey; it is kept only as an ablation. gamma < 1 makes the relay a
contraction, so it is stable on any graph and echoes die out.

The production term s_i adds the rate of rise of density (time-to-critical
style, Still 2014) once density is meaningful (>= rate_floor), so a single
person entering an empty zone is not mistaken for a surge.
Stale neighbour values are dropped after `stale_s` seconds (tolerance of
lossy radios, Werner-Allen et al. 2005).
"""

from dataclasses import dataclass, field

import numpy as np

NORMAL, WATCH, CRITICAL = 0, 1, 2
STATE_NAMES = {NORMAL: "NORMAL", WATCH: "WATCH", CRITICAL: "CRITICAL"}


@dataclass
class QSParams:
    rho_crit: float = 4.0       # persons/m^2, Green Guide moving-crowd limit (Still 2000)
    delta: float = 0.85         # leak: memory of past signal per step
    mode: str = "gradient"      # "gradient" (default) or "laplacian" (ablation)
    gamma: float = 0.9          # per-hop decay of the relayed signal; ~3-hop reach (see e5_gamma_sweep)
    D: float = 0.25             # diffusion strength (laplacian mode)
    rate_horizon_s: float = 60  # look-ahead for the rate term (time-to-critical style)
    rate_gain: float = 1.0
    rate_floor: float = 1.5     # persons/m^2; below this the rate term is ignored
    theta_watch: float = 0.6
    theta_crit: float = 0.85
    hysteresis: float = 0.1     # release thresholds sit this far below raise thresholds
    min_dwell_steps: int = 5    # a state must persist this long before it can drop
    stale_s: float = 5.0        # neighbour values older than this are ignored
    rate_window: int = 30       # steps used to estimate d(rho)/dt


def production(rho_hist, p: QSParams, dt: float):
    """Local signal s_i from the density history (most recent last).

    s = rho/rho_crit + gain * max(0, d rho/dt) * horizon / rho_crit
    i.e. "where density will be in `horizon` seconds if the trend holds",
    with the trend term only once rho >= rate_floor.
    """
    rho_now = rho_hist[-1]
    if len(rho_hist) > p.rate_window and rho_now >= p.rate_floor:
        rate = (rho_hist[-1] - rho_hist[-1 - p.rate_window]) / (p.rate_window * dt)
    else:
        rate = 0.0
    return rho_now / p.rho_crit + p.rate_gain * max(0.0, rate) * p.rate_horizon_s / p.rho_crit


class RhoHistory:
    """Density history for all nodes in a ring buffer; vectorised `production`."""

    def __init__(self, n, p: QSParams, dt: float):
        self.p, self.dt = p, dt
        self.buf = np.zeros((p.rate_window + 1, n))
        self.count = 0

    def push(self, rho):
        """Append this step's densities; return the production term s for every node."""
        p = self.p
        w = p.rate_window
        self.buf[self.count % (w + 1)] = rho
        self.count += 1
        rho_now = np.asarray(rho, dtype=float)
        if self.count > w:
            old = self.buf[(self.count - 1 - w) % (w + 1)]
            rate = np.where(rho_now >= p.rate_floor, (rho_now - old) / (w * self.dt), 0.0)
        else:
            rate = np.zeros_like(rho_now)
        return rho_now / p.rho_crit + p.rate_gain * np.maximum(0.0, rate) * p.rate_horizon_s / p.rho_crit


class StateMachine:
    """Three-level alert with hysteresis and minimum dwell (vectorised over nodes)."""

    def __init__(self, n, p: QSParams):
        self.p = p
        self.state = np.full(n, NORMAL)
        self.dwell = np.zeros(n, dtype=int)

    def update(self, level):
        p = self.p
        level = np.asarray(level)
        target = np.where(level >= p.theta_crit, CRITICAL, np.where(level >= p.theta_watch, WATCH, NORMAL))
        raise_ = target > self.state                     # raise immediately
        self.dwell = np.where(raise_, 0, self.dwell + 1)
        release = np.where(self.state == CRITICAL, p.theta_crit, p.theta_watch) - p.hysteresis
        drop = ~raise_ & (target < self.state) & (self.dwell >= p.min_dwell_steps) & (level < release)
        self.state = np.where(raise_, target, np.where(drop, self.state - 1, self.state))
        self.dwell = np.where(drop, 0, self.dwell)
        return self.state.copy()


@dataclass
class QSNetwork:
    """Decentralised QS alert over a radio graph (adjacency matrix)."""

    adjacency: np.ndarray
    params: QSParams = field(default_factory=QSParams)
    dt: float = 1.0

    def __post_init__(self):
        n = len(self.adjacency)
        self.adj = self.adjacency.astype(bool)
        self.A = np.zeros(n)
        self.hist = RhoHistory(n, self.params, self.dt)
        # last value heard from each neighbour and when (rows: receiver, cols: sender)
        self.heard_val = np.zeros((n, n))
        self.heard_age = np.full((n, n), np.inf)
        self.sm = StateMachine(n, self.params)

    def step(self, rho, link_up):
        """rho: density per node; link_up: boolean (n, n) of links working this step."""
        p, n = self.params, len(self.A)
        s = self.hist.push(rho)

        # gossip: receive neighbours' current A over working links
        self.heard_age += self.dt
        recv = self.adj & link_up
        self.heard_val[recv] = np.broadcast_to(self.A, (n, n))[recv]
        self.heard_age[recv] = 0.0
        fresh = self.adj & (self.heard_age <= p.stale_s)

        if p.mode == "gradient":
            relayed = np.where(fresh, self.heard_val, 0.0).max(axis=1) * p.gamma
            new_A = p.delta * self.A + (1 - p.delta) * np.maximum(s, relayed)
        else:
            deg = fresh.sum(axis=1)
            w = np.where(fresh, 1.0 / (1 + np.maximum(deg[:, None], deg[None, :])), 0.0)   # Metropolis
            new_A = p.delta * self.A + (1 - p.delta) * s \
                + p.D * (w * (self.heard_val - self.A[:, None])).sum(axis=1)
        self.A = new_A
        return self.sm.update(self.A), self.A.copy()


class ThresholdBaseline:
    """Current practice: each node alarms on its own density only."""

    def __init__(self, n, params: QSParams, use_rate=False, dt=1.0):
        self.p, self.use_rate, self.dt = params, use_rate, dt
        self.hist = RhoHistory(n, params, dt)
        self.sm = StateMachine(n, params)

    def step(self, rho, link_up=None):
        s = self.hist.push(rho)
        level = s if self.use_rate else np.asarray(rho, dtype=float) / self.p.rho_crit
        return self.sm.update(level), level


class CentralisedBaseline:
    """ICCC-style: all nodes report to one server, which broadcasts alerts back.

    The server sees every node (so it can warn upstream nodes about a
    downstream hotspot), but only while the backhaul is up. When it is down,
    field nodes receive nothing and stay silent.
    """

    def __init__(self, n, params: QSParams, upstream_of, dt=1.0):
        self.p, self.dt = params, dt
        self.upstream_of = upstream_of        # node -> list of nodes that feed it
        self.feeds = np.zeros((n, n), bool)   # feeds[u, i]: u is upstream of i
        for i, us in upstream_of.items():
            self.feeds[us, i] = True
        self.hist = RhoHistory(n, params, dt)
        self.sm = StateMachine(n, params)
        self.last_state = np.full(n, NORMAL)

    def step(self, rho, backhaul_up: bool):
        n = len(rho)
        level = self.hist.push(rho)
        if not backhaul_up:
            return np.full(n, NORMAL), level    # sirens get no command
        # server propagates the worst downstream level to upstream nodes
        prop = np.maximum(level, np.where(self.feeds, level[None, :], -np.inf).max(axis=1))
        self.last_state = self.sm.update(prop)
        return self.last_state.copy(), prop


# actions a node can take on the pilgrim flow it controls
ACT_NONE, ACT_INFORM, ACT_METER, ACT_HOLD = 0, 1, 2, 3
ACTION_NAMES = {ACT_NONE: "NONE", ACT_INFORM: "INFORM", ACT_METER: "METER", ACT_HOLD: "HOLD"}


@dataclass
class RouteParams:
    hold_horizon_s: float = 180.0   # with use_horizon: only feeders within this walk time act; others INFORM
    use_share: bool = True          # weight relayed threat by the route share of each edge
    # off by default: in the site test (E8) holding only near the hotspot jammed the small
    # gates and junctions, while letting the large upstream areas hold did not
    use_horizon: bool = False
    meter: bool = True              # graduated METER (WATCH) / HOLD (CRITICAL) instead of HOLD for both
    # node-failure handling, as in the firmware (firmware/include/pk_qs.h); both off by default
    failsafe_s: float = 0.0         # a successor silent while alarming keeps its last level this long
                                    # (inf = until heard again); 0 = off
    backup: bool = False            # also hear each successor's successors; used only while it is silent
    down_report_s: float = 30.0     # a held/bypassed successor is reported DOWN after this much silence
    # escalate: "unknown is not safe". A successor silent while alarming counts as CRITICAL; one silent
    # for more than down_report_s counts as at least WATCH even if it was calm (it may have died just
    # before its own alarm; E10)
    failsafe_escalate: bool = False


class RouteQSNetwork:
    """QS alerts that travel UPSTREAM along the pilgrim route graph.

    Each node keeps its own leaky level A_i exactly as in QSNetwork (local
    density + rate of rise). It also hears, from each node it sends pilgrims to,
    that node's broadcast threat (level, walk time to its origin). The relayed
    threat over edge i->j is share_ij * level_j: no fixed per-hop decay, only the
    fraction of i's pilgrims that actually go that way. Nodes that do not feed
    the hotspot never hear about it.

    Actions (per node, for the flow it controls):
      relayed threat WATCH    -> METER (partial hold)
      relayed threat CRITICAL -> HOLD
      optional (use_horizon): threat beyond hold_horizon_s walking time -> INFORM only.
    `edge_threat` (per edge, the branch's own level) lets a junction divert to a sibling branch.
    """

    def __init__(self, graph, params: QSParams = None, rp: RouteParams = None, dt=1.0):
        self.p, self.rp, self.dt = params or QSParams(), rp or RouteParams(), dt
        self.u, self.v, self.share, self.walk = graph.edge_arrays()
        self.m0 = len(self.u)                       # primary edges; 2-hop backups follow
        self.via = np.full(self.m0, -1)
        if self.rp.backup:
            bu, bv, bs, bw, bvia = [], [], [], [], []
            for e1 in range(self.m0):
                for e2 in np.nonzero(self.u[:self.m0] == self.v[e1])[0]:
                    bu.append(self.u[e1]); bv.append(self.v[e2])
                    bs.append(self.share[e1] * self.share[e2]); bw.append(self.walk[e1] + self.walk[e2])
                    bvia.append(e1)
            if bu:
                self.u = np.r_[self.u, bu]; self.v = np.r_[self.v, bv]
                self.share = np.r_[self.share, bs]; self.walk = np.r_[self.walk, bw]
                self.via = np.r_[self.via, bvia]
        n, m = graph.n, len(self.u)
        self.n = n
        self.hist = RhoHistory(n, self.p, dt)
        self.A = np.zeros(n)
        self.sm_local = StateMachine(n, self.p)
        self.sm_threat = StateMachine(n, self.p)
        self.heard_L = np.zeros(m)                 # last broadcast heard over each edge (from v)
        self.heard_W = np.zeros(m)
        self.heard_age = np.full(m, np.inf)
        self.best_L = np.zeros(n)                  # what each node broadcasts upstream
        self.best_W = np.zeros(n)
        self.action = np.zeros(n, int)
        self.edge_threat = np.zeros(m)
        self.relayed_L = np.zeros(n)
        self.relayed_W = np.full(n, np.inf)

    def step(self, rho, link_up):
        p, rp = self.p, self.rp
        s = self.hist.push(rho)

        # gossip: u hears its successor v's broadcast from the previous step
        self.heard_age += self.dt
        ok = link_up[self.u, self.v]
        self.heard_L[ok] = self.best_L[self.v[ok]]
        self.heard_W[ok] = self.best_W[self.v[ok]]
        self.heard_age[ok] = 0.0
        fresh = self.heard_age <= p.stale_s
        primary = self.via < 0
        m0 = self.m0
        pe = np.arange(m0)
        w = self.share if rp.use_share else np.ones_like(self.share)
        ok = fresh & primary
        L_eff = self.heard_L.copy()
        branch_L = np.where(fresh, self.heard_L, 0.0)[:m0]          # per-branch level for diversion
        heard_once = np.isfinite(self.heard_age[:m0])
        # fail-safe hold of a silent successor: silent while alarming, or (escalation) long silent
        held = np.zeros(m0, bool)
        held_L = self.heard_L[:m0].copy()
        if rp.failsafe_s > 0:
            silent = ~fresh[:m0] & heard_once & (self.heard_age[:m0] <= rp.failsafe_s)
            alarming = self.heard_L[:m0] >= p.theta_watch
            held = silent & alarming
            if rp.failsafe_escalate:
                long_silent = silent & (self.heard_age[:m0] > rp.down_report_s)
                held_L = np.where(alarming, np.maximum(held_L, p.theta_crit),
                                  np.where(long_silent, np.maximum(held_L, p.theta_watch), held_L))
                held = held | long_silent
        # 2-hop backups of silent successors; a successor both held and bypassed counts as the
        # larger of the two (the backups see beyond it, not the crowd inside it)
        bypassed = np.zeros(m0, bool)
        use_held = held.copy()
        if self.rp.backup and (~primary).any():
            b = np.nonzero(~primary)[0]
            use_b = fresh[b] & ~fresh[self.via[b]]
            bypassed[self.via[b][use_b]] = True
            contrib = w[b] * self.heard_L[b]
            b_sum = np.zeros(m0)
            b_max = np.zeros(m0)
            np.add.at(b_sum, self.via[b][use_b], contrib[use_b])
            np.maximum.at(b_max, self.via[b][use_b], contrib[use_b])
            h = w[:m0] * held_L
            use_held = held & (~bypassed | (h >= (b_sum if rp.use_share else b_max)))
            ok[b] = use_b & ~use_held[self.via[b]]
            np.maximum.at(branch_L, self.via[b][use_b], self.heard_L[b][use_b])
        ok[pe[use_held]] = True
        L_eff[pe[use_held]] = held_L[use_held]
        branch_L = np.where(held, np.maximum(branch_L, held_L), branch_L)
        self.down = held | (bypassed & heard_once)        # silent successors (not just not yet heard)
        # reported to staff only after 30 s of silence (firmware DOWN_REPORT_MS)
        self.down_report = self.down & (self.heard_age[:m0] > rp.down_report_s)
        cand_L = np.where(ok, w * L_eff, 0.0)
        cand_W = self.walk + self.heard_W

        # relayed threat per node. With share weighting it is the share-weighted SUM over
        # branches = "how much of my outflow is heading into trouble" (a junction whose
        # four branches are all crowded hears the full alarm; one of four, a quarter).
        # Without share weighting (ablation) it is the strongest single branch.
        # Walk time: that of the strongest branch (ties: the nearest).
        relayed_L = np.zeros(self.n)
        relayed_W = np.full(self.n, np.inf)
        if len(cand_L):
            order = np.lexsort((cand_W, -cand_L))
            _, first = np.unique(self.u[order], return_index=True)
            pick = order[first]
            relayed_W[self.u[pick]] = cand_W[pick]
            if rp.use_share:
                relayed_L = np.bincount(self.u, weights=cand_L, minlength=self.n)
            else:
                relayed_L[self.u[pick]] = cand_L[pick]

        self.A = p.delta * self.A + (1 - p.delta) * s      # own level stays local
        own = self.A >= relayed_L
        self.best_L = np.where(own, self.A, relayed_L)
        self.best_W = np.where(own, 0.0, relayed_W)

        local = self.sm_local.update(self.A)
        threat = self.sm_threat.update(relayed_L)
        near = relayed_W <= rp.hold_horizon_s if rp.use_horizon else np.ones(self.n, bool)
        act = (threat >= WATCH) & near
        action = np.where(threat >= WATCH, ACT_INFORM, ACT_NONE)
        if rp.meter:
            action = np.where(act & (threat == WATCH), ACT_METER, action)
            action = np.where(act & (threat == CRITICAL), ACT_HOLD, action)
        else:
            action = np.where(act, ACT_HOLD, action)
        # per-branch alarm level for diversion: the branch's own level, not share-weighted
        # per-branch alarm level for diversion (not share-weighted). With node-failure handling
        # on, a silent branch keeps the level it is held or bypassed at, and its room is unknown:
        # callers must not divert into it (edge_room_known False)
        failure_handling = rp.failsafe_s > 0 or rp.backup
        self.action = action
        self.edge_threat = branch_L if failure_handling else np.where(fresh, self.heard_L, 0.0)[:self.m0]
        self.edge_room_known = fresh[:self.m0] if failure_handling else np.ones(self.m0, bool)
        self.relayed_L, self.relayed_W = relayed_L, relayed_W
        # alert state for timing metrics: local alarm, or the relayed threat where this node must act
        return np.maximum(local, np.where(act, threat, NORMAL)), self.A.copy()
