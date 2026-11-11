"""Macroscopic pilgrim-flow model on a route graph (cell-transmission style).

Used for the site-scale test (68 compartments, 20 junctions, 10 Nagaras), where
an agent-based JuPedSim run of the whole site would be far too slow. Each graph
node is a cell holding N_i pilgrims on area A_i (density rho_i = N_i / A_i).

Per second, for each edge i -> j:
  sending_i   = min(N_i / T_i, w_i * J_cap)            T_i: time to walk through node i
                (compartments: T_i includes the bathing dwell)
  receiving_j = w_j * J_cap * clip((rho_jam - rho_j) / (rho_jam - rho_cap), 0, 1)
  flow        = share_ij * sending_i * control_i, scaled down proportionally when
                the sum of flows into j exceeds receiving_j (back-pressure: the
                surplus stays queued at i and its density rises)
J_cap = 1.22 p/(m s) and rho_jam = 5.4 p/m^2 are Weidmann's values (as calibrated
in calibrate.py); rho_cap = 1.75 p/m^2 is roughly where Weidmann flow peaks.

Controls per node: 1.0 open, METER 0.5, HOLD 0.1 (marshals let a trickle through).
Junctions may DIVERT: shares of threatened branches move to unthreatened siblings.
"""

from dataclasses import dataclass, field

import numpy as np

from route_graph import PEAK_SPECIFIC_FLOW, WALK_SPEED

RHO_JAM, RHO_CAP = 5.4, 1.75
CONTROL = {0: 1.0, 1: 1.0, 2: 0.5, 3: 0.1}      # ACT_NONE, ACT_INFORM, ACT_METER, ACT_HOLD


@dataclass
class SiteScenario:
    name: str = "surge"
    duration_s: int = 3600
    base_rate: float = 6.0             # arrivals per second at each Nagara (60 /s site-wide)
    surge_nagara: int = 3              # this Nagara gets the day-1 surge
    surge_rate: float = 50.0
    surge_start: int = 600
    surge_end: int = 1800
    dwell_s: float = 300.0             # bathing and rituals inside a compartment
    blocked_comp: int | None = None    # compartment whose exit is partly blocked
    blocked_factor: float = 0.02
    blocked_at: int = 300


SCENARIOS = {
    "site-surge": SiteScenario(name="site-surge"),
    "site-blocked": SiteScenario(name="site-blocked", surge_rate=6.0, blocked_comp=30),
}


@dataclass
class SiteFlow:
    graph: object
    sc: SiteScenario
    N: np.ndarray = field(init=False)

    def __post_init__(self):
        g = self.graph
        self.u, self.v, self.base_share, _ = g.edge_arrays()
        self.area = np.array(g.area)
        self.width = np.array(g.width)
        self.kind = np.array(g.kind)
        length = self.area / self.width
        self.T = length / WALK_SPEED + np.where(self.kind == "compartment", self.sc.dwell_s, 0.0)
        self.cap = self.width * PEAK_SPECIFIC_FLOW
        self.N = np.zeros(g.n)
        self.exited = 0.0
        self.sources = np.nonzero(self.kind == "source")[0]
        self.exits = np.nonzero(self.kind == "exit")[0]
        self.comps = np.nonzero(self.kind == "compartment")[0]
        # exit node of each compartment (for the blocked-exit scenario)
        self.comp_exit = {int(c): int(self.v[(self.u == c)][0]) for c in self.comps}

    @property
    def rho(self):
        return self.N / self.area

    def step(self, t, control, share=None):
        """Advance one second. control: per-node multiplier on sending (1 = open).
        share: per-edge shares (after diversion); defaults to the planned shares."""
        sc = self.sc
        share = self.base_share if share is None else share
        # arrivals at the Nagaras
        rate = np.full(len(self.sources), sc.base_rate)
        if sc.surge_start <= t < sc.surge_end:
            rate[sc.surge_nagara] = sc.surge_rate
        self.N[self.sources] += rate

        cap = self.cap.copy()
        if sc.blocked_comp is not None and t >= sc.blocked_at:
            ex = self.comp_exit[int(self.comps[sc.blocked_comp])]
            cap[ex] *= sc.blocked_factor
        sending = np.minimum(self.N / self.T, cap) * control
        rho = self.rho
        receiving = cap * np.clip((RHO_JAM - rho) / (RHO_JAM - RHO_CAP), 0.0, 1.0)
        want = share * sending[self.u]
        into = np.bincount(self.v, weights=want, minlength=len(self.N))
        scale = np.where(into > receiving, receiving / np.maximum(into, 1e-12), 1.0)
        flow = want * scale[self.v]
        self.N -= np.bincount(self.u, weights=flow, minlength=len(self.N))
        self.N += np.bincount(self.v, weights=flow, minlength=len(self.N))
        # exits empty into the town at their capacity
        out = np.minimum(self.N[self.exits], cap[self.exits])
        self.N[self.exits] -= out
        self.exited += out.sum()
        np.maximum(self.N, 0.0, out=self.N)
        return flow


def branch_density_index(graph):
    """Per edge, the node whose density tells whether that branch has room: for a
    junction -> gate edge, the compartment behind the gate; otherwise the edge's target."""
    u, v, _, _ = graph.edge_arrays()
    idx = v.copy()
    for e in range(len(v)):
        if graph.kind[v[e]] == "gate":
            nxt = v[u == v[e]]
            if len(nxt):
                idx[e] = nxt[0]
    return idx


def divert_shares(graph, base_share, edge_threat, branch_rho, theta=0.6, spare_rho=2.0):
    """At junctions, move the share of threatened branches to sibling branches that
    are not threatened and still have room (density behind them < spare_rho)."""
    u, v, _, _ = graph.edge_arrays()
    share = base_share.copy()
    for j in np.unique(u):
        out = np.nonzero(u == j)[0]
        if len(out) < 2:
            continue
        bad = out[edge_threat[out] >= theta]
        ok = out[(edge_threat[out] < 0.5 * theta) & (branch_rho[out] < spare_rho)]
        if len(bad) and len(ok):
            moved = share[bad].sum()
            share[bad] = 0.0
            share[ok] += moved * share[ok] / share[ok].sum() if share[ok].sum() > 0 else moved / len(ok)
    return share
