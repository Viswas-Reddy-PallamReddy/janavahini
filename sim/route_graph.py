"""Directed pilgrim-route graph: road segments, junctions, gates, compartments, exits.

Alerts follow this graph (upstream along the arrows) instead of plain radio
distance, so a crowded ghat only warns the places that actually feed it.

Edge attributes
  share   fraction of the upstream node's outflow that takes this edge
          (junction splits; planned values, later updated from gate counters)
  walk_s  walking time along the edge at free speed
Node attributes
  area_m2, width_m (sets flow capacity = width x Weidmann peak flow 1.22 p/(m s)),
  kind (source / road / junction / gate / compartment / exit), position (m)

Builders
  compartment_graph()   the 6-zone JuPedSim compartment (R1-R2-G-C1-C2-X)
  synthetic_site()      68 compartments, 20 junctions, 10 Nagaras (entry areas)
"""

from dataclasses import dataclass, field

import numpy as np

PEAK_SPECIFIC_FLOW = 1.22     # persons/(m s), Weidmann capacity (calibrate.weidmann_capacity)
WALK_SPEED = 1.2              # m/s, free walking speed used for edge walk times


@dataclass
class RouteGraph:
    names: list = field(default_factory=list)
    kind: list = field(default_factory=list)
    area: list = field(default_factory=list)
    width: list = field(default_factory=list)
    pos: list = field(default_factory=list)
    edges: list = field(default_factory=list)      # (u, v, share, walk_s)

    def add_node(self, name, kind, area_m2, width_m, pos):
        self.names.append(name)
        self.kind.append(kind)
        self.area.append(float(area_m2))
        self.width.append(float(width_m))
        self.pos.append(tuple(pos))
        return len(self.names) - 1

    def add_edge(self, u, v, share=1.0, walk_s=None):
        if walk_s is None:
            walk_s = float(np.hypot(*np.subtract(self.pos[v], self.pos[u]))) / WALK_SPEED
        self.edges.append((u, v, float(share), float(walk_s)))

    # ---- arrays used by the simulation code ----
    @property
    def n(self):
        return len(self.names)

    def edge_arrays(self):
        e = np.array(self.edges, dtype=float).reshape(-1, 4)
        return e[:, 0].astype(int), e[:, 1].astype(int), e[:, 2], e[:, 3]

    def index(self, name):
        return self.names.index(name)

    def capacity(self):
        """Outflow capacity per node, persons/s."""
        return np.array(self.width) * PEAK_SPECIFIC_FLOW

    def radio_adjacency(self):
        """Nodes along a route are within radio range of each other (undirected)."""
        a = np.zeros((self.n, self.n), dtype=int)
        for u, v, _, _ in self.edges:
            a[u, v] = a[v, u] = 1
        return a

    def upstream(self, target):
        """Every node that feeds `target`, with the fraction of its outflow that
        reaches it (sum over paths of the product of shares) and the shortest walk
        time to it. Returns {node: (share, walk_s)}."""
        u, v, share, walk = self.edge_arrays()
        out = {target: (1.0, 0.0)}
        frontier = [target]
        while frontier:
            nxt = []
            for t in frontier:
                s_t, w_t = out[t]
                for k in np.nonzero(v == t)[0]:
                    src = int(u[k])
                    s_new = share[k] * s_t
                    w_new = walk[k] + w_t
                    if src in out:
                        s_old, w_old = out[src]
                        out[src] = (s_old + s_new, min(w_old, w_new))
                    else:
                        out[src] = (s_new, w_new)
                    nxt.append(src)
            frontier = nxt
        out.pop(target)
        return out

    def upstream_map(self):
        """{node: [upstream nodes]} for the centralised baseline."""
        return {i: list(self.upstream(i)) for i in range(self.n)}


def compartment_graph():
    """The JuPedSim compartment as a route graph (ghat_scenario zone names and
    dimensions; walk times from zone centres)."""
    g = RouteGraph()
    r1 = g.add_node("R1", "road", 240, 8.0, (15, 4))
    r2 = g.add_node("R2", "road", 240, 8.0, (45, 4))
    gt = g.add_node("G", "gate", 64, 3.0, (58, 4))
    c1 = g.add_node("C1", "compartment", 300, 20.0, (69.5, 4))
    c2 = g.add_node("C2", "compartment", 300, 20.0, (84.5, 4))
    x = g.add_node("X", "exit", 16, 2.0, (81, 18))
    for a, b in [(r1, r2), (r2, gt), (gt, c1), (c1, c2), (c2, x)]:
        g.add_edge(a, b)
    return g


def synthetic_site(seed=0):
    """A Pushkaralu-scale site: 10 Nagaras (entry areas) -> 10 approach roads ->
    20 junctions -> 68 compartments (gate + compartment) -> exits.
    Each Nagara feeds two junctions (50/50); 8 junctions feed 4 compartments and
    12 feed 3 (equal shares). Compartments lie along a 6.8 km river bank."""
    rng = np.random.default_rng(seed)
    g = RouteGraph()
    n_comp = 68
    sizes = [4] * 8 + [3] * 12
    rng.shuffle(sizes)
    comp_x = np.arange(n_comp) * 100.0 + 50          # 100 m of river bank per compartment
    junctions, comps = [], []
    c = 0
    for j, k in enumerate(sizes):
        jx = comp_x[c:c + k].mean()
        jn = g.add_node(f"J{j:02d}", "junction", 1000, 25.0, (jx, 250))
        junctions.append(jn)
        for _ in range(k):
            # exit narrower than the gate, as at real ghats: a surge can build up inside
            gate = g.add_node(f"G{c:02d}", "gate", 60, 5.0, (comp_x[c], 120))
            cp = g.add_node(f"C{c:02d}", "compartment", 600, 20.0, (comp_x[c], 60))
            ex = g.add_node(f"X{c:02d}", "exit", 40, 3.0, (comp_x[c] + 30, 120))
            g.add_edge(jn, gate, share=1.0 / k)
            g.add_edge(gate, cp)
            g.add_edge(cp, ex)
            comps.append(cp)
            c += 1
    for s in range(10):
        j_a, j_b = junctions[2 * s], junctions[2 * s + 1]
        nx = (g.pos[j_a][0] + g.pos[j_b][0]) / 2
        nag = g.add_node(f"N{s}", "source", 20000, 40.0, (nx, 1200))
        road = g.add_node(f"R{s}", "road", 7500, 30.0, (nx, 600))
        g.add_edge(nag, road)
        g.add_edge(road, j_a, share=0.5)
        g.add_edge(road, j_b, share=0.5)
    return g
