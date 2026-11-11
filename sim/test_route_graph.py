"""Unit checks for the route graph and route-based QS alerts.

Run:  python test_route_graph.py
"""

import numpy as np

from qs_core import ACT_HOLD, ACT_INFORM, ACT_METER, ACT_NONE, QSParams, RouteParams, RouteQSNetwork
from route_graph import RouteGraph, compartment_graph, synthetic_site


def toy():
    """Bus stand -> Road A -> J1 -(0.7)-> G3 -> C3 ;  J1 -(0.3)-> G4 -> C4 ;
    Parking -> Road B -> J2 -> G5 -> C5 (unrelated branch)."""
    g = RouteGraph()
    ids = {}
    for name, kind, x, y in [("Bus", "source", 0, 900), ("RoadA", "road", 0, 600), ("J1", "junction", 0, 300),
                             ("G3", "gate", -50, 100), ("C3", "compartment", -50, 0),
                             ("G4", "gate", 50, 100), ("C4", "compartment", 50, 0),
                             ("Park", "source", 500, 900), ("RoadB", "road", 500, 600), ("J2", "junction", 500, 300),
                             ("G5", "gate", 500, 100), ("C5", "compartment", 500, 0)]:
        ids[name] = g.add_node(name, kind, 300, 10, (x, y))
    for a, b, s in [("Bus", "RoadA", 1), ("RoadA", "J1", 1), ("J1", "G3", 0.7), ("G3", "C3", 1),
                    ("J1", "G4", 0.3), ("G4", "C4", 1), ("Park", "RoadB", 1), ("RoadB", "J2", 1),
                    ("J2", "G5", 1), ("G5", "C5", 1)]:
        g.add_edge(ids[a], ids[b], share=s)
    return g, ids


def run(g, rho, steps=120, rp=None, link_loss=0.0, seed=0):
    net = RouteQSNetwork(g, QSParams(), rp or RouteParams())
    rng = np.random.default_rng(seed)
    for _ in range(steps):
        up = rng.random((g.n, g.n)) >= link_loss
        state, _ = net.step(rho, up & up.T)
    return net, state


def main():
    g, ids = toy()
    up = g.upstream(ids["C3"])
    assert set(up) == {ids["G3"], ids["J1"], ids["RoadA"], ids["Bus"]}, "upstream of C3 is only its feeders"
    assert abs(up[ids["J1"]][0] - 0.7) < 1e-9 and abs(up[ids["Bus"]][0] - 0.7) < 1e-9

    rho = np.zeros(g.n)
    rho[ids["C3"]] = 4.5                         # C3 over the 4 p/m^2 limit
    net, state = run(g, rho)
    for far in ["RoadB", "J2", "G5", "C5", "Park", "G4", "C4"]:
        assert net.relayed_L[ids[far]] == 0 and net.action[ids[far]] == ACT_NONE, f"{far} must not hear C3"
    lvl_g3, lvl_j1 = net.relayed_L[ids["G3"]], net.relayed_L[ids["J1"]]
    assert abs(lvl_j1 - 0.7 * lvl_g3) < 1e-6, "J1 hears 0.7 of the threat (its share)"
    assert net.action[ids["G3"]] == ACT_HOLD, "the gate feeding C3 holds"

    # horizon: with a 120 s hold horizon only the gate (83 s walk) acts; J1 (~255 s) and beyond are informed
    net_h, _ = run(g, rho, rp=RouteParams(hold_horizon_s=120, use_horizon=True))
    assert net_h.action[ids["G3"]] in (ACT_METER, ACT_HOLD)
    assert net_h.action[ids["J1"]] == ACT_INFORM, "feeder beyond the horizon is only informed"
    assert net_h.action[ids["Bus"]] in (ACT_INFORM, ACT_NONE)

    # without share weighting J1 hears the full threat (ablation)
    net_ns, _ = run(g, rho, rp=RouteParams(use_share=False))
    assert abs(net_ns.relayed_L[ids["J1"]] - net_ns.relayed_L[ids["G3"]]) < 1e-6

    # normal crowd: nobody acts
    net_n, state_n = run(g, np.full(g.n, 0.8))
    assert (net_n.action == ACT_NONE).all() and (state_n == 0).all()

    # the compartment graph uses ghat_scenario's node order
    import ghat_scenario as gs
    assert compartment_graph().names == gs.NODE_NAMES

    site = synthetic_site()
    kinds = site.kind
    assert kinds.count("compartment") == 68 and kinds.count("junction") == 20 and kinds.count("source") == 10
    print("ok")


if __name__ == "__main__":
    main()
