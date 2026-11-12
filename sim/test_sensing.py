"""Unit tests for sensing.py on a single compartment with a known in/out flow.

1. Dead reckoning with a biased counter drifts linearly, as predicted:
   drift after T s = (beta_in * q_in - beta_out * q_out) * T / A.
2. With a density reading every 60 s, the Kalman filter keeps the error bounded and
   learns the counter drift b.
3. Truth mode returns the true density.
"""

from types import SimpleNamespace

import numpy as np

from sensing import Sensing, SensingConfig


def one_node_graph(area=600.0, width=5.0):
    # node 0 = the compartment; edges: none (flows come in as arrivals and leave as exits)
    g = SimpleNamespace(n=1, area=[area], width=[width], kind=["compartment"])
    g.edge_arrays = lambda: (np.zeros(0, int), np.zeros(0, int), np.zeros(0), np.zeros(0))
    return g


def simulate(cfg, T=3600, q_in=6.0, q_out=5.5, beta_in=0.05, beta_out=0.0, seed=1):
    g = one_node_graph()
    s = Sensing(g, cfg, seed)
    # treat the node as both a counted entry and exit so the flux model is on
    s.c_src[:] = True
    s.c_exit[:] = True
    s.counted[:] = cfg.counters != "none"
    s.observed[:] = cfg.density != "none"
    s.blind[:] = ~s.counted & ~s.observed
    s.beta_src[:] = beta_in
    s.beta_exit[:] = beta_out
    s.gamma[:] = 0.0
    N, err = 0.0, []
    for t in range(T):
        N += q_in - q_out
        est = s.update(t, np.zeros(0), np.array([q_in]), np.array([q_out]), np.array([N / 600.0]))
        err.append(est[0] - N / 600.0)
    return np.array(err), s


def test_dead_reckoning_drifts_as_predicted():
    cfg = SensingConfig(counters="all", density="none", count_noise=0.0)
    err, _ = simulate(cfg)
    predicted = 0.05 * 6.0 * 3600 / 600.0                 # 1.8 persons/m^2 after one hour
    assert abs(err[-1] - predicted) < 1e-6, (err[-1], predicted)


def test_kalman_bounds_drift_and_learns_bias():
    """Over 5 seeds: the filter beats both dead reckoning (1.8 p/m^2 after 1 h) and the
    camera alone, stays within 0.35 p/m^2 after warm-up, and learns the counter drift on
    average (a single run's estimate is noisy with a 10 % reading every 60 s)."""
    cfg = SensingConfig(counters="all", density="all", obs_every_s=60, obs_bias_sd=0.0)
    cam = SensingConfig(counters="none", density="all", obs_every_s=60, obs_bias_sd=0.0)
    true_b = 0.05 * 6.0 / 600.0                           # drift of the counted flux, p/m^2/s
    bs, rms_kf, rms_cam = [], [], []
    for seed in range(5):
        err, s = simulate(cfg, seed=seed)
        assert np.abs(err[600:]).max() < 0.35, np.abs(err[600:]).max()
        rms_kf.append(np.sqrt(np.mean(err[600:] ** 2)))
        rms_cam.append(np.sqrt(np.mean(simulate(cam, seed=seed)[0][600:] ** 2)))
        bs.append(s.b[0])
    assert np.mean(rms_kf) < np.mean(rms_cam), (np.mean(rms_kf), np.mean(rms_cam))
    assert abs(np.mean(bs) - true_b) < 0.4 * true_b, (np.mean(bs), true_b)


def test_truth_mode():
    g = one_node_graph()
    s = Sensing(g, SensingConfig(truth=True), 0)
    assert s.update(0, np.zeros(0), np.array([1.0]), np.array([0.0]), np.array([2.5]))[0] == 2.5


if __name__ == "__main__":
    test_dead_reckoning_drifts_as_predicted()
    test_kalman_bounds_drift_and_learns_bias()
    test_truth_mode()
    print("sensing tests passed")
