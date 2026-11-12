// Per-node density estimator: a 2-state Kalman filter, x = [rho, b].
// Port of sim/sensing.py kf_step (the reference); test_sense_port.py checks that both give
// the same estimate for the same inputs. Portable C++ (no Arduino headers), scalar float
// operations only, so it runs on the ESP32 and in the PC test harness.
//
//   rho  patch density (persons/m^2)
//   b    drift of the counted net flux (persons/m^2 per s): the counters' combined bias
// Every second:
//   predict  rho <- rho + (in - out) / area - b      (only if every boundary of the patch is counted)
//   correct  with a density reading z (edge camera / CCTV count / RF), when one arrives:
//            removes accumulated drift and learns b, so the counters stay corrected between readings
// A patch whose boundaries are not all counted runs a random walk corrected by z; its step is
// physics-based: density cannot change faster than a full-capacity flow (1.22 persons/m/s, Weidmann)
// through the patch boundary fills the area, rw = 1.22 x width / area (sim: rw_flow_frac = 1).
// A patch with neither sensor is blind and reports 0. A failed density sensor can be flagged:
// the node then reports max(estimate, last good estimate, WATCH level) ("unknown is not safe").
#pragma once
#include <math.h>

namespace pksense {

struct SenseParams {
  float countNoise = 0.3f;     // random miscounts, x sqrt(crossings) per second
  float bWalkSd = 2e-5f;       // how fast the counter drift may change
  float rwSd = 0.01f;          // legacy fixed random walk, used only if the patch width is unknown (0)
  float peakFlow = 1.22f;      // persons/(m s), peak specific flow (Weidmann capacity)
  float obsRelSd = 0.10f;      // density reading: relative error (camera 10 %; RF ~25 %)
  float obsAbsSd = 0.05f;      // density reading: absolute error (persons/m^2)
  float watchRho = 2.4f;       // WATCH level (rho / rho_crit = 0.6) reported by a flagged sensor
};

struct DensityEstimator {
  float area = 1.0f;           // patch area (m^2)
  bool counted = false;        // every boundary of the patch has a line counter
  bool observed = false;       // the patch has a density reading
  bool flagged = false;        // density reading failed, firmware knows it
  float frozen = 0.0f;
  float rw = 0.01f;            // random-walk step of this patch (persons/m^2 per s)
  float rho = 0.0f, b = 0.0f;
  float P00 = 1e-4f, P01 = 0.0f, P11 = 1e-8f;    // (5 * bWalkSd)^2

  void init(float area_m2, bool hasCounters, bool hasReading, const SenseParams& p, float width_m = 0.0f) {
    area = area_m2; counted = hasCounters; observed = hasReading;
    rw = width_m > 0.0f ? p.peakFlow * width_m / area_m2 : p.rwSd;
    rho = 0.0f; b = 0.0f; P00 = 1e-4f; P01 = 0.0f;
    P11 = (5.0f * p.bWalkSd) * (5.0f * p.bWalkSd);
    flagged = false; frozen = 0.0f;
  }

  // the density reading stopped (the board runs on); flag = firmware detects it
  void sensorFault(bool flag, const SenseParams& p) {
    observed = false;
    if (flag) { flagged = true; frozen = fmaxf(rho, p.watchRho); }
  }

  // one second: cin/cout = crossings counted this second; due = a reading z arrived
  float step(float cin, float cout, bool due, float z, const SenseParams& p) {
    if (counted) {
      float qRho = (p.countNoise * p.countNoise * (cin + cout) + 1e-6f) / (area * area);
      rho = rho + (cin - cout) / area - b;
      float n00 = P00 - 2.0f * P01 + P11 + qRho;
      float n01 = P01 - P11;
      float n11 = P11 + p.bWalkSd * p.bWalkSd;
      P00 = n00; P01 = n01; P11 = n11;
    } else {
      P00 = P00 + rw * rw;
      P01 = 0.0f;
    }
    if (due && observed) {
      float zr = fmaxf(z, 0.5f);
      float R = (p.obsRelSd * zr) * (p.obsRelSd * zr) + p.obsAbsSd * p.obsAbsSd;
      float S = P00 + R;
      float K0 = P00 / S, K1 = P01 / S;
      float y = z - rho;
      rho = rho + K0 * y;
      if (counted) b = b + K1 * y;
      float n00 = (1.0f - K0) * P00;
      float n01 = (1.0f - K0) * P01;
      float n11 = P11 - K1 * P01;
      P00 = n00; P01 = n01; P11 = n11;
    }
    bool blind = !counted && !observed;
    rho = blind ? 0.0f : fmaxf(rho, 0.0f);
    if (flagged) rho = fmaxf(rho, frozen);       // stored, as in sensing.py
    return rho;
  }
};

}  // namespace pksense
