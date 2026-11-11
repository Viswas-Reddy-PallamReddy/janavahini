// Route-graph quorum-sensing alert (PS-01), shared by the ESP32 firmware
// (src/main.cpp) and the software-in-the-loop harness (sil/sil.cpp).
// Plain C++ with no Arduino calls, so the exact code that runs on the board can
// also run on a PC against simulated crowds and a simulated radio.
//
// Mirrors sim/qs_core.py RouteQSNetwork with the default QSParams/RouteParams,
// plus junction diversion from sim/site_flow.py divert_shares:
//   own level   A = DELTA*A + (1-DELTA)*s,  s = rho/RHO_CRIT + rate term
//   relayed     L_rel = sum over successors j of share_j * L_j (fresh only)
//   broadcast   L = max(A, L_rel), with the walk time to that threat's origin
//   actions     relayed threat WATCH -> METER, CRITICAL -> HOLD;
//               a junction with a threatened branch and a free sibling DIVERTs instead
// Differences from the Python model, on purpose:
//   - float32 (the ESP32 FPU) instead of float64
//   - the rate term divides by the measured time span of the history, not
//     RATE_WINDOW * 1 s (the board's loop period is ~1.0-1.06 s, not exactly 1 s)
//   - node-failure handling (both off by default, so the model comparison is exact;
//     tested in sim/esp32_sil.py):
//       fail-safe hold: a successor that falls silent while it was alarming keeps
//         its last level for failsafeMs (FAILSAFE_UNTIL_RESTORED = until it is heard
//         again) and is reported down, instead of the alarm quietly clearing;
//       2-hop backup: a node may also listen to its successors' successors
//         (addBackup); a backup counts only while the successor it bypasses is silent.
#pragma once
#include <math.h>
#include <stdint.h>
#include <string.h>

namespace pkqs {

static const float RHO_CRIT = 4.0f, DELTA = 0.85f;
static const float RATE_HORIZON_S = 60.0f, RATE_GAIN = 1.0f, RATE_FLOOR = 1.5f;
static const float THETA_WATCH = 0.6f, THETA_CRIT = 0.85f, HYST = 0.1f;
static const int MIN_DWELL = 5, RATE_WINDOW = 30;
static const uint32_t STALE_MS = 5000;
static const float DIVERT_THETA = 0.6f, SPARE_RHO = 2.0f;
static const int MAX_SUCC = 12;            // primaries + 2-hop backups
static const uint32_t FAILSAFE_UNTIL_RESTORED = 0xFFFFFFFFu;
// the hold starts as soon as a successor is stale (5 s), but staff are only told a node is
// DOWN after 30 s of silence: at 50 % message loss a 5-s gap happens ~2 % of seconds per
// link by chance, a 30-s gap ~1e-9 (sim/analytic_model.py M2)
static const uint32_t DOWN_REPORT_MS = 30000;
static const float NO_WALK = 65535.0f;
static const float UNKNOWN_RHO = 99.0f;     // a gate whose compartment node went silent: treat as full

enum { NORMAL = 0, WATCH = 1, CRITICAL = 2 };
enum { ACT_NONE = 0, ACT_INFORM = 1, ACT_METER = 2, ACT_HOLD = 3, ACT_DIVERT = 4 };
enum { KIND_OTHER = 0, KIND_GATE = 1, KIND_JUNCTION = 2 };

// One broadcast per node per loop (ESP-NOW; LoRa for long hops). 25 bytes.
struct __attribute__((packed)) RouteMsg {
  uint8_t magic;      // 'Q'
  uint16_t node;
  uint8_t state;      // alert state (local alarm, or the relayed threat where this node must act)
  uint8_t action;
  float A;            // own leaky level
  float L;            // threat this node passes upstream: max(A, relayed)
  uint16_t W;         // walking seconds from this node to that threat's origin
  float rho;          // own density, persons/m^2
  float rhoAhead;     // a gate: density of the compartment behind it; others: own density
  uint16_t seq;
};

struct StateMachine {
  uint8_t state = NORMAL;
  int dwell = 0;
  uint8_t update(float level) {
    uint8_t target = level >= THETA_CRIT ? CRITICAL : (level >= THETA_WATCH ? WATCH : NORMAL);
    bool raise = target > state;
    dwell = raise ? 0 : dwell + 1;
    float release = (state == CRITICAL ? THETA_CRIT : THETA_WATCH) - HYST;
    bool drop = !raise && target < state && dwell >= MIN_DWELL && level < release;
    if (raise) state = target;
    else if (drop) { state--; dwell = 0; }
    return state;
  }
};

struct Succ {
  uint16_t id;
  float share, walk;            // route share of this node's outflow, walking seconds to it
  int8_t via = -1;              // -1: a successor; k: a 2-hop backup bypassing successor k
  float L = 0, W = 0, rho = 0, rhoAhead = 0;
  uint32_t heardMs = 0;
  bool heard = false;
  bool ever = false;            // heard at least once since boot (not reset by clearDown)
};

struct Node {
  uint16_t id = 0;
  uint8_t kind = KIND_OTHER;
  Succ succ[MAX_SUCC];
  int nSucc = 0;
  bool useShare = true, useHorizon = false, meter = true;
  float holdHorizonS = 180.0f;
  uint32_t failsafeMs = 0;      // 0 = off (as in the Python model)
  // "unknown is not safe" (E10): a successor silent while alarming counts as CRITICAL, and one
  // silent for more than DOWN_REPORT_MS counts as at least WATCH even if it was calm (it may have
  // died just before its own alarm); its crowd may still be growing unseen
  bool failsafeEscalate = false;

  float hist[RATE_WINDOW + 1];
  uint32_t histMs[RATE_WINDOW + 1];
  int count = 0;
  float A = 0, rho = 0, relayedL = 0, relayedW = NO_WALK, bestL = 0, bestW = 0;
  StateMachine local, threat;
  uint8_t state = NORMAL, action = ACT_NONE;
  uint16_t divertMask = 0;      // bit k: branch k closed, its pilgrims sent to open siblings
  uint16_t downMask = 0;        // bit k: successor k silent while alarming (fail-safe holding or bypassed)
  uint16_t seq = 0;

  bool addSucc(uint16_t sid, float share, float walk) {
    if (nSucc >= MAX_SUCC) return false;
    Succ s;
    s.id = sid; s.share = share; s.walk = walk;
    succ[nSucc++] = s;
    return true;
  }

  // 2-hop backup behind successor k: share = share(k) * share(k -> sid), walk likewise summed
  bool addBackup(int k, uint16_t sid, float share, float walk) {
    if (!addSucc(sid, share, walk)) return false;
    succ[nSucc - 1].via = (int8_t)k;
    return true;
  }

  // ESP-NOW receive path: keep the latest broadcast of each successor
  void onMsg(const RouteMsg& m, uint32_t nowMs) {
    for (int k = 0; k < nSucc; k++)
      if (succ[k].id == m.node) {
        succ[k].L = m.L; succ[k].W = m.W; succ[k].rho = m.rho; succ[k].rhoAhead = m.rhoAhead;
        succ[k].heardMs = nowMs; succ[k].heard = true; succ[k].ever = true;
      }
  }

  bool fresh(const Succ& s, uint32_t nowMs) const { return s.heard && nowMs - s.heardMs <= STALE_MS; }

  // level a held successor counts as with failsafeEscalate
  static float heldLevel(const Succ& c) {
    return c.L >= THETA_WATCH ? fmaxf(c.L, THETA_CRIT) : fmaxf(c.L, THETA_WATCH);
  }

  // successor k is held or bypassed and has been silent long enough to send staff to check it
  bool reportDown(int k, uint32_t nowMs) const {
    return (downMask & (1 << k)) && nowMs - succ[k].heardMs > DOWN_REPORT_MS;
  }

  // Operator has checked a silent node on site: forget its last level so the fail-safe hold ends.
  // nodeId 0xFFFF clears every silent successor.
  void clearDown(uint16_t nodeId, uint32_t nowMs) {
    for (int k = 0; k < nSucc; k++)
      if ((nodeId == 0xFFFF || succ[k].id == nodeId) && !fresh(succ[k], nowMs)) succ[k].heard = false;
  }

  float production(float r, uint32_t nowMs) {
    int i = count % (RATE_WINDOW + 1);
    hist[i] = r; histMs[i] = nowMs;
    count++;
    float rate = 0;
    if (count > RATE_WINDOW && r >= RATE_FLOOR) {
      int j = (count - 1 - RATE_WINDOW) % (RATE_WINDOW + 1);
      float span = (nowMs - histMs[j]) / 1000.0f;
      if (span > 0) rate = (r - hist[j]) / span;
    }
    return r / RHO_CRIT + RATE_GAIN * fmaxf(0.0f, rate) * RATE_HORIZON_S / RHO_CRIT;
  }

  // One loop tick: density in, broadcast out.
  RouteMsg step(float r, uint32_t nowMs) {
    rho = r;
    float s = production(r, nowMs);

    // relayed threat over the successors (share-weighted sum; strongest branch's walk time)
    float sumL = 0, maxL = -1, pickW = NO_WALK;
    bool live[MAX_SUCC], bypassed[MAX_SUCC] = {false};
    for (int k = 0; k < nSucc; k++) live[k] = fresh(succ[k], nowMs);
    for (int k = 0; k < nSucc; k++)
      if (succ[k].via >= 0 && live[k] && !live[succ[k].via]) bypassed[succ[k].via] = true;
    // fail-safe hold of a silent successor: silent while alarming, or (escalation) long silent
    bool held[MAX_SUCC] = {false};
    float heldL[MAX_SUCC] = {0};
    for (int k = 0; k < nSucc; k++) {
      const Succ& c = succ[k];
      if (c.via >= 0 || live[k] || !failsafeMs || !c.heard) continue;
      if (failsafeMs != FAILSAFE_UNTIL_RESTORED && nowMs - c.heardMs > failsafeMs) continue;
      if (c.L >= THETA_WATCH || (failsafeEscalate && nowMs - c.heardMs > DOWN_REPORT_MS)) {
        held[k] = true;
        heldL[k] = failsafeEscalate ? heldLevel(c) : c.L;
      }
    }
    // a silent successor that is both held and bypassed counts as the larger of the two: the
    // backups see what lies beyond it, but not the crowd inside it
    float bSum[MAX_SUCC] = {0}, bMax[MAX_SUCC] = {0};
    for (int k = 0; k < nSucc; k++)
      if (succ[k].via >= 0 && live[k] && !live[succ[k].via]) {
        float c = (useShare ? succ[k].share : 1.0f) * succ[k].L;
        bSum[succ[k].via] += c;
        bMax[succ[k].via] = fmaxf(bMax[succ[k].via], c);
      }
    bool useHeld[MAX_SUCC] = {false};
    downMask = 0;
    for (int k = 0; k < nSucc; k++) {
      if (succ[k].via >= 0) continue;
      float h = (useShare ? succ[k].share : 1.0f) * heldL[k];
      useHeld[k] = held[k] && (!bypassed[k] || h >= (useShare ? bSum[k] : bMax[k]));
      if (held[k] || (bypassed[k] && succ[k].ever)) downMask |= 1 << k;   // silent, not just not yet heard
    }
    for (int k = 0; k < nSucc; k++) {
      const Succ& c = succ[k];
      bool ok;
      float Lk = c.L;
      if (c.via >= 0) {
        // backup: only while the successor it bypasses is silent and not held at a higher level
        ok = live[k] && !live[c.via] && !useHeld[c.via];
      } else {
        ok = live[k] || useHeld[k];
        if (!live[k] && useHeld[k]) Lk = heldL[k];
      }
      float w = useShare ? c.share : 1.0f;
      float candL = ok ? w * Lk : 0.0f;
      float candW = c.walk + c.W;
      sumL += candL;
      if (candL > maxL || (candL == maxL && candW < pickW)) { maxL = candL; pickW = candW; }
    }
    relayedL = nSucc ? (useShare ? sumL : maxL) : 0.0f;
    relayedW = nSucc ? pickW : NO_WALK;

    A = DELTA * A + (1 - DELTA) * s;
    bool own = A >= relayedL;
    bestL = own ? A : relayedL;
    bestW = own ? 0.0f : relayedW;

    uint8_t loc = local.update(A);
    uint8_t thr = threat.update(relayedL);
    bool near = useHorizon ? relayedW <= holdHorizonS : true;
    bool act = thr >= WATCH && near;
    action = thr >= WATCH ? ACT_INFORM : ACT_NONE;
    if (meter) {
      if (act && thr == WATCH) action = ACT_METER;
      if (act && thr == CRITICAL) action = ACT_HOLD;
    } else if (act) {
      action = ACT_HOLD;
    }
    state = loc > (act ? thr : NORMAL) ? loc : (act ? thr : NORMAL);

    // junction: close threatened branches if a sibling branch is calm and has room
    divertMask = 0;
    int nPrimary = 0;
    for (int k = 0; k < nSucc; k++) nPrimary += succ[k].via < 0;
    if (kind == KIND_JUNCTION && nPrimary >= 2) {
      // with node-failure handling on, a silent branch is never "calm with room" (its last
      // message is stale), and a held or bypassed branch keeps the alarm level it carries
      bool failureHandling = failsafeMs > 0 || nSucc > nPrimary;
      uint16_t bad = 0, okm = 0;
      for (int k = 0; k < nSucc; k++) {
        if (succ[k].via >= 0) continue;
        float lvl = live[k] ? succ[k].L : 0.0f;
        if (failureHandling && !live[k]) {
          if (bypassed[k])
            for (int j = 0; j < nSucc; j++)
              if (succ[j].via == k && live[j]) lvl = fmaxf(lvl, succ[j].L);
          if (held[k]) lvl = fmaxf(lvl, heldL[k]);
        }
        if (lvl >= DIVERT_THETA) bad |= 1 << k;
        if (lvl < 0.5f * DIVERT_THETA && succ[k].rhoAhead < SPARE_RHO && (live[k] || !failureHandling))
          okm |= 1 << k;
      }
      if (bad && okm) { divertMask = bad; action = ACT_DIVERT; }
    }

    float ahead = rho;
    if (kind == KIND_GATE) {
      ahead = 0;
      bool anyLive = false, anyEver = false;
      for (int k = 0; k < nSucc; k++) {
        if (succ[k].via >= 0) continue;
        anyLive |= live[k];
        anyEver |= succ[k].ever;
        if (live[k] && succ[k].rho > ahead) ahead = succ[k].rho;
      }
      bool failureHandling = failsafeMs > 0 || nSucc > nPrimary;
      if (failureHandling && !anyLive && anyEver) ahead = UNKNOWN_RHO;   // not "empty": unknown
    }
    RouteMsg m;
    m.magic = 'Q'; m.node = id; m.state = state; m.action = action;
    m.A = A; m.L = bestL; m.W = (uint16_t)fminf(bestW, NO_WALK);
    m.rho = rho; m.rhoAhead = ahead; m.seq = seq++;
    return m;
  }
};

}  // namespace pkqs
