// Pushkaralu prototype firmware.
//
// ROLE_NODE: fixed crowd node (shared by PS-01 and PS-03).
//   PS-01: runs the route-graph quorum-sensing (QS) alert (include/pk_qs.h,
//          the same code the SIL harness runs; sim/qs_core.py RouteQSNetwork is
//          the Python model) and broadcasts its threat over ESP-NOW to the
//          nodes that feed it.
//   PS-03: keeps a rolling 60-min buffer of child-tag sightings (opaque
//          rotating IDs only) and answers "search" queries from the desk;
//          watches for the family devices a flagged tag has learned; relays
//          desk commands (LOST on/off) to tags as a BLE command beacon.
//   IS_GATEWAY : also bridges everything to USB serial (JSON lines) and takes
//                desk commands from the backend (FIND / LOSTCMD / CLEARCMD / RHO).
//   IS_GUARDIAN: demo only - advertises a guardian beacon (stand-in for a
//                family phone) so the tag has a companion to learn.
// ROLE_TAG: band tag for children / senior citizens. Rotating HMAC ID +
//   rotating random MAC. Learns its "co-walkers" (family devices that stay
//   close) and flags possible separation when they all disappear. Goes LOST
//   on a desk command or the BOOT button.
//
// In deployment the ESP-NOW messages between nodes travel over LoRa for the
// longer hops; the message formats are the same.

#include <Arduino.h>
#include <BLEDevice.h>
#include <BLEAdvertisedDevice.h>
#include <BLEScan.h>
#include "pk_common.h"
#include "pk_qs.h"
#include "pk_sense.h"

#ifdef ROLE_NODE
#include <WiFi.h>
#include <esp_now.h>
#include <esp_wifi.h>
#endif

static String mfgPayload(const char kind[2], uint8_t flags, const uint8_t id[8],
                         const uint8_t* extra = nullptr, int extraLen = 0) {
  String s;
  s += (char)(COMPANY_ID & 0xFF);
  s += (char)(COMPANY_ID >> 8);
  s += kind[0];
  s += kind[1];
  s += (char)1;  // version
  s += (char)flags;
  for (int i = 0; i < 8; i++) s += (char)id[i];
  for (int i = 0; i < extraLen; i++) s += (char)extra[i];
  return s;
}

// Parses our manufacturer payload: kind is 'K' (tag), 'G' (guardian) or 'C' (command).
static bool parsePayload(const std::string& m, char& kind, uint8_t& flags, uint8_t id[8],
                         uint8_t extra[8], int& extraLen) {
  if (m.size() < 14) return false;
  if ((uint8_t)m[0] != (COMPANY_ID & 0xFF) || (uint8_t)m[1] != (COMPANY_ID >> 8)) return false;
  if (m[2] != 'P' || (m[3] != 'K' && m[3] != 'G' && m[3] != 'C')) return false;
  kind = m[3];
  flags = (uint8_t)m[5];
  memcpy(id, m.data() + 6, 8);
  extraLen = min((int)m.size() - 14, 8);
  if (extraLen > 0) memcpy(extra, m.data() + 14, extraLen);
  return true;
}

static void printHex(const uint8_t* b, int n) {
  for (int i = 0; i < n; i++) Serial.printf("%02x", b[i]);
}

static bool parseHex(const String& s, uint8_t* out, int n) {
  if ((int)s.length() < 2 * n) return false;
  for (int i = 0; i < n; i++) out[i] = (uint8_t)strtol(s.substring(2 * i, 2 * i + 2).c_str(), nullptr, 16);
  return true;
}

// =====================================================================
#ifdef ROLE_NODE

static const uint8_t BCAST[6] = {0xFF, 0xFF, 0xFF, 0xFF, 0xFF, 0xFF};
static portMUX_TYPE mux = portMUX_INITIALIZER_UNLOCKED;

// Route-graph QS node. Its successors (the places it sends pilgrims to) are set at
// installation from the route map, together with 2-hop backups (each successor's own
// successors), so one dead board never blinds the nodes behind it. Demo with two
// boards: the road node R2 (NODE_ID 2) feeds the holding area C1 (NODE_ID 4, the
// gateway); the gate between them is folded in, so there is no 2-hop backup.
// Site deployment, e.g. a road node before gate 7 / compartment 8:
//   qs.addSucc(7, 1.0f, 40.0f);  qs.addBackup(0, 8, 1.0f, 55.0f);
static pkqs::Node qs;
static void configureRoute() {
  qs.id = NODE_ID;
#if NODE_ID == 2
  qs.addSucc(4, 1.0f, 20.0f);
#endif
  // "unknown is not safe": a successor that falls silent while alarming counts as CRITICAL, one
  // silent for over 30 s as at least WATCH, until it is heard again or an operator sends
  // CLEARDOWN; it is reported as "down" after 30 s (sim/esp32_sil.py E9, failure_experiment.py E10)
  qs.failsafeMs = pkqs::FAILSAFE_UNTIL_RESTORED;
  qs.failsafeEscalate = true;
}

// every node heard on the radio, for the dashboard (not only successors)
struct Heard { uint16_t id; float A; uint8_t state; uint8_t action; float rho; uint32_t heard_ms; };
static Heard nbrs[8];
static int nNbrs = 0;
static float rho = 0;
static uint8_t state = 0;
static uint32_t lastFeedMs = 0;
static int bleDevicesSeen = 0;

// ---- PS-01 density estimator (include/pk_sense.h; checked against sim/sensing.py by
// sim/test_sense_port.py) ----
// Raw sensor input: line-counter crossings and density readings. In the demo they arrive over
// serial ("SENSE ...") or relayed by the gateway (CountMsg 'N'); real drivers (VL53L1X pair at a
// gate, camera density over UART on a road) add to the same accumulators. Once raw input has been
// seen, the node's density is the estimator's output instead of a ready-made RHO value.
#ifndef PATCH_AREA_M2
#define PATCH_AREA_M2 300.0f      // patch area (m^2), set per node at installation
#endif
#ifndef PATCH_WIDTH_M
#define PATCH_WIDTH_M 8.0f        // boundary width (m): sets how fast an uncounted patch may fill
#endif
#ifndef HAS_COUNTERS
#define HAS_COUNTERS 1            // every boundary of the patch has a line counter (gates, exits)
#endif
#ifndef HAS_READING
#define HAS_READING 1             // the patch has a density reading (camera / CCTV / RF)
#endif
static pksense::SenseParams senseP;
static pksense::DensityEstimator est;
static bool useEst = false;
static float accIn = 0, accOut = 0, zVal = 0;
static bool zDue = false;
static void senseInput(float cin, float cout, float z) {   // caller holds mux
  accIn += cin; accOut += cout;
  if (z >= 0) { zVal = z; zDue = true; }
  useEst = true;
  lastFeedMs = millis();
}

// ---- PS-03 buffers ----
static const uint32_t SIGHT_KEEP_S = 3600;     // tag sightings kept 60 min, then expire
static const uint32_t DEV_KEEP_S = 600;        // device-address hashes kept 10 min
struct Sight { uint8_t id[8]; uint32_t t; int8_t rssi; bool used; };
static Sight sights[600];
static int sightIdx = 0;
struct Dev { uint8_t h[4]; uint32_t t; int8_t rssi; bool used; };
static Dev devs[256];
static int devIdx = 0;
static uint8_t watch[8][4];
static int nWatch = 0;
static uint32_t watchUntilMs = 0;

// outgoing/print queues (filled from callbacks, drained in loop)
static SightingMsg qSight[8];  static int nQSight = 0;
static HitMsg qHit[16];        static int nQHit = 0;
static FamilyMsg qFam[8];      static int nQFam = 0;
static SearchMsg pendingSearch; static bool hasSearch = false;
static WatchMsg pendingWatch;   static bool hasWatchOut = false;
static CmdMsg activeCmd;        static uint32_t cmdUntilMs = 0; static bool cmdChanged = false;

static uint32_t nowS() { return millis() / 1000; }

static void recordSight(const uint8_t id[8], int8_t rssi) {
  uint32_t t = nowS();
  for (int i = 0; i < 600; i++)                     // refresh if heard in the last 10 s
    if (sights[i].used && t - sights[i].t < 10 && memcmp(sights[i].id, id, 8) == 0) {
      sights[i].t = t; sights[i].rssi = max(sights[i].rssi, rssi); return;
    }
  Sight& s = sights[sightIdx];
  memcpy(s.id, id, 8); s.t = t; s.rssi = rssi; s.used = true;
  sightIdx = (sightIdx + 1) % 600;
}

static void recordDev(const uint8_t h[4], int8_t rssi) {
  uint32_t t = nowS();
  for (int i = 0; i < 256; i++)
    if (devs[i].used && memcmp(devs[i].h, h, 4) == 0) { devs[i].t = t; devs[i].rssi = rssi; return; }
  Dev& d = devs[devIdx];
  memcpy(d.h, h, 4); d.t = t; d.rssi = rssi; d.used = true;
  devIdx = (devIdx + 1) % 256;
}

static void queueFamily(const uint8_t h[4], uint16_t age, int8_t rssi) {
  if (nQFam >= 8) return;
  FamilyMsg m = {'M', (uint8_t)NODE_ID, {0}, age, rssi};
  memcpy(m.hash, h, 4);
  qFam[nQFam++] = m;
}

static void checkWatchAgainstBuffer() {
  uint32_t t = nowS();
  for (int w = 0; w < nWatch; w++)
    for (int i = 0; i < 256; i++)
      if (devs[i].used && t - devs[i].t <= DEV_KEEP_S && memcmp(devs[i].h, watch[w], 4) == 0)
        queueFamily(watch[w], (uint16_t)(t - devs[i].t), devs[i].rssi);
}

static void addWatch(const uint8_t* hashes, int n) {
  for (int k = 0; k < n; k++) {
    bool dup = false;
    for (int w = 0; w < nWatch; w++) if (memcmp(watch[w], hashes + 4 * k, 4) == 0) dup = true;
    if (!dup && nWatch < 8) memcpy(watch[nWatch++], hashes + 4 * k, 4);
  }
  watchUntilMs = millis() + 15 * 60 * 1000;      // watch for 15 min (one address-rotation period)
}

#ifdef IS_GATEWAY
struct PeerGuardian { bool valid; uint8_t node; uint8_t state; float rssi; uint32_t ms; } peerG = {false};
#endif

static Heard* findNbr(uint16_t id) {
  for (int i = 0; i < nNbrs; i++) if (nbrs[i].id == id) return &nbrs[i];
  if (nNbrs < 8) { nbrs[nNbrs] = {id, 0, 0, 0, 0, 0}; return &nbrs[nNbrs++]; }
  return nullptr;
}

static void onRecv(const uint8_t* mac, const uint8_t* data, int len) {
  if (len < 1) return;
  portENTER_CRITICAL(&mux);
  switch (data[0]) {
    case 'Q':
      if (len == sizeof(pkqs::RouteMsg)) {
        pkqs::RouteMsg m; memcpy(&m, data, sizeof(m));
        if (m.node != NODE_ID) {
          qs.onMsg(m, millis());
          Heard* n = findNbr(m.node);
          if (n) { n->A = m.A; n->state = m.state; n->action = m.action; n->rho = m.rho; n->heard_ms = millis(); }
        }
      }
      break;
    case 'R':
      if (len == sizeof(RhoMsg)) {
        RhoMsg m; memcpy(&m, data, sizeof(m));
        if (m.node == NODE_ID) { rho = m.rho; lastFeedMs = millis(); }
      }
      break;
    case 'N':
      if (len == sizeof(CountMsg)) {
        CountMsg m; memcpy(&m, data, sizeof(m));
        if (m.node == NODE_ID) senseInput(m.cin, m.cout, m.z);
      }
      break;
    case 'S':
      if (len == sizeof(SightingMsg) && nQSight < 8) memcpy(&qSight[nQSight++], data, sizeof(SightingMsg));
      break;
    case 'F':
      if (len == sizeof(SearchMsg) && !hasSearch) { memcpy(&pendingSearch, data, sizeof(SearchMsg)); hasSearch = true; }
      break;
    case 'H':
      if (len == sizeof(HitMsg) && nQHit < 16) memcpy(&qHit[nQHit++], data, sizeof(HitMsg));
      break;
    case 'W':
      if (len == sizeof(WatchMsg)) { WatchMsg w; memcpy(&w, data, sizeof(w)); addWatch(&w.hashes[0][0], min((int)w.n, 2)); }
      break;
    case 'M':
      if (len == sizeof(FamilyMsg) && nQFam < 8) memcpy(&qFam[nQFam++], data, sizeof(FamilyMsg));
      break;
    case 'C':
      if (len == sizeof(CmdMsg)) { memcpy(&activeCmd, data, sizeof(CmdMsg)); cmdUntilMs = millis() + 60000; cmdChanged = true; }
      break;
    case 'D':
      if (len == sizeof(ClearDownMsg)) { ClearDownMsg d; memcpy(&d, data, sizeof(d)); qs.clearDown(d.node, millis()); }
      break;
#ifdef IS_GATEWAY
    case 'G':
      if (len == 7) { peerG.valid = true; peerG.node = data[1]; peerG.state = data[2]; memcpy(&peerG.rssi, data + 3, 4); peerG.ms = millis(); }
      break;
#endif
  }
  portEXIT_CRITICAL(&mux);
}

class ScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    bleDevicesSeen++;
    uint8_t h[4];
    addr_hash4(*d.getAddress().getNative(), h);
    int8_t rssi = (int8_t)d.getRSSI();
    portENTER_CRITICAL(&mux);
    recordDev(h, rssi);
    if (millis() < watchUntilMs)
      for (int w = 0; w < nWatch; w++) if (memcmp(watch[w], h, 4) == 0) queueFamily(h, 0, rssi);
    portEXIT_CRITICAL(&mux);

    if (!d.haveManufacturerData()) return;
    char kind; uint8_t flags; uint8_t id[8]; uint8_t extra[8]; int extraLen;
    if (!parsePayload(d.getManufacturerData(), kind, flags, id, extra, extraLen) || kind != 'K') return;
    portENTER_CRITICAL(&mux);
    recordSight(id, rssi);
    if (flags & (FLAG_LOST | FLAG_SEPARATED)) {
      if (nQSight < 8) { SightingMsg s = {'S', (uint8_t)NODE_ID, {0}, rssi, flags}; memcpy(s.id, id, 8); qSight[nQSight++] = s; }
      if (extraLen >= 4) {                       // tag carries hashes of its learned family devices
        int n = extraLen / 4;
        addWatch(extra, n);
        pendingWatch = {'W', (uint8_t)NODE_ID, (uint8_t)n, {{0}}};
        memcpy(pendingWatch.hashes, extra, 4 * n);
        hasWatchOut = true;
      }
    }
    portEXIT_CRITICAL(&mux);
  }
};

static BLEScan* scan;

static void broadcastSearch(const uint8_t ids[][8], int n) {
  for (int start = 0; start < n; start += 8) {
    SearchMsg m = {'F', (uint8_t)min(8, n - start), {{0}}};
    memcpy(m.ids, ids[start], 8 * m.n);
    esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
    portENTER_CRITICAL(&mux);
    if (!hasSearch) { pendingSearch = m; hasSearch = true; }   // also search our own buffer
    portEXIT_CRITICAL(&mux);
    delay(5);
  }
}

static void answerSearch() {
  SearchMsg m;
  portENTER_CRITICAL(&mux);
  if (!hasSearch) { portEXIT_CRITICAL(&mux); return; }
  m = pendingSearch; hasSearch = false;
  uint32_t t = nowS();
  HitMsg hits[16]; int nh = 0;
  for (int k = 0; k < m.n; k++)
    for (int i = 0; i < 600 && nh < 16; i++)
      if (sights[i].used && t - sights[i].t <= SIGHT_KEEP_S && memcmp(sights[i].id, m.ids[k], 8) == 0) {
        hits[nh] = {'H', (uint8_t)NODE_ID, {0}, (uint16_t)(t - sights[i].t), sights[i].rssi};
        memcpy(hits[nh].id, sights[i].id, 8);
        nh++;
      }
  for (int i = 0; i < nh && nQHit < 16; i++) qHit[nQHit++] = hits[i];
  portEXIT_CRITICAL(&mux);
  for (int i = 0; i < nh; i++) esp_now_send(BCAST, (uint8_t*)&hits[i], sizeof(HitMsg));
}

static void readSerial() {
  // RHO <v> | RHO <node> <v>        simulated sensor feed (hardware-in-the-loop)
  // SENSE <in> <out> <z> | SENSE <node> <in> <out> <z>   raw counts + density reading (z < 0: none)
  //                                  for the on-node estimator (pk_sense.h)
  // FIND <hex16> [<hex16> ...]       last-seen search for a case's tag IDs
  // LOSTCMD <hex16> / CLEARCMD <hex16>   desk command to a tag
  // CLEARDOWN <node> | CLEARDOWN ALL operator checked a silent node: end the fail-safe hold
  while (Serial.available()) {
    String line = Serial.readStringUntil('\n');
    line.trim();
    if (line.startsWith("CLEARDOWN ")) {
      String arg = line.substring(10);
      ClearDownMsg d = {'D', arg == "ALL" ? (uint16_t)0xFFFF : (uint16_t)arg.toInt()};
      esp_now_send(BCAST, (uint8_t*)&d, sizeof(d));
      portENTER_CRITICAL(&mux);
      qs.clearDown(d.node, millis());
      portEXIT_CRITICAL(&mux);
    } else if (line.startsWith("RHO ")) {
      String rest = line.substring(4);
      int sp = rest.indexOf(' ');
      if (sp < 0) { rho = rest.toFloat(); lastFeedMs = millis(); }
      else {
        int node = rest.substring(0, sp).toInt();
        float v = rest.substring(sp + 1).toFloat();
        if (node == NODE_ID) { rho = v; lastFeedMs = millis(); }
        else { RhoMsg m = {'R', (uint8_t)node, v}; esp_now_send(BCAST, (uint8_t*)&m, sizeof(m)); }
      }
    } else if (line.startsWith("SENSE ")) {
      float v[4];
      int k = 0;
      String rest = line.substring(6) + " ";
      while (k < 4 && rest.length() > 0) {
        int sp = rest.indexOf(' ');
        if (sp > 0) v[k++] = rest.substring(0, sp).toFloat();
        rest = rest.substring(sp + 1);
      }
      if (k == 3 || (k == 4 && (int)v[0] == NODE_ID)) {
        float* x = k == 4 ? v + 1 : v;
        portENTER_CRITICAL(&mux);
        senseInput(x[0], x[1], x[2]);
        portEXIT_CRITICAL(&mux);
      } else if (k == 4) {
        CountMsg m = {'N', (uint8_t)v[0], v[1], v[2], v[3]};
        esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
      }
    } else if (line.startsWith("FIND ")) {
      static uint8_t ids[64][8];
      int n = 0;
      String rest = line.substring(5) + " ";
      while (rest.length() >= 16 && n < 64) {
        if (parseHex(rest, ids[n], 8)) n++;
        int sp = rest.indexOf(' ');
        rest = rest.substring(sp + 1);
      }
      broadcastSearch(ids, n);
    } else if (line.startsWith("LOSTCMD ") || line.startsWith("CLEARCMD ")) {
      bool lost = line.startsWith("LOSTCMD ");
      CmdMsg c = {'C', {0}, lost ? FLAG_LOST : (uint8_t)0};
      if (parseHex(line.substring(lost ? 8 : 9), c.id, 8)) {
        esp_now_send(BCAST, (uint8_t*)&c, sizeof(c));
        portENTER_CRITICAL(&mux);
        activeCmd = c; cmdUntilMs = millis() + 60000; cmdChanged = true;
        portEXIT_CRITICAL(&mux);
      }
    }
  }
}

static void qsStep() {
  portENTER_CRITICAL(&mux);              // onRecv writes the successor table and the sensor inputs
  if (useEst) {                          // one estimator step per loop tick (~1 s)
    rho = est.step(accIn, accOut, zDue, zVal, senseP);
    accIn = accOut = 0;
    zDue = false;
  }
  pkqs::RouteMsg m = qs.step(rho, millis());
  portEXIT_CRITICAL(&mux);
  state = qs.state;
  esp_now_send(BCAST, (uint8_t*)&m, sizeof(m));
}

// Node advertising: a desk command beacon while one is active; otherwise the
// demo guardian beacon (IS_GUARDIAN) or nothing.
static void updateAdvertising() {
  static uint32_t lastSlot = 0xFFFFFFFF;
  static bool cmdShown = false;
  bool cmdActive = millis() < cmdUntilMs;
  uint32_t slot = local_slot();
  if (!cmdChanged && cmdActive == cmdShown && slot == lastSlot) return;
  cmdChanged = false; cmdShown = cmdActive; lastSlot = slot;
  BLEAdvertising* adv = BLEDevice::getAdvertising();
  adv->stop();
  BLEAdvertisementData data;
  data.setFlags(0x06);
  if (cmdActive) {
    data.setManufacturerData(mfgPayload("PC", activeCmd.cmd, activeCmd.id).c_str());
  } else {
#ifdef IS_GUARDIAN
    uint8_t id[8];
    hmac8(DOMAIN_GUARDIAN_ID, slot, id);
    data.setManufacturerData(mfgPayload("PG", 0, id).c_str());
#else
    return;
#endif
  }
  adv->setAdvertisementData(data);
  adv->setMinInterval(cmdActive ? 160 : 800);
  adv->setMaxInterval(cmdActive ? 160 : 800);
  adv->start();
}

static void led() {
  // NORMAL off, WATCH slow blink, CRITICAL solid
  digitalWrite(LED_PIN, state == 2 || (state == 1 && (millis() / 500) % 2));
}

static void report() {
  uint32_t now = millis();
  Serial.printf("{\"type\":\"node\",\"node\":%d,\"rho\":%.3f,\"A\":%.3f,\"state\":%d,\"action\":%d,\"L\":%.3f,"
                "\"divert\":%d,\"src\":\"%s\",\"ble_devices\":%d,\"down\":[",
                NODE_ID, rho, qs.A, state, qs.action, qs.bestL, qs.divertMask,
                (now - lastFeedMs < 10000) ? "feed" : "ble", bleDevicesSeen);
  for (int k = 0, first = 1; k < qs.nSucc; k++)          // successors silent > 30 s, held by the fail-safe
    if (qs.reportDown(k, now)) { Serial.printf("%s%d", first ? "" : ",", qs.succ[k].id); first = 0; }
  Serial.print("],\"nbrs\":[");
  SightingMsg s[8]; HitMsg h[16]; FamilyMsg f[8];
  int ns, nh, nf;
  portENTER_CRITICAL(&mux);
  for (int i = 0; i < nNbrs; i++)
    Serial.printf("%s{\"n\":%d,\"A\":%.3f,\"s\":%d,\"act\":%d,\"rho\":%.3f,\"age_ms\":%lu}", i ? "," : "", nbrs[i].id,
                  nbrs[i].A, nbrs[i].state, nbrs[i].action, nbrs[i].rho, (unsigned long)(now - nbrs[i].heard_ms));
  ns = nQSight; memcpy(s, qSight, sizeof(SightingMsg) * ns); nQSight = 0;
  nh = nQHit;   memcpy(h, qHit, sizeof(HitMsg) * nh);        nQHit = 0;
  nf = nQFam;   memcpy(f, qFam, sizeof(FamilyMsg) * nf);     nQFam = 0;
  bool sendWatch = hasWatchOut; WatchMsg w = pendingWatch; hasWatchOut = false;
  portEXIT_CRITICAL(&mux);
  Serial.println("]}");
  if (sendWatch) { esp_now_send(BCAST, (uint8_t*)&w, sizeof(w)); checkWatchAgainstBuffer(); }
  for (int i = 0; i < ns; i++) {
    if (s[i].node == NODE_ID) esp_now_send(BCAST, (uint8_t*)&s[i], sizeof(SightingMsg));
    Serial.printf("{\"type\":\"sighting\",\"node\":%d,\"rssi\":%d,\"flags\":%d,\"id\":\"", s[i].node, s[i].rssi, s[i].flags);
    printHex(s[i].id, 8); Serial.println("\"}");
  }
  for (int i = 0; i < nh; i++) {
    Serial.printf("{\"type\":\"hit\",\"node\":%d,\"age_s\":%u,\"rssi\":%d,\"id\":\"", h[i].node, h[i].age_s, h[i].rssi);
    printHex(h[i].id, 8); Serial.println("\"}");
  }
  for (int i = 0; i < nf; i++) {
    if (f[i].node == NODE_ID) esp_now_send(BCAST, (uint8_t*)&f[i], sizeof(FamilyMsg));
    Serial.printf("{\"type\":\"family\",\"node\":%d,\"age_s\":%u,\"rssi\":%d,\"hash\":\"", f[i].node, f[i].age_s, f[i].rssi);
    printHex(f[i].hash, 4); Serial.println("\"}");
  }
#ifdef IS_GATEWAY
  if (peerG.valid && now - peerG.ms < 5000)
    Serial.printf("{\"type\":\"guardian\",\"node\":%d,\"state\":%d,\"rssi_f\":%.1f}\n", peerG.node, peerG.state, peerG.rssi);
#endif
}

void setup() {
  Serial.setRxBufferSize(4096);     // a FIND command carries up to 64 tag IDs
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  WiFi.mode(WIFI_STA);
  esp_wifi_set_channel(1, WIFI_SECOND_CHAN_NONE);
  esp_now_init();
  esp_now_register_recv_cb(onRecv);
  esp_now_peer_info_t peer = {};
  memcpy(peer.peer_addr, BCAST, 6);
  peer.channel = 1;
  esp_now_add_peer(&peer);
  BLEDevice::init("");
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new ScanCB(), true);
  scan->setActiveScan(false);
  scan->setInterval(100);
  scan->setWindow(90);
  configureRoute();
  est.init(PATCH_AREA_M2, HAS_COUNTERS != 0, HAS_READING != 0, senseP, PATCH_WIDTH_M);
  Serial.printf("{\"type\":\"boot\",\"node\":%d}\n", NODE_ID);
}

void loop() {
  uint32_t t0 = millis();
  readSerial();
  bleDevicesSeen = 0;
  scan->start(1, false);            // ~1 s scan doubles as the loop tick
  scan->clearResults();
  if (millis() - lastFeedMs > 10000) rho = bleDevicesSeen * 3.0f / 25.0f;   // crude proxy without a feed
  qsStep();
  answerSearch();
  updateAdvertising();
  report();
  while (millis() - t0 < 1000) { led(); delay(20); }
}
#endif  // ROLE_NODE

// =====================================================================
#ifdef ROLE_TAG

// Co-walker learning. Demo timings are short so it can be shown live;
// deployment values (from sim/companion_sim.py) in brackets.
static const float NEAR_DBM = -80.0f;
static const float EMA_A = 1.0f - 5.0f / 120.0f;    // ~2 min window   [10 min]
static const float COMP_TH = 0.4f, STRONG_TH = 0.6f;
static const uint32_t ARM_S = 60;                   // companion must stay [600 s]
static const uint32_t SEP_AFTER_S = 30;             // all companions silent [120 s]
static const uint32_t IMPRINT_S = 60;               // after power-on at the kiosk
static const int IMPRINT_DBM = -65;                 // "within ~1-2 m"

struct Comp { uint8_t addr[6]; uint8_t cls; float score; uint32_t since; int8_t lastRssi;
              bool heardNow, heardLast, used; };
static Comp comps[32];
struct Heard { uint8_t addr[6]; uint8_t cls; int8_t rssi; };
static Heard heard[64];
static int nHeard = 0;

static BLEAdvertising* adv;
static BLEScan* scan;
static uint8_t curId[8];
static bool cmdLost = false, manualLost = false, separated = false;
static uint32_t lastCompanionHeardS = 0, armedSinceS = 0;
static bool armed = false;
static uint8_t remembered[2][6];
static int nRemembered = 0;

static uint32_t nowS() { return millis() / 1000; }

class TagScanCB : public BLEAdvertisedDeviceCallbacks {
  void onResult(BLEAdvertisedDevice d) override {
    if (d.haveManufacturerData()) {
      char kind; uint8_t flags; uint8_t id[8]; uint8_t extra[8]; int extraLen;
      if (parsePayload(d.getManufacturerData(), kind, flags, id, extra, extraLen) && kind == 'C' &&
          memcmp(id, curId, 8) == 0) {
        cmdLost = flags & FLAG_LOST;            // desk command addressed to this tag
        return;
      }
      if (kind == 'K' || kind == 'C') return;   // ignore other tags and command beacons
    }
    if (nHeard >= 64) return;
    Heard& h = heard[nHeard++];
    memcpy(h.addr, *d.getAddress().getNative(), 6);
    h.rssi = (int8_t)d.getRSSI();
    // payload "shape" survives address rotation (company ID, services, TX power, length)
    uint8_t cls = d.getPayloadLength() & 0x1F;
    if (d.haveManufacturerData()) { std::string m = d.getManufacturerData(); if (m.size() >= 2) cls ^= (uint8_t)m[0] ^ ((uint8_t)m[1] << 3); }
    if (d.haveServiceUUID()) cls ^= 0x40;
    if (d.haveTXPower()) cls ^= 0x80;
    h.cls = cls;
  }
};

static Comp* findComp(const uint8_t a[6]) {
  for (int i = 0; i < 32; i++) if (comps[i].used && memcmp(comps[i].addr, a, 6) == 0) return &comps[i];
  return nullptr;
}

static Comp* newComp() {
  int worst = 0;
  for (int i = 0; i < 32; i++) {
    if (!comps[i].used) return &comps[i];
    if (comps[i].score < comps[worst].score) worst = i;
  }
  return &comps[worst];
}

static void learnCompanions() {
  uint32_t t = nowS();
  for (int i = 0; i < 32; i++) { comps[i].heardLast = comps[i].heardNow; comps[i].heardNow = false; }
  bool isNew[64];
  for (int k = 0; k < nHeard; k++) {
    Comp* c = findComp(heard[k].addr);
    isNew[k] = (c == nullptr);
    if (c) { c->heardNow = true; c->lastRssi = heard[k].rssi; }
  }
  // rotation re-link: a companion vanished this scan and exactly one brand-new
  // address of the same payload class appeared with similar RSSI
  for (int i = 0; i < 32; i++) {
    Comp& c = comps[i];
    if (!c.used || c.score < COMP_TH || c.heardNow || !c.heardLast) continue;
    int match = -1, count = 0;
    for (int k = 0; k < nHeard; k++)
      if (isNew[k] && heard[k].cls == c.cls && abs(heard[k].rssi - c.lastRssi) <= 6) { match = k; count++; }
    if (count == 1) {
      memcpy(c.addr, heard[match].addr, 6);
      c.heardNow = true; c.lastRssi = heard[match].rssi;
      isNew[match] = false;
    }
  }
  for (int i = 0; i < 32; i++) {
    Comp& c = comps[i];
    if (!c.used) continue;
    bool near = c.heardNow && c.lastRssi >= NEAR_DBM;
    c.score = EMA_A * c.score + (1 - EMA_A) * (near ? 1.0f : 0.0f);
    if (c.score >= COMP_TH && c.since == 0) c.since = t;
    if (c.score < COMP_TH) c.since = 0;
    if (c.score < 0.01f && !c.heardNow) c.used = false;
  }
  bool imprinting = t < IMPRINT_S;
  for (int k = 0; k < nHeard; k++) {
    if (!isNew[k] || heard[k].rssi < NEAR_DBM) continue;
    Comp* c = newComp();
    memcpy(c->addr, heard[k].addr, 6);
    c->cls = heard[k].cls; c->lastRssi = heard[k].rssi; c->heardNow = true; c->used = true;
    bool imprint = imprinting && heard[k].rssi >= IMPRINT_DBM;   // the adult at the kiosk
    c->score = imprint ? 0.8f : (1 - EMA_A);
    c->since = imprint ? (t > ARM_S ? t - ARM_S : 1) : 0;
  }
  nHeard = 0;

  // armed companions (stayed long enough; strangers in a moving crowd rarely do)
  int nStrong = 0;
  bool anyHeard = false;
  for (int i = 0; i < 32; i++) {
    Comp& c = comps[i];
    if (!c.used || c.score < STRONG_TH || c.since == 0 || t - c.since < ARM_S) continue;
    if (c.heardNow) anyHeard = true;
    if (nStrong < 2) memcpy(remembered[nStrong], c.addr, 6);
    nStrong++;
  }
  if (nStrong) { nRemembered = min(nStrong, 2); if (!armed) armedSinceS = t; armed = true; }
  if (anyHeard) lastCompanionHeardS = t;
  // possible separation: was armed, now none of the remembered companions heard for SEP_AFTER_S
  bool rememberedHeard = false;
  for (int r = 0; r < nRemembered; r++)
    for (int i = 0; i < 32; i++)
      if (comps[i].used && comps[i].heardNow && memcmp(comps[i].addr, remembered[r], 6) == 0) rememberedHeard = true;
  if (rememberedHeard) lastCompanionHeardS = t;
  separated = armed && t - lastCompanionHeardS >= SEP_AFTER_S;
}

static void advertise(uint32_t slot, uint8_t flags) {
  uint8_t addrBytes[8];
  hmac8(DOMAIN_TAG_ID, slot, curId);
  hmac8(DOMAIN_TAG_ADDR, slot, addrBytes);
  esp_bd_addr_t addr;
  memcpy(addr, addrBytes, 6);
  addr[0] |= 0xC0;                  // static random address: top two bits set
  uint8_t extra[8];
  int extraLen = 0;
  if (flags)                         // share hashes of learned family devices only when flagged
    for (int r = 0; r < nRemembered; r++) { addr_hash4(remembered[r], extra + extraLen); extraLen += 4; }
  adv->stop();
  esp_ble_gap_set_rand_addr(addr);  // MAC rotates together with the payload ID
  BLEAdvertisementData data;
  data.setFlags(0x06);
  data.setManufacturerData(mfgPayload("PK", flags, curId, extra, extraLen).c_str());
  adv->setAdvertisementData(data);
  uint16_t iv = flags ? 160 : 480;  // 100 ms when flagged, 300 ms normally (0.625 ms units)
  adv->setMinInterval(iv);
  adv->setMaxInterval(iv);
  adv->setDeviceAddress(addr, BLE_ADDR_TYPE_RANDOM);
  adv->start();
  Serial.printf("{\"type\":\"tag\",\"slot\":%lu,\"flags\":%d,\"companions\":%d,\"id\":\"", (unsigned long)slot, flags, nRemembered);
  printHex(curId, 8);
  Serial.println("\"}");
}

void setup() {
  Serial.begin(115200);
  pinMode(LED_PIN, OUTPUT);
  pinMode(BUTTON_PIN, INPUT_PULLUP);
  BLEDevice::init("");
  adv = BLEDevice::getAdvertising();
  scan = BLEDevice::getScan();
  scan->setAdvertisedDeviceCallbacks(new TagScanCB(), false);
  scan->setActiveScan(false);
  advertise(local_slot(), 0);
}

void loop() {
  static uint32_t lastSlot = local_slot();
  static uint32_t lastScan = 0, lastButton = 0;
  static uint8_t lastFlags = 0;
  uint32_t now = millis();
  if (now - lastScan > 5000) {        // listen 1 s in every 5 s
    lastScan = now;
    scan->start(1, false);
    scan->clearResults();
    learnCompanions();
  }
  if (digitalRead(BUTTON_PIN) == LOW && now - lastButton > 800) {   // demo LOST toggle
    lastButton = now;
    manualLost = !manualLost;
  }
  uint8_t flags = ((manualLost || cmdLost) ? FLAG_LOST : 0) | (separated ? FLAG_SEPARATED : 0);
  uint32_t slot = local_slot();
  if (slot != lastSlot || flags != lastFlags) {
    lastSlot = slot;
    lastFlags = flags;
    advertise(slot, flags);
  }
  digitalWrite(LED_PIN, flags ? (millis() / 150) % 2 : 0);
  delay(20);
}
#endif  // ROLE_TAG
