// Software-in-the-loop (SIL) harness: runs many virtual ESP32 nodes, each
// executing the firmware's own alert code (include/pk_qs.h), on a PC.
//
// Each node has its own boot time and loop period (BLE scan ~1 s + work, so a
// little over 1 s, with jitter), reads its density from a crowd-simulation
// trace, and broadcasts its RouteMsg as raw bytes, as esp_now_send would. The
// radio delivers each message to the nodes that listen to the sender, losing it
// with probability `loss` and delaying it by a random latency. Nodes can be
// killed at a given time (power or hardware failure).
//
// --ideal: lock-step 1.000 s loops, no loss, and every message arrives exactly at
// the next tick, which matches the Python model (sim/qs_core.py) step for step.
//
// Usage: sil <graph.txt> <trace.txt> <out.txt> [--ideal] [--loss p] [--lat-min ms]
//            [--lat-max ms] [--period-min ms] [--period-max ms] [--jitter ms]
//            [--seed n] [--failsafe ms|forever] [--escalate] [--backup] [--kill node:seconds ...]
// --backup: every node also listens to its successors' successors (2-hop backup)
// graph.txt: "n m", then n lines "id kind", then m lines "u v share walk_s"
// trace.txt: "T n", then T lines of n densities (one line per second)
// out.txt:   one line per node tick: "t_ms node state action L relayedL divertMask downMask"
#include <cstdio>
#include <cstdlib>
#include <cstring>
#include <queue>
#include <random>
#include <vector>

#include "../include/pk_qs.h"

using namespace pkqs;

struct Event {
  uint64_t t;
  int type;       // 0 delivery (before ticks at the same time), 1 tick
  uint64_t order;
  int node;
  std::vector<uint8_t> bytes;
  bool operator>(const Event& o) const {
    if (t != o.t) return t > o.t;
    if (type != o.type) return type > o.type;
    return order > o.order;
  }
};

int main(int argc, char** argv) {
  if (argc < 4) { fprintf(stderr, "usage: sil graph trace out [options]\n"); return 2; }
  bool ideal = false;
  double loss = 0.3, latMin = 2, latMax = 15, perMin = 1005, perMax = 1060, jitter = 5;
  unsigned seed = 1;
  uint32_t failsafe = 0;
  bool backup = false, escalate = false;
  std::vector<std::pair<int, double>> kills;
  for (int i = 4; i < argc; i++) {
    std::string a = argv[i];
    auto next = [&]() { return atof(argv[++i]); };
    if (a == "--ideal") ideal = true;
    else if (a == "--loss") loss = next();
    else if (a == "--lat-min") latMin = next();
    else if (a == "--lat-max") latMax = next();
    else if (a == "--period-min") perMin = next();
    else if (a == "--period-max") perMax = next();
    else if (a == "--jitter") jitter = next();
    else if (a == "--seed") seed = (unsigned)next();
    else if (a == "--failsafe") {
      std::string v = argv[++i];
      failsafe = v == "forever" ? FAILSAFE_UNTIL_RESTORED : (uint32_t)atof(v.c_str());
    }
    else if (a == "--backup") backup = true;
    else if (a == "--escalate") escalate = true;
    else if (a == "--kill") {
      std::string s = argv[++i];
      size_t c = s.find(':');
      kills.push_back({atoi(s.substr(0, c).c_str()), atof(s.substr(c + 1).c_str())});
    }
  }
  if (ideal) { loss = 0; jitter = 0; }

  FILE* fg = fopen(argv[1], "r");
  int n, m;
  if (!fg || fscanf(fg, "%d %d", &n, &m) != 2) { fprintf(stderr, "bad graph\n"); return 2; }
  std::vector<Node> nodes(n);
  std::vector<std::vector<int>> listeners(n);    // who uses node v's broadcast (its predecessors)
  for (int i = 0; i < n; i++) {
    int id, kind;
    if (fscanf(fg, "%d %d", &id, &kind) != 2) return 2;
    nodes[i].id = (uint16_t)id;
    nodes[i].kind = (uint8_t)kind;
    nodes[i].failsafeMs = failsafe;
    nodes[i].failsafeEscalate = escalate;
  }
  for (int e = 0; e < m; e++) {
    int u, v;
    float share, walk;
    if (fscanf(fg, "%d %d %f %f", &u, &v, &share, &walk) != 4) return 2;
    if (!nodes[u].addSucc((uint16_t)v, share, walk)) { fprintf(stderr, "node %d: too many successors\n", u); return 2; }
    listeners[v].push_back(u);
  }
  fclose(fg);
  if (backup) {
    std::vector<int> nPrim(n);
    for (int u = 0; u < n; u++) nPrim[u] = nodes[u].nSucc;
    for (int u = 0; u < n; u++)
      for (int k = 0; k < nPrim[u]; k++) {
        const Succ p = nodes[u].succ[k];
        const Node& v = nodes[p.id];
        for (int j = 0; j < nPrim[p.id]; j++) {
          int w = v.succ[j].id;
          if (!nodes[u].addBackup(k, (uint16_t)w, p.share * v.succ[j].share, p.walk + v.succ[j].walk)) {
            fprintf(stderr, "node %d: too many successors with backups\n", u);
            return 2;
          }
          listeners[w].push_back(u);
        }
      }
  }

  FILE* ft = fopen(argv[2], "r");
  int T, nt;
  if (!ft || fscanf(ft, "%d %d", &T, &nt) != 2 || nt != n) { fprintf(stderr, "bad trace\n"); return 2; }
  std::vector<float> trace((size_t)T * n);
  for (size_t i = 0; i < trace.size(); i++)
    if (fscanf(ft, "%f", &trace[i]) != 1) { fprintf(stderr, "short trace\n"); return 2; }
  fclose(ft);

  std::mt19937_64 rng(seed);
  std::uniform_real_distribution<double> U(0.0, 1.0);
  std::vector<double> period(n), killAt(n, 1e18);
  for (auto& k : kills) if (k.first >= 0 && k.first < n) killAt[k.first] = k.second * 1000.0;

  std::priority_queue<Event, std::vector<Event>, std::greater<Event>> q;
  uint64_t order = 0;
  for (int i = 0; i < n; i++) {
    period[i] = ideal ? 1000.0 : perMin + (perMax - perMin) * U(rng);
    uint64_t boot = ideal ? 0 : (uint64_t)(1000.0 * U(rng));
    q.push({boot, 1, order++, i, {}});
  }

  FILE* fo = fopen(argv[3], "w");
  if (!fo) return 2;
  const uint64_t tEnd = (uint64_t)T * 1000;
  uint64_t sent = 0, delivered = 0, lost = 0, bytes = 0, ticks = 0;
  while (!q.empty()) {
    Event ev = q.top();
    q.pop();
    if (ev.t >= tEnd) continue;
    Node& nd = nodes[ev.node];
    if (ev.t >= killAt[ev.node]) continue;                      // dead node: no rx, no tx
    if (ev.type == 0) {
      RouteMsg msg;
      if (ev.bytes.size() != sizeof(RouteMsg)) continue;        // what onRecv's length check does
      memcpy(&msg, ev.bytes.data(), sizeof(msg));
      if (msg.magic == 'Q') nd.onMsg(msg, (uint32_t)ev.t);
      delivered++;
      continue;
    }
    // loop tick: read the sensor (latest whole second of the trace), run the QS step, broadcast
    size_t sec = (size_t)(ev.t / 1000);
    RouteMsg out = nd.step(trace[sec * n + ev.node], (uint32_t)ev.t);
    ticks++;
    fprintf(fo, "%llu %d %d %d %.5f %.5f %d %d\n", (unsigned long long)ev.t, ev.node, nd.state, nd.action,
            nd.bestL, nd.relayedL, nd.divertMask, nd.downMask);
    std::vector<uint8_t> raw(sizeof(RouteMsg));
    memcpy(raw.data(), &out, sizeof(out));
    sent++;
    bytes += raw.size();
    for (int dst : listeners[ev.node]) {
      if (U(rng) < loss) { lost++; continue; }
      uint64_t at = ideal ? ev.t + 1000 : ev.t + (uint64_t)(latMin + (latMax - latMin) * U(rng));
      q.push({at, 0, order++, dst, raw});
    }
    double next = ev.t + period[ev.node] + (jitter > 0 ? (U(rng) * 2 - 1) * jitter : 0.0);
    q.push({(uint64_t)next, 1, order++, ev.node, {}});
  }
  fclose(fo);
  printf("{\"nodes\": %d, \"ticks\": %llu, \"sent\": %llu, \"delivered\": %llu, \"lost\": %llu, "
         "\"bytes_sent\": %llu, \"msg_bytes\": %zu, \"node_state_bytes\": %zu}\n",
         n, (unsigned long long)ticks, (unsigned long long)sent, (unsigned long long)delivered,
         (unsigned long long)lost, (unsigned long long)bytes, sizeof(RouteMsg), sizeof(Node));
  return 0;
}
