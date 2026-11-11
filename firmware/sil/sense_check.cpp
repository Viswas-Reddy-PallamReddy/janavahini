// Equivalence harness for the firmware density estimator (include/pk_sense.h).
// Reads inputs written by sim/test_sense_port.py, runs the estimator of every node, and
// writes its estimate; the Python test runs sensing.kf_step on the same inputs and compares.
//
// Usage: sense_check <in.txt> <out.txt>
// in.txt:  "T n", then n lines "area counted observed",
//          then T*n lines "cin cout due z fault"   (fault: 0 none, 1 naive, 2 flag; applied
//          before that second's step, as site_experiment.run does)
// out.txt: T lines of n estimates
#include <cstdio>
#include <vector>

#include "../include/pk_sense.h"

using namespace pksense;

int main(int argc, char** argv) {
  if (argc < 3) { fprintf(stderr, "usage: sense_check in out\n"); return 2; }
  FILE* fi = fopen(argv[1], "r");
  int T, n;
  if (!fi || fscanf(fi, "%d %d", &T, &n) != 2) { fprintf(stderr, "bad input\n"); return 2; }
  SenseParams p;
  std::vector<DensityEstimator> est(n);
  for (int i = 0; i < n; i++) {
    float area;
    int c, o;
    if (fscanf(fi, "%f %d %d", &area, &c, &o) != 3) return 2;
    est[i].init(area, c != 0, o != 0, p);
  }
  FILE* fo = fopen(argv[2], "w");
  if (!fo) return 2;
  for (int t = 0; t < T; t++) {
    for (int i = 0; i < n; i++) {
      float cin, cout, z;
      int due, fault;
      if (fscanf(fi, "%f %f %d %f %d", &cin, &cout, &due, &z, &fault) != 5) { fprintf(stderr, "short input\n"); return 2; }
      if (fault) est[i].sensorFault(fault == 2, p);
      float r = est[i].step(cin, cout, due != 0, z, p);
      fprintf(fo, i ? " %.6f" : "%.6f", r);
    }
    fprintf(fo, "\n");
  }
  fclose(fi);
  fclose(fo);
  return 0;
}
