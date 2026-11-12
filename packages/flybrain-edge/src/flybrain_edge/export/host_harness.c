/* Host-side harness for the generated fly_escape.c (used by the equivalence test).
 * argv[1]: 0 = connectome, 1 = looming baseline. stdin: one line per step,
 * "raw_l raw_r dt_ms", or "R" to reset the model. stdout per step:
 * "trigger side gf_l gf_r drive". Baselines are fixed at the Python default (3400). */
#include <stdio.h>
#include <stdlib.h>

#include "fly_escape.h"

int main(int argc, char **argv) {
  static fe_state_t s;
  char line[256];
  int model = argc > 1 ? atoi(argv[1]) : 0;
  fe_init(&s, (uint8_t)model);
  while (fgets(line, sizeof line, stdin)) {
    double rl, rr, dt;
    if (line[0] == 'R') {
      fe_reset(&s);
      continue;
    }
    if (sscanf(line, "%lf %lf %lf", &rl, &rr, &dt) != 3) continue;
    {
      fe_result_t r = fe_step(&s, (float)rl, (float)rr, (float)dt);
      printf("%d %d %.9g %.9g %.9g\n", r.trigger, r.side, s.gf_l, s.gf_r, r.drive);
    }
  }
  return 0;
}
