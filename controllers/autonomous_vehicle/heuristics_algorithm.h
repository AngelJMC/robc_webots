

#ifndef HEURISTICS_ALGORITHM_H
#define HEURISTICS_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>
#include "../src/robc_heuristics.h"


enum{
  POINTS_NBH = 5,
  NORMAL_ITER_IN_NBH = 7,
  MAX_ITER_IN_NBH = 9,
};


struct point{
  double val;
  double devstd;
};


typedef union
{
  struct{
      struct point kp;
      struct point ki;
      struct point a;
      struct point b;
      struct point brakelimit;
  }pt;
  struct point list[NVAR];
}neighbor_t;




int heuristics_loadParam( decisionVar_t* var );

void heuristics_generate_neighbor( neighbor_t* nbh, decisionVar_t* var , double bestrsl);

void heuristics_init( decisionVar_t* var );

void heuristics_get_neighbor( decisionVar_t* var, neighbor_t* nbh );

bool heuristics_is_finish_neighbor_search( decisionVar_t* var );

void heuristics_intensify_neighbor_search( decisionVar_t* var );

int heuristics_get_range( decisionVar_t* var );

void heuristics_print_neighborhood( neighbor_t* nbh );
#endif /*HEURISTICS_ALGORITHM_H*/
