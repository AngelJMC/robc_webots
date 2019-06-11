

#ifndef HEURISTICS_ALGORITHM_H
#define HEURISTICS_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>

enum{
    NVAR = 5,
};

enum{
  POINTS_NBH = 5,
  NORMAL_ITER_IN_NBH = 7,
  MAX_ITER_IN_NBH = 9,
};

enum{
  SML_CONTINUE = 0,
  SML_FINISH = 1,
  SML_ERROR_RESTRICTION = 2,
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


struct dvar{
  double xl;   //lower limit
  double xu;   // uper limit
  double x;
  int    range;
};

typedef union
{

  struct data{
      struct dvar kp;
      struct dvar ki;
      struct dvar a;
      struct dvar b;
      struct dvar brakelimit;
  }var;
  struct dvar list[NVAR];
}decisionVar_t;


typedef struct {
  double speed;
  struct {
    double x;
    double y;
    double z;
  } accel;
  double dist;
  double offsetRad;
  double angle;
  int numfail;
  int numfailAngle;

} statusVar_t;



int heuristics_loadParam( decisionVar_t* var );

void heuristics_generate_neighbor( neighbor_t* nbh, decisionVar_t* var , double bestrsl);

void heuristics_init( decisionVar_t* var );

int heutistics_evaluate_restrictions( statusVar_t* st, bool finishCycle );

double heuristics_get_objetive( statusVar_t* st );

void heuristics_get_neighbor( decisionVar_t* var, neighbor_t* nbh );

void heutistics_print_point( decisionVar_t* var );

bool heuristics_is_finish_neighbor_search( decisionVar_t* var );

void heuristics_intensify_neighbor_search( decisionVar_t* var );

int heuristics_get_range( decisionVar_t* var );

void heuristics_print_neighborhood( neighbor_t* nbh );
#endif /*HEURISTICS_ALGORITHM_H*/
