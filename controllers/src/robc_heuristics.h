#ifndef ROBC_HEURISTICS_H
#define ROBC_HEURISTICS_H

#include <stdbool.h>
#include <stdint.h>

enum{
    NVAR = 5,
};

enum{
  SML_CONTINUE = 0,
  SML_FINISH = 1,
  SML_ERROR_RESTRICTION = 2,
};


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


int heuristics_evaluate_restrictions( statusVar_t* st, bool finishCycle );

double heuristics_get_objetive( statusVar_t* st );

void heuristics_print_point( decisionVar_t* var );

double float_rand( double min, double max );

#endif
