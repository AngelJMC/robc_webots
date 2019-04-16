

#ifndef HEURISTICS_ALGORITHM_H
#define HEURISTICS_ALGORITHM_H

#include <stdbool.h>
#include <stdint.h>


typedef struct var{

    double kp;
    double ki;
    double kd;


    double a;
    double b;
    double brakelimit;

}decisionVar_t;

typedef struct {
  double speed;
  struct {
    double x;
    double y;
    double z;
  } accel;
  double encoder;
} statusVar_t;


int heuristics_loadParam( decisionVar_t* var );

void heutistics_evaluate_restrictions( statusVar_t* st, bool finishCycle );

#endif /*HEURISTICS_ALGORITHM_H*/
