
#include "heuristics_algorithm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>

static int count = 1;

// Line following PID
/*#define KP 0.75
#define KI 0.006
#define KD 2*/

  //double a = 23.45;
  //double b = -0.2299;
  //double const brakelimit = 20.0;
enum{
  verbose = 1,
};

int heuristics_loadParam( decisionVar_t* var ){

  var->kp = 1.5;
  var->ki = 0.0002;
  var->kd = 0;

  var->a = 23.45;
  var->b = -0.2299;
  var->brakelimit = 20.0;


  count++;
  return count;
}


void heutistics_evaluate_restrictions( statusVar_t* st, bool finishCycle ){
  if( verbose ){
    printf( " ACCEL -> X: %f, Y: %f, Z: %f\r\n", st->accel.x, st->accel.y, st->accel.z);
    printf( " DISTANCE -> Encoder: %f\r\n", st->encoder);
    printf( " SPEED -> Speed: %f\r\n", st->speed );
  }
  if ( finishCycle ) printf( " FINISH SIMULATION... \r\n");
}