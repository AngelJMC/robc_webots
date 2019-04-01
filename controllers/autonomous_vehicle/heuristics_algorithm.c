
#include "heuristics_algorithm.h"
#include <stdio.h>

static int count = 1;

// Line following PID
/*#define KP 0.75
#define KI 0.006
#define KD 2*/



int heuristics_loadParam( decisionVar_t* var ){

  var->pidSteering.kp = 2;
  var->pidSteering.ki = 0.006;
  var->pidSteering.kd = 0.75;

  var->pidSpeed.kp = 1;
  var->pidSpeed.ki = 0.006;
  var->pidSpeed.kd = 1;


  count++;
  return count;
}

void heuristics_update( ){
}

void heutistics_checkRestrictions(){
}

void heutistics_evaluate_restrictions( statusVar_t* st ){

  
}