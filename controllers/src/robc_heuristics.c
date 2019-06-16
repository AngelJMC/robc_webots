#include "robc_heuristics.h"
#include <stdbool.h>
#include <stdint.h>
#include <stdio.h>
#include <assert.h>
#include <stdlib.h>


#define UNKNOWN 99999.99

enum{
  verbose = 0,
};


int heuristics_evaluate_restrictions( statusVar_t* st, bool finishCycle ){
  if( verbose ){
    printf( " ACCEL    -> X: %f, Y: %f, Z: %f\r\n", st->accel.x, st->accel.y, st->accel.z);
    printf( " DISTANCE -> m: %f\r\n", st->dist);
    printf( " SPEED    -> Speed: %f\r\n", st->speed );
    printf( " ANGLE    -> angle: %f\r\n", st->angle );
  }

  if( st->accel.z < 7.0 ) 
    ++st->numfail;
  else
    if( st->numfail > 0 ){ --st->numfail; }
  
  if ( st->angle == UNKNOWN || st->numfail > 1 ){
    printf( "  Finish simulation by restrictions: %s\n", st->angle == UNKNOWN? "Fail angle detection" : "Fail acceleration" );
    return SML_ERROR_RESTRICTION;
  }

  if ( finishCycle ) { 
    printf( "  Finish simulation by end cycle.\n" );
    return SML_FINISH;
  }

  return SML_CONTINUE;
} 


double heuristics_get_objetive( statusVar_t* st ){
    return st->dist;
}

double float_rand( double min, double max )
{
    double scale = rand() / (double) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}


void heuristics_print_point( decisionVar_t* var ){

    printf( "  Point decision var: ");
    for( int i = 0; i < NVAR; ++i ) {
      printf( " %f,", var->list[i].x );  
    }
    printf( "\r\n");
}