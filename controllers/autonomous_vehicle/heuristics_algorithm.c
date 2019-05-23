
#include "heuristics_algorithm.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <time.h>
#include <math.h>

static int count = 1;

#define GSL_RNG_SEED 123
#define UNKNOWN 99999.99

enum{
  verbose = 0,
};


struct rnd{
  const gsl_rng_type * T;
  gsl_rng * r;
};

static struct rnd rnd;

//static struct neighborPoint neighborhood[NVAR];

void heuristics_loadDefault( decisionVar_t* var ){


}

int heuristics_loadParam( decisionVar_t* var ){

  var->var.kp.x = 1.5;
  var->var.ki.x = 0.0002;

  var->var.a.x = 23.45;
  var->var.b.x = -0.2299;
  var->var.brakelimit.x = 20.0;


  count++;
  return count;
}


void heuristics_init( decisionVar_t* var ){

  struct data* data= &var->var;

  gsl_rng_env_setup();
  rnd.T = gsl_rng_default;
  rnd.r = gsl_rng_alloc ( rnd.T  );
  gsl_rng_set(rnd.r,time(NULL) );

  
  data->kp.x = 1.0;
  data->kp.xl = 0;
  data->kp.xu = 4.0;

  data->ki.x  = 2.0;
  data->ki.xl = 1.0;
  data->ki.xu = 3.0;

  data->a.x =  23.0;
  data->a.xl = 10.0;
  data->a.xu = 40.0;

  data->b.x  = 0.2;
  data->b.xl = 0;
  data->b.xu = 2.0;

  data->brakelimit.x = 20.0;
  data->brakelimit.xl = 5.0;
  data->brakelimit.xu = 50.0;

  //for (int i=0; i<NVAR; ++i){
    //points[i].devstd = ( points[i].xu - points[i].xl ) / 10;
    //points[i].mu = ( points[i].xu + points[i].xl ) / 2;
  //}
}

void heuristics_generate_neighbor( neighbor_t* nbh, decisionVar_t* var ){
  
  assert( POINTS_NBH == NVAR );
  printf(" %s%s%s%s%s\r\n", 
          "     [Kp]     ",
          "     [Ki]     ",
          "     [Sa]     ",
          "     [Sb]     ",
          "     [Bp]     ");

	for( int n = 0; n < POINTS_NBH; ++n )
	{
    neighbor_t* nb = &nbh[n];
    for( int i = 0; i < NVAR; ++i ) 
		{
			double val = 0;
      double const sigma = ( var->list[i].xu - var->list[i].xl ) / 5;
      nb->list[i].devstd = sigma;
      if( n == i ){
        do{
          val = gsl_ran_gaussian( rnd.r, sigma ) + var->list[i].x;
        }while( val > var->list[i].xu || val < var->list[i].xl );
        nb->list[i].val = val ;
      }
      else
        nb->list[i].val = var->list[i].x;
      
      printf("  [%f]  ", nb->list[i].val); 
		}
    printf("\r\n"); 
	}
}

void heuristics_get_neighbor( decisionVar_t* var, neighbor_t* nbh )
{
  
  for( int i = 0; i < NVAR; ++i ) {
    // if( nbh->list[i].val > var->list[i].xu )
    //   var->list[i].x = var->list[i].xu;
    // else if( nbh->list[i].val < var->list[i].xl )
    //   var->list[i].x = var->list[i].xl;
    // else
      var->list[i].x = nbh->list[i].val;
  }
}

bool heutistics_evaluate_restrictions( statusVar_t* st, bool finishCycle ){
  if( verbose ){
    printf( " ACCEL    -> X: %f, Y: %f, Z: %f\r\n", st->accel.x, st->accel.y, st->accel.z);
    printf( " DISTANCE -> m: %f\r\n", st->dist);
    printf( " SPEED    -> Speed: %f\r\n", st->speed );
    printf( " ANGLE    -> angle: %f\r\n", st->angle );
  }
  bool err = false;
  if( st->accel.z < 7.0 ){
    ++st->numfail;
    err |= st->numfail > 1;
  }
  else{
    if( st->numfail > 0 ){ --st->numfail; }
  }

  err |= st->angle == UNKNOWN;
  
  if ( finishCycle || err ) { 
    printf( " Finish simulation by %s\r\n", err ? "fail restrictions." : "end cycle." );
    return false;
  }

  return true;
}

void heutistics_print_point( decisionVar_t* var ){

    printf( "\n Point decision var: ");
    for( int i = 0; i < NVAR; ++i ) {
      printf( " %f,", var->list[i].x );  
    }
    printf( "\r\n");
}