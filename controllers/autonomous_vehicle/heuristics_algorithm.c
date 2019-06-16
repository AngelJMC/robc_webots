
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
  gsl_rng * rid;
};

static struct rnd rnd;

void _generate_neighbor_normal( neighbor_t* nb, decisionVar_t* var );
void _generate_neighbor_random( neighbor_t* nb, decisionVar_t* var ); 
void _generate_neighbor_close( neighbor_t* nb, decisionVar_t* var );
int _get_range( decisionVar_t* var );



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

  assert( NORMAL_ITER_IN_NBH < MAX_ITER_IN_NBH );

  struct data* data= &var->var;
  
  gsl_rng_env_setup();
  rnd.T = gsl_rng_default;
  rnd.r = gsl_rng_alloc ( rnd.T  );
  rnd.rid = gsl_rng_alloc ( rnd.T  );
  gsl_rng_set(rnd.r,time(NULL) );
  gsl_rng_set(rnd.rid,time(NULL) );
  srand( time(0) );

  data->kp.xl = 0;
  data->kp.xu = 4.0;
  data->kp.x = float_rand( data->kp.xl, data->kp.xu );

  
  data->ki.xl = 1.0;
  data->ki.xu = 3.0;
  data->ki.x = float_rand( data->ki.xl, data->ki.xu );

  
  data->a.xl = 10.0;
  data->a.xu = 40.0;
  data->a.x = float_rand( data->a.xl, data->a.xu );

  
  data->b.xl = 0;
  data->b.xu = 2.0;
  data->b.x = float_rand( data->b.xl, data->b.xu );

  
  data->brakelimit.xl = 5.0;
  data->brakelimit.xu = 50.0;
  data->brakelimit.x = float_rand( data->brakelimit.xl, data->brakelimit.xu );

  for( int i = 0; i < NVAR; ++i){
    var->list[i].range = 1;
  }

}


bool heuristics_is_finish_neighbor_search( decisionVar_t* var ){

    return var->list[0].range > MAX_ITER_IN_NBH;
}


void heuristics_intensify_neighbor_search( decisionVar_t* var ){
  for( int i = 0; i < NVAR; ++i){
    ++var->list[i].range;
  }
  printf("\nLevel of search intensification: %d\r\n  ", var->list[0].range);
}


void heuristics_generate_neighbor( neighbor_t* nbh, decisionVar_t* var, double bestrsl ){
  
    printf("\nGenerate neighborhood: " );
    int iter_range = _get_range( var);
    
    if ( bestrsl == 0 ){
        printf(" random type\n" );
        for( int n = 0; n < POINTS_NBH; ++n ){
            _generate_neighbor_random( &nbh[n] , var);
        }
    }
    else if( iter_range < NORMAL_ITER_IN_NBH ){
        printf(" normal type\n" );
        for( int n = 0; n < POINTS_NBH; ++n ){
            _generate_neighbor_normal( &nbh[n], var );
        }
    }
    else{
        printf(" close type\n" );
        for( int n = 0; n < POINTS_NBH; ++n ){
            _generate_neighbor_close( &nbh[n], var );
        }
    }
    heuristics_print_neighborhood( nbh );
}


void heuristics_get_neighbor( decisionVar_t* var, neighbor_t* nbh )
{
    for( int i = 0; i < NVAR; ++i ) {
        var->list[i].x = nbh->list[i].val;
    }
}


void heuristics_print_neighborhood( neighbor_t* nbh ){
    
    printf(" %s%s%s%s%s\r\n", 
          "     [Kp]     ",
          "     [Ki]     ",
          "     [Sa]     ",
          "     [Sb]     ",
          "     [Bp]     ");

    for( int n = 0; n < POINTS_NBH; ++n )
	{
        neighbor_t* nb = &nbh[n];
        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", nb->list[i].val);  
        } 
        printf("\r\n"); 
    }

}


int _get_range( decisionVar_t* var ){
    return var->list[0].range;
}


void _generate_neighbor_normal( neighbor_t* nb, decisionVar_t* var ){
  

    int const id1 = (gsl_rng_get( rnd.rid ) + (int)var->list[0].x) % NVAR;
    int const id2 = (gsl_rng_get( rnd.rid ) + (int)var->list[3].x) % NVAR;
    for( int i = 0; i < NVAR; ++i ) {
		double val = 0;
        double const sigma = ( var->list[i].xu - var->list[i].xl ) / pow( 2, var->list[i].range );
        nb->list[i].devstd = sigma;
      
        if( id1 == i || id2 == i){
            do{
                val = gsl_ran_gaussian( rnd.r, sigma ) + var->list[i].x;
            }while( val > var->list[i].xu || val < var->list[i].xl );
            nb->list[i].val = val ;
        }
        else
            nb->list[i].val = var->list[i].x;
	}

}


void _generate_neighbor_random( neighbor_t* nb, decisionVar_t* var ) {

    for( int i = 0; i < NVAR; ++i ) {
		double val = 0;
        double const sigma = float_rand( var->list[i].xl, var->list[i].xu );
        nb->list[i].devstd = sigma;
        val = float_rand( var->list[i].xl, var->list[i].xu );
        nb->list[i].val = val ;
	}
}


void _generate_neighbor_close( neighbor_t* nb, decisionVar_t* var ){
  
    int const id1 = (gsl_rng_get( rnd.rid ) + (int)var->list[0].x) % NVAR;
    int const id2 = (gsl_rng_get( rnd.rid ) + (int)var->list[3].x) % NVAR;
    for( int i = 0; i < NVAR; ++i ) {
		double val = 0;
        double const sigma = ( var->list[i].xu - var->list[i].xl ) / pow( 2, var->list[i].range );
        nb->list[i].devstd = sigma;
      
        if( id1 == i || id2 == i){
            do{
                unsigned long int r = gsl_rng_get( rnd.rid );
                val = var->list[i].x + ( (r % 2 )? 1 : -1) * sigma ;
            }while( val > var->list[i].xu || val < var->list[i].xl );
            nb->list[i].val = val ;
        }
        else
            nb->list[i].val = var->list[i].x;
	}
}