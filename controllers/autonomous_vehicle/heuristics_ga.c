#include "heuristics_algorithm.h"
#include "heuristic_ga.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <time.h>
#include <math.h>

void ga_init( decisionVar_t* var ){

  assert( NORMAL_ITER_IN_NBH < MAX_ITER_IN_NBH );

  struct data* data= &var->var;
  srand( time(0) );

  data->kp.xl = 0;
  data->kp.xu = 4.0;
  data->kp.x = _float_rand( data->kp.xl, data->kp.xu );

  
  data->ki.xl = 1.0;
  data->ki.xu = 3.0;
  data->ki.x = _float_rand( data->ki.xl, data->ki.xu );

  
  data->a.xl = 10.0;
  data->a.xu = 40.0;
  data->a.x = _float_rand( data->a.xl, data->a.xu );

  
  data->b.xl = 0;
  data->b.xu = 2.0;
  data->b.x = _float_rand( data->b.xl, data->b.xu );

  
  data->brakelimit.xl = 5.0;
  data->brakelimit.xu = 50.0;
  data->brakelimit.x = _float_rand( data->brakelimit.xl, data->brakelimit.xu );

  for( int i = 0; i < NVAR; ++i){
    var->list[i].range = 1;
  }

}


double float_rand( double min, double max )
{
    double scale = rand() / (double) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}



void ga_generate( population_t* pplt, decisionVar_t* var ){

    for( int n = 0; n < POINTS_NBH; ++n ){
        population_t* pl = &pplt[n];        
        for( int i = 0; i < NVAR; ++i ) {
            double val = 0;
            val = float_rand( var->list[i].xl, var->list[i].xu );
            pl->var.plist[i] = val ;
        }
    }
}

void ga_registerSolution( population_t* pplt, double res ){
    pplt->res = res;
}

enum{
    NUM_PARENTS = 4,
    NUM_CHILDS = NUM_PARENTS,
};

int selectRuleta( population_t const* pplt ){
    double sumres = 0;
    for( int i = 0; i < NUM_PARENTS; ++i){
        sumres += pplt[i].res;

    double const rnd = float_rand( 0.0 , sumres );
    double tempsum = 0;
    int best = NUM_PARENTS-1;
    for( int i = 0; i < NUM_PARENTS; ++i){
        tempsum += pplt[i].res;
        if( tempsum >= rnd ) {
            return best;
        }
    }
    return best;
}


void ga_select( parents_t* prts, population_t* pplt ){

    for( int i = 0; i < NUM_PARENTS; ++i){
        for (int j = 0; j < 2; ++j ){
            int id = selectRuleta( pplt );
            prts[i].list[j]= pplt[id];
        }   
    }
}

void get_crosschilds ( parents_t* childs, parents_t* prts  ){
    
    int k = (int)float_rand( 0.0 , NVAR-1 ); /*Punto de cruce*/
    double alpha = ((double) rand() / (RAND_MAX)) + 1;
    assert( 0 <= k && NVAR > k);
    
    population_t* p1 = &prts->list[0];
    population_t* p2 = &prts->list[1];

    for int j = 0; j < 2 ; ++j){ 
        population_t* ch = &childs->list[j];
        for(int i = 0; i < NVAR; ++i ){
        
            if( i < 2 ){
                ch->var.plist[i]  = prts->list[j];
            }
            else
            {
                double const crss = ( j == 0 ) ? 
                    ( alpha * p1->var.plist[i] + ( 1.0 - alpha) * p2->var.plist[i] ) : 
                    ( ( 1.0 - alpha) * p1->var.plist[i] + (alpha) * p2->var.plist[i]) 
                ch->var.plist[i] = crss;
            }   
        }
    }

}

void ga_cross( parents_t* childs, parents_t* prts ){
    for( int i = 0; i < NUM_PARENTS; ++i){
        get_crosschilds ( childs[i], prts[i]  );
    }

}

void mutation_population( population_t* pplt, decisionVar_t* var ){

    struct data* data= &var->var;
    for(int i = 0; i < NVAR; ++i ){
        double m = ((double) rand() / (RAND_MAX)) + 1;
        if( m < 0.02)
        pplt->var.list[i] = _float_rand( data->list[i].xl, data->list[i].xu );
        
    }
}

void ga_mutation( parents_t* childs , decisionVar_t* var){
    
    
    for( int i = 0; i < NUM_CHILDS; ++i){
        for( int j = 0; j < 2; ++j ){
            mutation_population( childs[i].lis[j], var );
        }
    }
}

void ga_recombination( population_t * pplt, parents_t* childs ){

    int best = 0;
    double bestres = 0; 
    for( int i = 0; i < POINTS_NBH; ++i ){
        population_t* pl = &pplt[i]; 
        best = pl->res > bestres ? i : best;
    }

    population_t bestsol = pplt[best];

    for( int i = 0; i < NUM_CHILDS; ++i ){
        for( int j = 0; j < 2; ++j){
            if( i >= POINTS_NBH ) break;
            pplt[i] = childs[i].list[j];
        }
    }

    pplt[0] = bestsol;

}