#include "../src/robc_heuristics.h"
#include "heuristic_ga.h"
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <string.h>
#include <assert.h>
#include <gsl/gsl_rng.h>
#include <gsl/gsl_randist.h>
#include <gsl/gsl_cdf.h>
#include <time.h>
#include <math.h>
#include <assert.h>





double float_rand( double min, double max )
{
    double scale = rand() / (double) RAND_MAX; /* [0, 1.0] */
    return min + scale * ( max - min );      /* [min, max] */
}

void ga_init( decisionVar_t* var ){

  assert( NORMAL_ITER_IN_NBH < MAX_ITER_IN_NBH );

  struct data* data= &var->var;
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
}


void print_member( population_t* pplt){

        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", pplt->var.plist[i] );  
        } 
        printf("  [%f]  ", pplt->res );
        printf("\r\n"); 
}

void print_couple( couple_t* prts ){
    for( int i = 0; i < 2; ++i ){
        printf("Couple %d\r\n", i +1); 
        print_member( &prts->list[i] );
    }
}

void ga_generaterandMember( decisionVar_t* var ){

    for( int i = 0; i < NVAR; ++i )
         var->list[i].x = float_rand( var->list[i].xl, var->list[i].xu );

}

void ga_pushMemberToPopulation( population_t* pplt, decisionVar_t* var, double dist ){
    for( int i = 0; i < NVAR; ++i ) {
            pplt->var.plist[i] = var->list[i].x;
    }
    pplt->res = dist;

}

void ga_generatePopulation( population_t* pplt, decisionVar_t* var ){

    for( int n = 0; n < NPOPULATION; ++n ){
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



int selectRuleta( population_t const* pplt ){
    double sumres = 0;
    for( int i = 0; i < NPOPULATION; ++i){
        sumres += pplt[i].res;
    }
    
    double const rnd = float_rand( 0.0 , sumres );
    
    double tempsum = 0;
    int best = NPOPULATION-1;
    for( int i = 0; i < NPOPULATION; ++i){
        tempsum += pplt[i].res;
        if( tempsum >= rnd ) {
            printf( "Sumres: %f ,Rand %f, Best: %d\n", sumres, rnd, i );
            return i;
        }
    }
    
    return best;
}


void ga_select( couple_t* prts, population_t* pplt ){
   
    for( int i = 0; i < NUM_PARENTS; ++i){
        for (int j = 0; j < 2; ++j ){
            int id = selectRuleta( pplt );
            prts[i].list[j]= pplt[id];
        }   
    }
}

void get_crosschilds ( couple_t* childs, couple_t* prts  ){
    
    int k = (int)float_rand( 0.0 , NVAR-1 ); /*Punto de cruce*/
    double alpha = ((double) rand() / (RAND_MAX)) + 1;
    assert( 0 <= k && NVAR > k);
    
    population_t* p1 = &prts->list[0];
    population_t* p2 = &prts->list[1];

    for (int j = 0; j < 2 ; ++j){ 
        population_t* ch = &childs->list[j];
        for(int i = 0; i < NVAR; ++i ){
        
            if( i < k ){
                ch->var.plist[i]  = prts->list[j].var.plist[i];
            }
            else
            {
                double const crss = ( j == 0 ) ? 
                    ( alpha * p1->var.plist[i] + ( 1.0 - alpha) * p2->var.plist[i] ) : 
                    ( ( 1.0 - alpha) * p1->var.plist[i] + (alpha) * p2->var.plist[i]);
                ch->var.plist[i] = crss;
            }   
        }
    }

}


void ga_pcross( couple_t* childs, couple_t* prts ){

    for ( int j = 0; j < 2; ++j){
        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", prts->list[j].var.plist[i] );  
        } 
        printf("  ---->  "); 
        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", childs->list[j].var.plist[i] );  
        } 
        printf("\r\n"); 
    }
    
    //printf("  [%f]  ", pplt->res );
    printf("\r\n"); 

}

void ga_cross( couple_t* childs, couple_t* prts ){
    for( int i = 0; i < NUM_PARENTS; ++i){

        get_crosschilds ( &childs[i], &prts[i]  );
        ga_pcross( &childs[i], &prts[i] );
    }

}


void mutation_population( population_t* pplt, decisionVar_t* var ){

    for(int i = 0; i < NVAR; ++i ){
        double m = ((double) rand() / (RAND_MAX));
        if( m < 0.2)
            pplt->var.plist[i] = float_rand( var->list[i].xl, var->list[i].xu );
        
    }
}

void ga_mutation( couple_t* childs , decisionVar_t* var){
    
    
    for( int i = 0; i < NUM_CHILDS; ++i){
        for( int j = 0; j < 2; ++j ){
            mutation_population( &childs[i].list[j], var );
        }
    }
}

void ga_getBestMember( population_t* pbest, population_t* pplt ){
    int best = 0;
    double bestres = 0; 
    for( int i = 0; i < NPOPULATION; ++i ){
        population_t* pl = &pplt[i]; 
        if( pl->res > bestres ){
            best = i;
            memcpy( pbest, pl , sizeof( population_t ) );
        }        
    }
    printf(" Best solution id %d\r\n", best);
}

void ga_recombination( population_t* pplt, couple_t* childs, population_t* pbestglobal ){

    
    population_t pbest = { .res = 0 } ;
    ga_getBestMember( &pbest, pplt );


    for( int i = 0; i < NUM_CHILDS; ++i ){
        for( int j = 0; j < 2; ++j){
            int idx = 2*i + j;
            pplt[idx] = childs[i].list[j];
            pplt[idx].res = 0;
        }
    }


    int p = (int)float_rand( 0.0 , NPOPULATION-1 ); 
    printf(" Best solution of last generation: %d\r\n", p);
    print_member( &pbest ); 
    pplt[p] = pbest;
    
    p = (int)float_rand( 0.0 , NPOPULATION-1 );
    printf(" Best solution of all generation: %d\r\n", p);
    print_member( pbestglobal ); 
    memcpy( &pplt[p], pbestglobal, sizeof(population_t ));
        
    printf(" ----------------\r\n");

    

}


void ga_get_memberFromPopulation( decisionVar_t* var, population_t* pplt )
{
    for( int i = 0; i < NVAR; ++i ) {
        var->list[i].x = pplt->var.plist[i];
    }
}



void ga_printpoulation( population_t* pplt){
    
    printf(" %s%s%s%s%s\r\n", 
          "     [Kp]     ",
          "     [Ki]     ",
          "     [Sa]     ",
          "     [Sb]     ",
          "     [Bp]     ");

    for( int n = 0; n < NPOPULATION; ++n )
	{
        print_member( &pplt[n] );
    }

}