
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "../src/robc_heuristics.h"
#include "heuristic_ga.h"

enum{
    verbose = 1
};

void mutation_population( population_t* pplt, decisionVar_t const* var );
int RouletteSelection ( population_t const* pplt );
void print_couple( couple_t const* prts );
void print_member( population_t const* pplt);
void getBestFromPopulation( population_t* pbest, population_t const* plist );
void get_crosschilds ( couple_t* childs, couple_t const* prts, decisionVar_t const* best  );
void ga_pcross( couple_t const* childs, couple_t const* prts );

void ga_init( decisionVar_t* var ){

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



void ga_generaterandMember( decisionVar_t* var ){

    for( int i = 0; i < NVAR; ++i )
         var->list[i].x = float_rand( var->list[i].xl, var->list[i].xu );

}

void ga_pushMemberToPopulation( population_t* pplt, decisionVar_t const* var, double dist ){
    
    memcpy( &pplt->dvar, var, sizeof( decisionVar_t) );
    pplt->res = dist;

}


void ga_registerSolution( population_t* pplt, double res ){
    pplt->res = res;
}


void ga_select( couple_t* prts, population_t const* pplt ){
   
    for( int i = 0; i < NUM_PARENTS; ++i){
        for (int j = 0; j < 2; ++j ){
            int id = RouletteSelection ( pplt );
            prts[i].list[j]= pplt[id];
        }   
    }
}


void ga_cross( couple_t* childs, couple_t const* prts, decisionVar_t const* bestdvar ){
    
    assert( NUM_PARENTS == NUM_CHILDS);
    for( int i = 0; i < NUM_PARENTS; ++i){
        get_crosschilds ( &childs[i], &prts[i], bestdvar  );
        if( 0 < verbose )
            ga_pcross( &childs[i], &prts[i] );
    }
}


void ga_mutation( couple_t* childs , decisionVar_t const* var){
    
    for( int i = 0; i < NUM_CHILDS; ++i){
        for( int j = 0; j < 2; ++j ){
            mutation_population( &childs[i].list[j], var );
        }
    }
}


void ga_recombination( population_t* pplt, couple_t const* childs, population_t const* pbestglobal ){

    
    population_t pbest = { 
        .dvar = childs[0].list[0].dvar,
        .res = 0 } ;
    getBestFromPopulation( &pbest, pplt );

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


void ga_get_memberFromPopulation( decisionVar_t* var, population_t const* pplt )
{
    memcpy( var, &pplt->dvar, sizeof(decisionVar_t) );
}



void ga_printpoulation( population_t const* pplt){
    
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



void mutation_population( population_t* pplt, decisionVar_t const* var ){

    for(int i = 0; i < NVAR; ++i ){
        double m = ((double) rand() / (RAND_MAX));
        if( m < 0.3)
            pplt->dvar.list[i].x = float_rand( var->list[i].xl, var->list[i].xu );
    }
}

void getBestFromPopulation( population_t* pbest, population_t const* plist ){
    int best = 0;
    double bestres = 0; 
    for( int i = 0; i < NPOPULATION; ++i ){
        if( plist[i].res > bestres ){
            best = i;
            bestres = plist[i].res;
        }        
    }
    memcpy( pbest, &plist[best] , sizeof( population_t ) );
    printf(" Best solution id %d\r\n", best);
}

int RouletteSelection ( population_t const* pplt ){
    double sumres = 0;
    for( int i = 0; i < NPOPULATION; ++i){
        sumres += pplt[i].res;
    }
    
    double const rnd = float_rand( 0.0 , sumres );
    double tempsum = 0;
    for( int i = 0; i < NPOPULATION; ++i){
        tempsum += pplt[i].res;
        if( tempsum >= rnd ) {
            printf( "Sumres: %f ,Rand %f, Best: %d\n", sumres, rnd, i );
            return i;
        }
    }
    
    return NPOPULATION-1;
}

void get_crosschilds ( couple_t* childs, couple_t const* prts, decisionVar_t const* best  ){
    
    int k = (int)float_rand( 0.0 , NVAR-1 ); /*Punto de cruce*/
    double alpha = ((double) rand() / (RAND_MAX)) + 1;
    assert( 0 <= k && NVAR > k);
    
    population_t const* p1 = &prts->list[0];
    population_t const* p2 = &prts->list[1];

    for (int j = 0; j < 2 ; ++j){ 
        population_t* ch = &childs->list[j];
        for(int i = 0; i < NVAR; ++i ){
        
            if( i < k ){
                ch->dvar.list[i]  = prts->list[j].dvar.list[i];
            }
            else
            {
                double const crss = ( j == 0 ) ? 
                    ( alpha * p1->dvar.list[i].x + ( 1.0 - alpha) * p2->dvar.list[i].x ) : 
                    ( ( 1.0 - alpha) * p1->dvar.list[i].x + (alpha) * p2->dvar.list[i].x );
            #if 1
                const bool outrange = crss > best->list[i].xu || crss < best->list[i].xl;

                ch->dvar.list[i].x = outrange ? 
                    float_rand( best->list[i].xl, best->list[i].xu  ) : crss;
            #else
                if( crss > best->list[i].xu )
                    ch->dvar.list[i].x = best->list[i].xu;
                else if( crss < best->list[i].xl )
                    ch->dvar.list[i].x = best->list[i].xl;
                else
                    ch->dvar.list[i].x = crss;
            #endif
            }   
        }
    }

}

void print_member( population_t const* pplt){

        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", pplt->dvar.list[i].x );  
        } 
        printf("  [%f]  ", pplt->res );
        printf("\r\n"); 
}

void print_couple( couple_t const* prts ){
    for( int i = 0; i < 2; ++i ){
        printf("Couple %d\r\n", i +1); 
        print_member( &prts->list[i] );
    }
}

void ga_pcross( couple_t const* childs, couple_t const* prts ){

    for ( int j = 0; j < 2; ++j){
        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", prts->list[j].dvar.list[i].x );  
        } 
        printf("  ---->  "); 
        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", childs->list[j].dvar.list[i].x );  
        } 
        printf("\r\n"); 
    }
    
    printf("\r\n"); 

}