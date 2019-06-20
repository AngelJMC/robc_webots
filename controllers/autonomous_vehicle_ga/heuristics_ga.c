
#include <stdio.h>
#include <stdbool.h>
#include <stdint.h>
#include <assert.h>
#include <stdlib.h>
#include <string.h>
#include <time.h>
#include <math.h>
#include "heuristic_ga.h"

enum{
    verbose = 1
};

void mutation_population( population_t* pplt, decisionVar_t const* var, int ngen, int type );
int rouletteSelection ( population_t const* pplt );
void print_member( char const* msg, population_t const* pplt);
void getBestFromPopulation( population_t* pbest, population_t const* plist );
void get_crosschilds ( couple_t* childs, couple_t const* prts );
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


void ga_select( couple_t prts[], population_t const pplt[] ){
   
    for( int i = 0; i < NUM_PARENTS; ++i){
        for (int j = 0; j < 2; ++j ){
            int id = rouletteSelection ( pplt );
            prts[i].list[j]= pplt[id];
        }   
    }
}


void ga_cross( couple_t childs[], couple_t const* prts ){
    
    assert( NUM_PARENTS == NUM_CHILDS);
    for( int i = 0; i < NUM_PARENTS; ++i){
        get_crosschilds ( &childs[i], &prts[i] );
        if( 0 < verbose )
            ga_pcross( &childs[i], &prts[i] );
    }
}


void ga_mutation( couple_t childs[] , decisionVar_t const* var, int ngen, int type){
    
    printf(" Num generation: %d\r\n", ngen);
    for( int i = 0; i < NUM_CHILDS; ++i){
        for( int j = 0; j < 2; ++j ){
            mutation_population( &childs[i].list[j], var, ngen, type );
        }
    }
}


void ga_recombination( population_t pplt[], couple_t const childs[], population_t const* pbestglobal ){

    
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
    print_member( "\n", &pbest ); 
    pplt[p] = pbest;
    
    #if 1
    p = (int)float_rand( 0.0 , NPOPULATION-1 );
    printf(" Best solution of all generation: %d\r\n", p);
    print_member( "\n", pbestglobal ); 
    memcpy( &pplt[p], pbestglobal, sizeof(population_t ));
    #endif
        
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
        print_member( "\n", &pplt[n] );
    }

}


double disruption( double ngen, double y){

    enum{
        b = 5, //No uniform factor
    };
    double const s = ((double) rand() / (RAND_MAX));
    int const gen = fmin( ngen, MAX_GEN );
    double const r = y * s * pow( 1.0 - (double)gen / MAX_GEN , b ) ;
    return r;
}




void mutation_population( population_t* pplt, decisionVar_t const* var, int ngen, int type ){

    double ratio =  1.0 - ( (double) ngen - 1.0 ) / ( MAX_GEN - 1.0);
    ratio = fmin( fmax( ratio , 0.20), 1.0 );

    for(int i = 0; i < NVAR; ++i ){
        double const m = ((double) rand() / (RAND_MAX));
        
        if( m >= ratio )
            continue;

        if( type == MUT_UNIFORM ){    
            pplt->dvar.list[i].x = float_rand( var->list[i].xl, var->list[i].xu );
        }
        else if( type == MUT_NOT_UNIFORM ){
            double const r = ((double) rand() / (RAND_MAX));
            double diff = 0.0;
            if ( r < 0.5 ){
                diff = var->list[i].xu - pplt->dvar.list[i].x;
                pplt->dvar.list[i].x = pplt->dvar.list[i].x + disruption( ngen, diff);
            }
            else{
                diff = pplt->dvar.list[i].x - var->list[i].xl;
                pplt->dvar.list[i].x = pplt->dvar.list[i].x - disruption( ngen, diff);
            }
        }
    }
}

void getBestFromPopulation( population_t* pbest, population_t const plist[] ){
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

int rouletteSelection ( population_t const pplt[] ){
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
    
    return NPOPULATION - 1;
}

void get_crosschilds ( couple_t* childs, couple_t const* prts ){
    
    double a = ((double) rand() / (RAND_MAX));
    population_t const* v = &prts->list[0];
    population_t const* w = &prts->list[1];

    for (int j = 0; j < 2 ; ++j){ 
        population_t* ch = &childs->list[j];
        for(int i = 0; i < NVAR; ++i ){
            ch->dvar.list[i].x = ( j == 0 ) ? 
                ( a * v->dvar.list[i].x + ( 1.0 - a) * w->dvar.list[i].x ) : 
                ( ( 1.0 - a) * v->dvar.list[i].x + (a) * w->dvar.list[i].x );  
        }
    }
}

void print_member( char const* msg, population_t const* pplt){

        for( int i = 0; i < NVAR; ++i ) {
            printf("  [%f]  ", pplt->dvar.list[i].x );  
        } 
        printf("  [%f]  ", pplt->res );
        printf("%s", msg); 
}

void ga_pcouple( couple_t const* prts ){
    for( int i = 0; i < 2; ++i ){
        printf("Couple %d : ", i +1); 
        print_member( "   ", &prts->list[i] );
    }
    printf("\n");
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



