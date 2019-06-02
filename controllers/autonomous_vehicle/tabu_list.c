#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "tabu_list.h"
#include "math.h"


void _print_tabu( struct tabu* tbu );


void tabulist_init( tabuhdlr_t* htbu ){
    htbu->front = -1;
    htbu->rear  = -1;
}

bool _isinrange( decisionVar_t* refvar, decisionVar_t* dvar ){

    double ref = 0;
    double val = 0;
    double incr = 0;
    printf("\n");
    for ( int i = 0; i < NVAR; ++i ){
        ref = refvar->list[i].x;
        val = dvar->list[i].x;
        incr = ref * TABU_THRESHOLD / 100.0;
        printf(" [%f >= %f >= %f] ? ", fabs( ref + incr ), val, fabs( ref - incr ) );
        if ( fabs( ref + incr ) < fabs( val )  
                ||  fabs( ref - incr ) > fabs( val ) ){
                    printf(" FAIL\n");
                    return false;
        }
        printf(" OK\n");
    }
    printf("\n");
    return true;
}

bool tabulist_isinlist( tabuhdlr_t* htbu, decisionVar_t* dvar ){

    bool inrange = true;
    if( htbu->front == -1){
        printf("\nTabu list is Empty!!!\n");
        inrange = false;
    }
    else{
        int i = htbu->front;
        printf("\nTabu list elements are : \n");
        if( htbu->front <= htbu->rear ){
	        while(i <= htbu->rear)
                inrange &= _isinrange( &htbu->list[i++].dvar , dvar );
        }
        else{
	        while(i <= TABU_LIST_SIZE - 1)
	            inrange &= _isinrange( &htbu->list[i++].dvar , dvar );
	        i = 0;
	        while(i <= htbu->rear)
	            inrange &= _isinrange( &htbu->list[i++].dvar , dvar );
        }
    }
    return inrange;
}


bool tabulist_insert( tabuhdlr_t* htbu , decisionVar_t* dvar, double dist )
{
    if( ( htbu->front == 0 && htbu->rear == TABU_LIST_SIZE - 1 ) 
            || ( htbu->front == htbu->rear+1 ) ){
        printf("\nTabu list is Full! Insertion not possible!!!\n");
        return false;
    }
    
    if( htbu->rear == TABU_LIST_SIZE-1 && htbu->front != 0 )
	    htbu->rear = -1;
    struct tabu const t = {
            .dvar = *dvar,
            .dist = dist,
    };
    htbu->list[ ++htbu->rear ] = t;
    printf("\nInsertion Success!!!\n");
    if( htbu->front == -1 )     
        htbu->front = 0;
   
   return true;
}

void tabulist_remove( tabuhdlr_t* htbu )
{
    if( htbu->front == -1 && htbu->rear == -1 )
        printf("\nTabu list is Empty! Deletion is not possible!!!\n");
    else{
        printf("\nDeleted element : %f\n",htbu->list[ htbu->front++ ].dist );
        if( htbu->front == TABU_LIST_SIZE )
	        htbu->front = 0;
        if( htbu->front-1 == htbu->rear)
	        htbu->front = htbu->rear = -1;
    }
}

void tabulist_print( tabuhdlr_t* htbu ){

    if( htbu->front == -1)
        printf("\nTabu list is Empty!!!\n");
    else{
        int i = htbu->front;
        printf("\nTabu list elements are : \n");
        if( htbu->front <= htbu->rear ){
	        while(i <= htbu->rear)
                _print_tabu( &htbu->list[i++] );
        }
        else{
	        while(i <= TABU_LIST_SIZE - 1)
	            _print_tabu( &htbu->list[i++] );
	        i = 0;
	        while(i <= htbu->rear)
	            _print_tabu( &htbu->list[i++] );
        }
    }
}

void _print_tabu( struct tabu* tbu ){
    
    decisionVar_t* dvar = &tbu->dvar;
    printf("Distance: %f --- Param Kp: %f, Ki: %f, a: %f, b: %f, Bp: %f\n",
        tbu->dist,
        dvar->var.kp.x,
        dvar->var.ki.x,
        dvar->var.a.x,
        dvar->var.b.x,
        dvar->var.brakelimit.x );

}
