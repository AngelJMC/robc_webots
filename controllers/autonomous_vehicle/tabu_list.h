
#ifndef TABU_LIST_H
#define TABU_LIST_H

#include <stdio.h>
#include <string.h>
#include <stdlib.h>
#include <stdbool.h>
#include "heuristics_algorithm.h"


enum{
    TABU_LIST_SIZE = 15,
    TABU_THRESHOLD = 25,
};


struct tabu{
  decisionVar_t dvar;
  double        dist;
};


typedef struct{
    struct tabu list[TABU_LIST_SIZE];
    int front;
    int rear;
    int itemCount;
}tabuhdlr_t;

void tabulist_init( tabuhdlr_t* htbu );

bool tabulist_insert( tabuhdlr_t* htbu , decisionVar_t* dvar, double dist );

void tabulist_remove( tabuhdlr_t* htbu );

void tabulist_print( tabuhdlr_t* htbu );

bool tabulist_isinlist( tabuhdlr_t* htbu, decisionVar_t* dvar );


#endif /*TABU_LIST_H*/