#ifndef HEURISTICS_GA_H
#define HEURISTICS_GA_H

enum{
  NPOPULATION = 10,
};

enum{
    NUM_PARENTS = NPOPULATION/2,
    NUM_CHILDS = NUM_PARENTS
};


typedef struct{
    decisionVar_t dvar;
    double res;
}population_t;


typedef union {
        struct{
            population_t p1;
            population_t p2;
        }pt;
        population_t list[2];
}couple_t;

void ga_init( decisionVar_t* var );

void ga_generaterandMember( decisionVar_t* var );

void ga_pushMemberToPopulation( population_t* pplt, decisionVar_t const* var, double dist );

void ga_registerSolution( population_t* pplt, double res );

void ga_select( couple_t* prts, population_t const* pplt );

void ga_cross( couple_t* childs, couple_t const* prts, decisionVar_t const* bestdvar );

void ga_mutation( couple_t* childs , decisionVar_t const* var);

void ga_recombination( population_t* pplt, couple_t const* childs, population_t const* pbestglobal );

void ga_printpoulation( population_t const* pplt);


void ga_get_memberFromPopulation( decisionVar_t* var, population_t const* pplt );



#endif