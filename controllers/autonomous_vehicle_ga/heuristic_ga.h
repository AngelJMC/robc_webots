#ifndef HEURISTICS_GA_H
#define HEURISTICS_GA_H

enum{
    NUM_PARENTS = 10,
    NUM_CHILDS = NUM_PARENTS
};

enum{
  NPOPULATION = 10,
};

typedef struct{
    union{
        struct{
            double kp;
            double ki;
            double a;
            double b;
            double brakelimit;
        }pt;
        double plist[5];
    }var;
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

void ga_generate( population_t* pplt, decisionVar_t* var );

void ga_pushMemberToPopulation( population_t* pplt, decisionVar_t* var, double dist );

void ga_registerSolution( population_t* pplt, double res );

void ga_select( couple_t* prts, population_t* pplt );

void ga_cross( couple_t* childs, couple_t* prts );

void ga_mutation( couple_t* childs , decisionVar_t* var);

void ga_recombination( population_t * pplt, couple_t* childs );

void ga_printpoulation( population_t* pplt);




void ga_get_memberFromPopulation( decisionVar_t* var, population_t* pplt );



#endif