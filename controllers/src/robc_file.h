#ifndef ROBC_FILE_H
#define ROBC_FILE_H

#include <stdio.h>
#include "robc_heuristics.h"


struct results{
    decisionVar_t* dvar;
    double res;
    double bres;
    bool simres;
};

typedef struct {
    char filename[64];
    struct results out;

}fl_t;

void robc_init( fl_t* fl, char const* msg);

void robc_fl( fl_t* fl );

#endif /*ROBC_FILE_H */
