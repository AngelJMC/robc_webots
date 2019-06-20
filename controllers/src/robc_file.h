#ifndef ROBC_FILE_H
#define ROBC_FILE_H

#include <stdio.h>
#include "robc_heuristics.h"


typedef struct {

    char filename[64];

}fl_t;

void robc_init( fl_t* fl, char const* msg);

void robc_fl( fl_t* fl, decisionVar_t* dvar, double res, double bres);

#endif /*ROBC_FILE_H */
