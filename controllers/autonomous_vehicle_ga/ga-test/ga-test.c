/*
 * Copyright 1996-2018 Cyberbotics Ltd.
 *
 * Licensed under the Apache License, Version 2.0 (the "License");
 * you may not use this file except in compliance with the License.
 * You may obtain a copy of the License at
 *
 *     http://www.apache.org/licenses/LICENSE-2.0
 *
 * Unless required by applicable law or agreed to in writing, software
 * distributed under the License is distributed on an "AS IS" BASIS,
 * WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
 * See the License for the specific language governing permissions and
 * limitations under the License.
 */

/*
 * Description:   Autonoumous vehicle controller example
 */
 

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>
#include <float.h>

#include "../heuristic_ga.h"
#include "../../src/robc_heuristics.h"



enum{
    PAIR_CHILDS = 1,
    NPOPU = 2*PAIR_CHILDS,
};

int main(int argc, char **argv) {
    
    decisionVar_t decvar;
    decisionVar_t defaultdvar;
    couple_t childs[PAIR_CHILDS];
    population_t popu[NPOPU];

    ga_init( &defaultdvar );

    for( int i = 0; i < PAIR_CHILDS; ++i ){
        for( int p = 0; p < 2; ++p ){
            ga_init( &decvar );
            memcpy( &childs[i].list[p].dvar, &decvar, sizeof( decisionVar_t) );
        }
    }

    for (int gen = 0 ; gen < 150; ++gen ){
        ga_mutation( childs , &defaultdvar, gen, MUT_UNIFORM );
        couple_t const* cp = &childs[0];
        ga_pcouple( cp );
    }


    for (int gen = 0 ; gen < 150; ++gen ){
        ga_mutation( childs , &defaultdvar, gen, MUT_NOT_UNIFORM );
        couple_t const* cp = &childs[0];
        ga_pcouple( cp );
    }


    return 0;  // ignored
}
