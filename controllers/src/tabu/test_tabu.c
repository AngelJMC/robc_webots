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

#include "../heuristics_algorithm.h"
#include "../tabu_list.h"







int main(int argc, char **argv) {
    
    

    static tabuhdlr_t htbu;

    tabulist_init( &htbu );

    decisionVar_t dvar;
    for ( int j = 0; j< 6; ++j){
        
        for ( int i = 0; i< NVAR; ++i)
            dvar.list[i].x = (i + 1)*2;
        double dist = rand() % 5;

        tabulist_insert( &htbu , &dvar, dist );
    }
    
    tabulist_print( &htbu );
    for( int j = 0; j<2; ++j)
        tabulist_remove( &htbu );

    tabulist_print( &htbu );

    for ( int i = 0; i< NVAR; ++i){
        dvar.list[i].x = (i + 1)*2; 
    }
        
    int res = tabulist_isinlist( &htbu, &dvar );
    if( res ) printf(" Decision var is in tabu list\n");


    for ( int i = 0; i< NVAR; ++i){
        dvar.list[i].x = (i + 1); 
    }
    res = tabulist_isinlist( &htbu, &dvar );
    if( res ) printf(" Decision var is in tabu list\n");


    for ( int i = 0; i< NVAR; ++i){
        dvar.list[i].x = (i + 1)*2; 
    }
    dvar.list[4].x = 1;
    res = tabulist_isinlist( &htbu, &dvar );
    if( res ) printf(" Decision var is in tabu list\n");



    return 0;  // ignored
}
