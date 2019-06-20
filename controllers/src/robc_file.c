

#include <string.h>
#include <time.h>
#include "robc_heuristics.h"
#include "robc_file.h" 

enum{
    verbose = 1,
};

void robc_init( fl_t* fl, char const* msg){
    
    
    time_t now = time(0);
    char tmp[100];

    sprintf( tmp, "%s%s-%ld%s",
        "./../../results/",
        msg,
        now,
        ".csv");

    strcpy( fl->filename, tmp );
    
    FILE *fp;
    fp = fopen( fl->filename, "w" );
    if ( fp != NULL ){
        printf("Create file to save results: %s\n", fl->filename);
        fprintf( fp, "timestamp, Kp, Ki, Sa, Sb, Bp, distance, bestdist, simres \r\n");
        fclose(fp);
    }else{
        printf( " Error open file %s \n", fl->filename );
    }
}

void robc_fl( fl_t* fl ){
    FILE* fp;
    decisionVar_t const* dvar = fl->out.dvar;
    struct results const* out = &fl->out;

    fp = fopen( fl->filename, "a" );
    fseek(fp, 0, SEEK_END);
    if( fp != NULL ){
        fprintf( fp, "%ld, " , time(0) );       
        for(int i = 0; i < NVAR; ++i ){
            fprintf( fp, "%f, " , dvar->list[i].x );
        }
        fprintf( fp, "%f, %f, %d\r\n" , out->res, out->bres, out->simres );
        fclose(fp);
    }


}



