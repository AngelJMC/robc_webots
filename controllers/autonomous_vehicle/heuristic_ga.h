



typedef struct{
    union var{
        struct{
            double kp;
            double ki;
            double a;
            double b;
            double brakelimit;
        }pt;
        double plist[5];
    };
    double res;
}population_t;


typedef union {
        struct{
            population_t p1;
            population_t p2;
        }pt;
        population_t list[2];
}parents_t;