


struct pidparam{
  double kp;
  double ki;
  double kd;
};

typedef struct var{
  struct pidparam pidSteering;
  struct pidparam pidSpeed;
  double kspeed;
}decisionVar_t;

typedef struct {
  double speed;
  struct {
    double x;
    double y;
    double z;
  } accel;
  double encoder;
} statusVar_t;

int heuristics_loadParam( decisionVar_t* var );

void heuristics_update();

void heutistics_evaluate_restrictions( statusVar_t* st );