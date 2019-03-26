


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

int heuristics_loadParam( decisionVar_t* var );

void heuristics_update();

void heutistics_checkRestrictions();