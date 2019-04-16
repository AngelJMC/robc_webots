#ifndef ROBC_CONTROL_H
#define ROBC_CONTROL_H

#include <stdbool.h>
#include <stdint.h>

struct pid_param{
  double kp;
  double ki;
  double kd;
  double integral;
  double oldValue;
  bool reset;
};



struct speedcrl{

  double a;
  double b;
  double brakelimit;
  double integral;
  double oldValue;
  bool reset;

};

void robc_startEngine( void );
void set_speed( struct speedcrl* speedcrl, double yellow_line_angle );
void set_steering_angle( struct pid_param* pid, double yellow_line_angle );
void init_speedParam(  struct speedcrl* spc, double a, double b, double brakelimit);
void init_steeringParam(  struct pid_param* pid, double kp, double ki, double kd );

#endif /*ROBC_CONTROL_H*/