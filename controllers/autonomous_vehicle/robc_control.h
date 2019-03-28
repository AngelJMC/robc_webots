

struct pid_param{
  double kp;
  double ki;
  double kd;
  double integral;
  double oldValue;
  bool reset;
};

void robc_startEngine( void );
void set_speed( struct pid_param* pid, double yellow_line_angle );
void set_steering_angle( struct pid_param* pid, double yellow_line_angle );