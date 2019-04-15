
 
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <math.h>
#include <stdio.h>
#include "robc_control.h"
#include <assert.h>


static double applyPID(struct pid_param* pid, double ref );


void robc_startEngine( void ) {
    wbu_driver_init();
    wbu_driver_set_hazard_flashers(true);
    wbu_driver_set_dipped_beams(true);
    wbu_driver_set_antifog_lights(true);
    wbu_driver_set_wiper_mode(SLOW);

}


// set target speed
void set_speed( struct pid_param* pid, double yellow_line_angle ) {
  
  assert( NULL != pid );
  // max speed
  
 double targetSpeed = 150;
 double realSpeed = wbu_driver_get_current_speed();
 double const road_angle = fabs(yellow_line_angle);
  
  double a = 23.45;
  double b = -0.2299;
  double const brakelimit = 20.0;


  targetSpeed = a*powf(road_angle, b);

  double ref = targetSpeed;
  if (pid->reset) {
        pid->oldValue = ref;
        pid->integral = 0.0;
        pid->reset = false;
  }

    // anti-windup mechanism
  if (signbit(ref) != signbit(pid->oldValue))
        pid->integral = 0.0;

    // limit integral
  if ( pid->integral < 30 && pid->integral > -30)
        pid->integral += ref;


  pid->oldValue = ref;
  double kmh = 0;
  double difspeed = targetSpeed - realSpeed;
  if ( difspeed >= -5.0 ){
    kmh =  0.8 * ref + 0.1 * pid->integral;
    kmh = kmh < 0.0 ? 0.0 : kmh;
    kmh = kmh > 250.0 ? 250.0 : kmh;
  }
  else{
    double brakecoeff = fabs(difspeed) > brakelimit ? 1.0:fabs(difspeed / brakelimit );
    printf( " Break coef: %f \r\n", brakecoeff);
    kmh =  ( 1.0 - brakecoeff ) * ref ;
  }

  
  printf( " Road angel: %f target Speed: %f, kmh: %f , real speed: %f\r\n", road_angle, targetSpeed, kmh, realSpeed);
  wbu_driver_set_cruising_speed(kmh);
  //printf("setting speed to %g km/h\n", kmh);
  #if 0
  if( (speed - kmh) >15.0){
    wbu_driver_set_brake_intensity(1.0);
    wbu_driver_set_cruising_speed( 0.0);
    printf( " Break!! %f\r\n", wbu_driver_get_brake_intensity( ) );
  }
  else{
  wbu_driver_set_cruising_speed(kmh);
  }
  #endif
  
}


// positive: turn right, negative: turn left
void set_steering_angle( struct pid_param* pid, double yellow_line_angle ) {

    assert( NULL != pid );
    static double steering_angle = 0.0;
    double wheel_angle = applyPID( pid, yellow_line_angle );

    // limit the difference with previous steering_angle
    if (wheel_angle - steering_angle > 0.1)   wheel_angle = steering_angle + 0.1;
    if (wheel_angle - steering_angle < -0.1)  wheel_angle = steering_angle - 0.1;
    steering_angle = wheel_angle;
    // limit range of the steering angle
    if (wheel_angle > 0.5) wheel_angle = 0.5;
    else if (wheel_angle < -0.5) wheel_angle = -0.5;
    
    wbu_driver_set_steering_angle(wheel_angle);
}


static double applyPID(struct pid_param* pid, double ref ) {

    assert( NULL != pid );

    if (pid->reset) {
        pid->oldValue = ref;
        pid->integral = 0.0;
        pid->reset = false;
    }

    // anti-windup mechanism
    if (signbit(ref) != signbit(pid->oldValue))
        pid->integral = 0.0;

    // limit integral
    if ( pid->integral < 30 && pid->integral > -30)
        pid->integral += ref;

    double const diff = ref - pid->oldValue;    

    pid->oldValue = ref;
    return pid->kp * ref + pid->ki * pid->integral + pid->kd * diff;
}


