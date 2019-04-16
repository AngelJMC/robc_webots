
 
#include <webots/robot.h>
#include <webots/vehicle/driver.h>
#include <math.h>
#include <stdio.h>
#include "robc_control.h"
#include <assert.h>


enum{
    verbose = 0,
};


void robc_startEngine( void ) {
    wbu_driver_init();
    wbu_driver_set_hazard_flashers(true);
    wbu_driver_set_dipped_beams(true);
    wbu_driver_set_antifog_lights(true);
    wbu_driver_set_wiper_mode(SLOW);

}


void init_speedParam(  struct speedcrl* spc, double a, double b, double brakelimit){
    assert( NULL != spc );
    spc->a = a;
    spc->b = b;
    spc->brakelimit = brakelimit;
    spc->oldValue = 0.0;
    spc->integral = 0.0;
    spc->reset = true;

}

void init_steeringParam(  struct pid_param* pid, double kp, double ki, double kd ){
    assert( NULL != pid );
    pid->oldValue = 0.0;
    pid->integral = 0.0;
    pid->kp = kp;
    pid->ki = ki;
    pid->kd = kd;
    pid->reset = true;
}

// set target speed
void set_speed( struct speedcrl* spc, double yellow_line_angle ) {
  
    assert( NULL != spc );
    // max speed
  
    double realSpeed = wbu_driver_get_current_speed();
    if (isnan(realSpeed))
        realSpeed = 0.0;
    double const road_angle = fabs(yellow_line_angle);
    double targetSpeed = spc->a*powf(road_angle, spc->b);
    double ref = targetSpeed;

    if (spc->reset) {
        spc->oldValue = ref;
        spc->integral = 0.0;
        spc->reset = false;
    }

    // anti-windup mechanism
    if (signbit(ref) != signbit(spc->oldValue))
        spc->integral = 0.0;

    // limit integral
    if ( spc->integral < 30 && spc->integral > -30)
        spc->integral += ref;


    spc->oldValue = ref;
    double kmh = 0.0;
    double brakecoeff = 0.0;
    double difspeed = targetSpeed - realSpeed;
    if ( difspeed >= -5.0 ){
        kmh =  0.8 * ref + 0.1 * spc->integral;
        kmh = kmh < 0.0 ? 0.0 : kmh;
        kmh = kmh > 250.0 ? 250.0 : kmh;
    }
    else{
        brakecoeff = fabs(difspeed) > spc->brakelimit ? 1.0:fabs(difspeed / spc->brakelimit );
        kmh =  ( 1.0 - brakecoeff ) * ref ;
    }

    wbu_driver_set_cruising_speed(kmh);

    if ( verbose )
        printf( " Road angel: %f target Speed: %f, kmh: %f , real speed: %f, brakecoeff: %f\r\n", road_angle, targetSpeed, kmh, realSpeed, brakecoeff);

}


// positive: turn right, negative: turn left
void set_steering_angle( struct pid_param* pid, double angle ) {

    assert( NULL != pid );
    static double steering_angle = 0.0;

    if (pid->reset) {
        pid->oldValue = angle;
        pid->integral = 0.0;
        pid->reset = false;
    }

    // anti-windup mechanism
    if (signbit(angle) != signbit(pid->oldValue))
        pid->integral = 0.0;

    // limit integral
    if ( pid->integral < 30 && pid->integral > -30)
        pid->integral += angle;

    double const diff = angle - pid->oldValue;    
    pid->oldValue = angle;
    double wheel_angle = pid->kp * angle + pid->ki * pid->integral + pid->kd * diff;

    // limit the difference with previous steering_angle
    if (wheel_angle - steering_angle > 0.1)   wheel_angle = steering_angle + 0.1;
    if (wheel_angle - steering_angle < -0.1)  wheel_angle = steering_angle - 0.1;
    steering_angle = wheel_angle;
    // limit range of the steering angle
    if (wheel_angle > 0.5) wheel_angle = 0.5;
    else if (wheel_angle < -0.5) wheel_angle = -0.5;
    
    wbu_driver_set_steering_angle(wheel_angle);
}



