
 
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
    spc->b = b * ( -1.0 );
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
    pid->ki = ki / 10000;
    pid->kd = kd;
    pid->reset = true;
}

// set target speed
void set_speed( struct speedcrl* spc, double yellow_line_angle ) {
  
    assert( NULL != spc );

    double realSpeed = wbu_driver_get_current_speed();
    realSpeed = isnan(realSpeed) ? 0.0 : realSpeed;

    double const road_angle = fabs(yellow_line_angle);
    double const targetSpeed = fmin( 200.0, spc->a * powf( road_angle, spc->b ) );

    if (spc->reset) {
        spc->oldValue = targetSpeed;
        spc->integral = 0.0;
        spc->reset = false;
    }

    // anti-windup mechanism
    if (signbit(targetSpeed) != signbit(spc->oldValue))
        spc->integral = 0.0;
    spc->oldValue = targetSpeed;

    // limit integral
    if ( spc->integral < 30 && spc->integral > -30)
        spc->integral += targetSpeed;

    double kmh = 0.0;
    double brakecoeff = 0.0;
    double const errSpeed = targetSpeed - realSpeed;
    
    const double brakehys = -1 * spc->brakelimit/4;
    if ( errSpeed > brakehys ){
        kmh =  0.8 * targetSpeed + 0.1 * spc->integral;   
    }
    else{
        brakecoeff = (fabs(errSpeed) > spc->brakelimit) ? 1.0 : fabs( (errSpeed - brakehys) / spc->brakelimit );
        kmh =  ( 1.0 - brakecoeff ) * targetSpeed ;
    }
    
    kmh = fmin( 250.0, fmax( 0.0, kmh ));
    wbu_driver_set_cruising_speed(kmh);

    if ( verbose )
        printf( " Road angel: %f target Speed: %f, kmh: %f , real speed: %f, brakecoeff: %f\r\n", road_angle, targetSpeed, kmh, realSpeed, brakecoeff);

}


// positive: turn right, negative: turn left
void set_steering_angle( struct pid_param* pid, double angleErr ) {

    assert( NULL != pid );
    static double steering_angle = 0.0;

    if (pid->reset) {
        pid->oldValue = angleErr;
        pid->integral = 0.0;
        pid->reset = false;
    }

    // anti-windup mechanism
    if (signbit(angleErr) != signbit(pid->oldValue))
        pid->integral = 0.0;

    // limit integral
    if ( pid->integral < 30 && pid->integral > -30)
        pid->integral += angleErr;

    double const diff = angleErr - pid->oldValue;    
    pid->oldValue = angleErr;
    double wheel_angle = pid->kp * angleErr + pid->ki * pid->integral + pid->kd * diff;

    // limit the difference with previous steering_angle
    if (wheel_angle - steering_angle > 0.1)   wheel_angle = steering_angle + 0.1;
    if (wheel_angle - steering_angle < -0.1)  wheel_angle = steering_angle - 0.1;
    steering_angle = wheel_angle;
    // limit range of the steering angle
    if (wheel_angle > 0.5) wheel_angle = 0.5;
    else if (wheel_angle < -0.5) wheel_angle = -0.5;
    
    wbu_driver_set_steering_angle(wheel_angle);
}



