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
 
#include <webots/robot.h>

#include <webots/camera.h>
#include <webots/device.h>
#include <webots/display.h>
#include <webots/gps.h>
#include <webots/accelerometer.h>
#include <webots/keyboard.h>
#include <webots/vehicle/driver.h>
#include <webots/vehicle/car.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>
#include <assert.h>
#include <stdlib.h>

#include "heuristics_algorithm.h"
#include "robc_lane_detection.h"
#include "robc_control.h"


// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 30
#define UNKNOWN 99999.99



// camera
typedef struct {
    bool                isEnabled;
    WbDeviceTag         tag;
    struct camera_param param;
}camera_t;

// speedometer
typedef struct {
    bool        isEnabled;
    int         width ;
    int         height;
    WbDeviceTag tag;
    WbImageRef  speedometer_image;
}display_t;

// GPS
typedef struct {
    bool        isEnabled;
    double      gps_speed;
    double      coords[3];
    WbDeviceTag tag;
}gps_t;

// acceleromenter
typedef struct {
    bool        isEnabled;
    WbDeviceTag tag;
}accel_t;


// enabe various 'features'
static struct     pid_param pidSteering;
static struct     speedcrl speedcrl;
static camera_t   camera;
static display_t  display;
static gps_t      gps;
static accel_t    accel;



void init_camera( camera_t* cam ){
    assert( NULL != cam );
    cam->tag = wb_robot_get_device("camera");
    wb_camera_enable(cam->tag , TIME_STEP);
    cam->param.camera_width = wb_camera_get_width(cam->tag );
    cam->param.camera_height = wb_camera_get_height(cam->tag );
    cam->param.camera_fov = wb_camera_get_fov(cam->tag );
}


void init_display( display_t* disp ){
    assert( NULL != disp );
    disp->tag = wb_robot_get_device( "display" );
    disp->speedometer_image = wb_display_image_load( disp->tag, "speedometer.png" );
    disp->width = 0;
    disp->height = 0 ;
}


void init_gps( gps_t* gps ){
    assert( NULL != gps);
    gps->tag = wb_robot_get_device("gps");
    wb_gps_enable(gps->tag , TIME_STEP);
    gps->gps_speed = 0;
}

void init_accelerometer( accel_t* accel ){
    assert( NULL != accel );
    accel->tag = wb_robot_get_device("accelerometer");
    wb_accelerometer_enable( accel->tag, TIME_STEP );
}

void update_display( display_t* disp, double* gps_coords, double gps_speed ) {
  
    const double NEEDLE_LENGTH = 60.0;

    // display background
    wb_display_image_paste(disp->tag, disp->speedometer_image, 0, 0, false);

    // draw speedometer needle
    double speed = wbu_driver_get_current_speed();
    if (isnan(speed))
        speed = 0.0;
    double alpha = speed / 260.0 * 3.72 - 0.27;
    int x = -NEEDLE_LENGTH * cos(alpha);
    int y = -NEEDLE_LENGTH * sin(alpha);
    wb_display_draw_line( disp->tag, 100, 95, 100 + x, 95 + y);

    // draw text
    char txt[64];
    sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
    wb_display_draw_text( disp->tag, txt, 10, 130);
    sprintf(txt, "GPS speed:  %.1f", gps_speed);
    wb_display_draw_text( disp->tag, txt, 10, 140);
}


void compute_gps_speed( gps_t* gps ) {
    const double *coords = wb_gps_get_values( gps->tag );
    const double speed = wb_gps_get_speed( gps->tag );
    // store into global variables
    gps->gps_speed = speed * 3.6;  // convert from m/s to km/h
    memcpy(gps->coords, coords, sizeof(gps->coords));
}

void status_init( statusVar_t* st ){
    
    enum {
        WBU_CAR_WHEEL_FRONT_RIGHT,
        WBU_CAR_WHEEL_FRONT_LEFT,
        WBU_CAR_WHEEL_REAR_RIGHT,
        WBU_CAR_WHEEL_REAR_LEFT,
        WBU_CAR_WHEEL_NB
    };
    
    st->angle = 0;
    st->accel.y = 0;  
    st->accel.z = 0;            
    st->accel.x = 0;
    double rad = ( wbu_car_get_wheel_encoder( WBU_CAR_WHEEL_REAR_RIGHT ) 
                    + wbu_car_get_wheel_encoder( WBU_CAR_WHEEL_REAR_LEFT ) ) / 2;
    st->offsetRad = isnan(rad) ? 0.0 : rad;
    st->dist = 0;
}

void status_update( statusVar_t* st , double angle){
    
    enum {
        WBU_CAR_WHEEL_FRONT_RIGHT,
        WBU_CAR_WHEEL_FRONT_LEFT,
        WBU_CAR_WHEEL_REAR_RIGHT,
        WBU_CAR_WHEEL_REAR_LEFT,
        WBU_CAR_WHEEL_NB
    };
    
    double wheelradious = wbu_car_get_rear_wheel_radius( );
    double rad = ( wbu_car_get_wheel_encoder( WBU_CAR_WHEEL_REAR_RIGHT ) 
                    + wbu_car_get_wheel_encoder( WBU_CAR_WHEEL_REAR_LEFT ) ) / 2;
    st->dist = fabs( rad - st->offsetRad ) * wheelradious;
    st->angle = angle;
#if 0
    st->speed = ( wbu_car_get_wheel_speed( WBU_CAR_WHEEL_REAR_RIGHT ) 
                    + wbu_car_get_wheel_speed( WBU_CAR_WHEEL_REAR_LEFT ) ) / 2;
#endif

    st->speed = wbu_driver_get_current_speed();

    const double* a = wb_accelerometer_get_values( accel.tag );
    st->accel.y = a[0];  
    st->accel.z = a[1];            
    st->accel.x = a[2]; 
}

double get_travelled_distance( statusVar_t* st  ){
    return st->dist;
}


void run_simulation( decisionVar_t* decvar , statusVar_t* stvar )
{
        stvar->numfail = 0;
        init_speedParam(  &speedcrl, decvar->var.a.x, decvar->var.b.x, decvar->var.brakelimit.x  );
        init_steeringParam(  &pidSteering, decvar->var.kp.x, decvar->var.ki.x, 0.0 );
        wbu_car_init();
        status_init( stvar );
        /* Execute a new simulation */
        for (double t = 0.0; t <= 90.0; t += TIME_STEP / 1000.0) {
          
            wbu_driver_step( );   // updates sensors only every TIME_STEP milliseconds
            double yellow_line_angle = UNKNOWN;
            if ( camera.isEnabled ) {
                const unsigned char *camera_image = wb_camera_get_image( camera.tag );
                yellow_line_angle = robc_getAngleFromCamera( camera_image, &camera.param );
                if (yellow_line_angle != UNKNOWN) {
                    // no obstacle has been detected, simply follow the line
                    set_speed( &speedcrl, yellow_line_angle );
                    set_steering_angle( &pidSteering, yellow_line_angle );
                    
                }else {
                    // no obstacle has been detected but we lost the line => we brake and hope to find the line again
                    wbu_driver_set_brake_intensity(0.4);
                    pidSteering.reset = true;
                }
            }
    
            // update stuff
            if ( gps.isEnabled )
                compute_gps_speed( &gps );
            if ( display.isEnabled )
                update_display( &display, gps.coords, gps.gps_speed );

            status_update( stvar, yellow_line_angle );
            bool const lastCycle = t >= 89.93;
            bool res = heutistics_evaluate_restrictions( stvar, lastCycle );
            if ( !res ) { 
                break; }
        }


}

int main(int argc, char **argv) {
    
    decisionVar_t decvar;
    decisionVar_t bestvar;
    neighbor_t nbh[NVAR];

    wb_robot_init();
    
    WbNodeRef robot_node = wb_supervisor_node_get_self();
    WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
    const double *initial_trans = wb_supervisor_field_get_sf_vec3f( trans_field );
    WbFieldRef rotation_field = wb_supervisor_node_get_field(robot_node, "rotation");
    const double *initial_rotation = wb_supervisor_field_get_sf_rotation( rotation_field );

    WbNodeRef vw_node = wb_supervisor_node_get_from_def("viewp");
    WbFieldRef vw_trans_field = wb_supervisor_node_get_field(vw_node, "position");
    const double *trans = wb_supervisor_field_get_sf_vec3f(vw_trans_field);
    printf("MY_ROBOT is at position: %g %g %g\n", trans[0], trans[1], trans[2]);

    // check devices
    for (int j = 0; j < wb_robot_get_number_of_devices(); ++j) {
        WbDeviceTag device = wb_robot_get_device_by_index(j);
        const char *name = wb_device_get_name( device );
        if (strcmp(name, "display") == 0){
            display.isEnabled = true;
            init_display( &display );
        }
        else if (strcmp(name, "gps") == 0){
            gps.isEnabled = true;
            init_gps( &gps );
        }
        else if (strcmp(name, "camera") == 0){
            camera.isEnabled = true;
            init_camera( &camera ); 
        }
        else if (strcmp(name, "accelerometer") == 0){
            accel.isEnabled = true;
            init_accelerometer( &accel ); 
        }
    }
    
    heuristics_init( &decvar );
    memcpy( &bestvar, &decvar, sizeof( decisionVar_t ) );
    
    static int iter = 0;
    static double bestrsl = 0;
    static double dist = 0;
    // start engine
    robc_startEngine( );
    /* Run simulation */
    while(true){
        /* Load the value of decision variables from heuristic algorithm */
        //int nsim = heuristics_loadParam( &decvar );
        printf("Num simulation:  \n");
        heuristics_generate_neighbor( nbh, &bestvar );
        
        int bestiter = -1;
        for( iter = 0; iter < POINTS_NBH; ++iter ){
            statusVar_t stvar;
            heuristics_get_neighbor( &decvar, &nbh[iter] );
            heutistics_print_point( &decvar );
            
            run_simulation( &decvar, &stvar );
            dist = get_travelled_distance( &stvar );
            if ( dist > bestrsl ){
                bestiter = iter;
                bestrsl = dist;
            }
            printf("Iter: %d, distance: %f, best distance: %f\r\n", iter, dist, bestrsl ); 
            
            /*Return to initial position*/
            wb_supervisor_field_set_sf_vec3f( trans_field, initial_trans );
            wb_supervisor_field_set_sf_rotation( rotation_field, initial_rotation );
            wb_supervisor_field_set_sf_vec3f( vw_trans_field, trans );
            wb_supervisor_node_reset_physics( robot_node );
            
            //wbu_car_cleanup();

        }

        if( bestiter != -1 ){
            printf("Best: [%d], distance: %f\r\n", bestiter, bestrsl); 
            heuristics_get_neighbor( &decvar, &nbh[bestiter] );
            memcpy( &bestvar, &decvar, sizeof( decisionVar_t ) );
        }
        
        


    }
    
    //wb_supervisor_simulation_reset(); 



    wbu_driver_cleanup(); 
    //wbu_car_cleanup();

    return 0;  // ignored
}
