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
#include <float.h>

#include "../src/robc_heuristics.h"
#include "heuristic_ga.h"
#include "../src/robc_lane_detection.h"
#include "../src/robc_control.h"
#include "../src/robc_file.h"


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

struct wb_node{
    WbFieldRef      field;
    const double*   value;
};


struct nodeHdlr{
    WbNodeRef      carNode;
    struct wb_node  carTrans;
    struct wb_node  carRot;
    WbNodeRef       viewNode;
    struct wb_node  viewPos;
};

// enabe various 'features'
static struct     pid_param pidSteering;
static struct     speedcrl speedcrl;
static camera_t   camera;
static display_t  display;
static gps_t      gps;
static accel_t    accel;


bool run_simulation( struct nodeHdlr* nh, decisionVar_t* decvar , statusVar_t* stvar );
void car_restart_position( struct nodeHdlr* nh );

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
    st->speed = wbu_driver_get_current_speed();

    const double* a = wb_accelerometer_get_values( accel.tag );
    st->accel.y = a[0];  
    st->accel.z = a[1];            
    st->accel.x = a[2]; 
}



bool check_best_solution( struct nodeHdlr* nh, decisionVar_t* decvar , statusVar_t* stvar, double bestdist ){

    enum{
        NUM_ITER_TO_VALIDATE = 2,
    };
    printf("\n  ---- Validate new solution ---- \n" );
    printf("        Check %d  -> distante: %f\n",  0, bestdist );
    for ( int i = 0; i < NUM_ITER_TO_VALIDATE; ++i ){
        bool const res = run_simulation( nh, decvar , stvar );
        double const dist = heuristics_get_objetive( stvar );
        printf("        Check %d  -> distante: %f\n",  i +1, dist );
        if ( (bestdist - dist) > 40.0  || !res ) return false;
    }
    return true;
    
}


void car_restart_position( struct nodeHdlr* nh ){

    /*Return to initial position*/       
    wb_supervisor_field_set_sf_vec3f( nh->carTrans.field, nh->carTrans.value );
    wb_supervisor_field_set_sf_rotation( nh->carRot.field, nh->carRot.value );
    wb_supervisor_field_set_sf_vec3f( nh->viewPos.field, nh->viewPos.value );
    wb_supervisor_node_reset_physics( nh->carNode );
    wbu_car_init();
}

bool run_simulation( struct nodeHdlr* nh, decisionVar_t* decvar , statusVar_t* stvar )
{
        car_restart_position( nh );
        printf("  Setting param new cycle... ");
        stvar->numfail = 0;
        init_speedParam(  &speedcrl, decvar->var.a.x, decvar->var.b.x, decvar->var.brakelimit.x  );
        init_steeringParam(  &pidSteering, decvar->var.kp.x, decvar->var.ki.x, 0.0 );
        
        printf("start simulation... \n");
        status_init( stvar );
        robc_initLaneDetection( );
        /* Execute a new simulation */
        for (double t = 0.0; t <= 90.0; t += TIME_STEP / 1000.0) {
          
            wbu_driver_step( );   // updates sensors only every TIME_STEP milliseconds
            if ( camera.isEnabled ) {
                const unsigned char *camera_image = wb_camera_get_image( camera.tag );
                double const angle = robc_getAngleFromCamera( camera_image, &camera.param );
                set_speed( &speedcrl, angle );
                set_steering_angle( &pidSteering, angle );
                status_update( stvar, angle );
            }
    
            if ( gps.isEnabled )
                compute_gps_speed( &gps );
            if ( display.isEnabled )
                update_display( &display, gps.coords, gps.gps_speed );

            
            bool const lastCycle = t >= 89.93;
            int const stopSim = heuristics_evaluate_restrictions( stvar, lastCycle );
            if ( stopSim ) {
                return stopSim == SML_FINISH;
            }
                
        }
        printf("stop simulation error \n");
    return SML_FINISH;
}


void car_devices_init( struct nodeHdlr* nh )
{
    nh->carNode = wb_supervisor_node_get_self();
    nh->carTrans.field = wb_supervisor_node_get_field( nh->carNode, "translation" );
    nh->carTrans.value = wb_supervisor_field_get_sf_vec3f( nh->carTrans.field );
    nh->carRot.field = wb_supervisor_node_get_field( nh->carNode, "rotation" );
    nh->carRot.value = wb_supervisor_field_get_sf_rotation( nh->carRot.field );

    nh->viewNode = wb_supervisor_node_get_from_def( "viewp" );
    nh->viewPos.field = wb_supervisor_node_get_field( nh->viewNode, "position" );
    nh->viewPos.value = wb_supervisor_field_get_sf_vec3f( nh->viewPos.field );

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

}


int main(int argc, char **argv) {
    
    decisionVar_t decvar;
    decisionVar_t bestdvar;
    decisionVar_t defaultdvar;

    struct nodeHdlr nh;
    population_t pbest;
    population_t pplt[NPOPULATION];
    couple_t parents[NUM_PARENTS];
    couple_t childs[NUM_CHILDS];

    fl_t fl;

    
    wb_robot_init();
    car_devices_init( &nh );    // check devices
    robc_control_init();        // start engine
    ga_init( &defaultdvar );
    robc_init( &fl, "simulation_results" );

    int      num_iter = 0;
    bool     restart_search = true;
    double   bestrsl = 0;

    /* Run simulation */
    int ngen = 0;
    while( ngen < MAX_GEN ){
        /* Load the value of decision variables from heuristic algorithm */
            
        if( restart_search ){
            printf("  Restart search------------------------------------------------------\r\n");
            restart_search = false;
            bestrsl = 0;
            ga_init( &decvar );
            for( int n = 0; n < NPOPULATION; ++n ){
                double dist = 0;
                do{
                    statusVar_t stvar;
                    ga_generaterandMember( &decvar );
                    printf(" -New point:\r\n");
                    heuristics_print_point( &decvar ); 
                    run_simulation( &nh, &decvar, &stvar );
                    dist = heuristics_get_objetive( &stvar );
                }while( dist < 400 );
                printf(" -Register member %d, dist: %f\r\n", n, dist);
                ga_pushMemberToPopulation( &pplt[n], &decvar, dist );
            }
            memcpy( &bestdvar, &decvar, sizeof( decisionVar_t ) );
        }
            
            
        /*It is iterated over all members of the neighborhood until the best solution is found. */
        ga_printpoulation( pplt);
        for( int nbIter = 0; nbIter < NPOPULATION; ++nbIter ){
                
            statusVar_t stvar;
            struct results* fout = &fl.out; 

            ++num_iter;
            printf("\nGet member %d from iteration %d\r\n", nbIter , num_iter );
            ga_get_memberFromPopulation( &decvar, &pplt[nbIter] );
            heuristics_print_point( &decvar );  
            fout->dvar = &decvar;

            bool const simres = fout->simres = run_simulation( &nh, &decvar, &stvar );
            double dist = fout->res = heuristics_get_objetive( &stvar );

            dist = dist >= 400 ? dist : 0.0;
            ga_registerSolution( &pplt[nbIter], dist );

            printf("  Results from iter %d -> distance: %f, best distance: %f \r\n", 
            num_iter, dist, bestrsl ); 
                
            if ( dist > bestrsl  &&  simres ){
                bool const res = check_best_solution(  &nh, &decvar , &stvar, dist );
                if( res ){
                    ga_pushMemberToPopulation( &pbest, &decvar, dist );
                    memcpy( &bestdvar, &decvar, sizeof( decisionVar_t ) );
                    fout->bres = bestrsl = dist;
                }
            }
                
            /* Save simulation results in file */  
            robc_fl( &fl );
        }


        ngen = num_iter/ NPOPULATION;
        ga_printpoulation( pplt);
        ga_select( parents, pplt );
        ga_cross( childs, parents );
        ga_mutation( childs , &defaultdvar, ngen, MUT_NOT_UNIFORM );
        ga_recombination( pplt, childs, &pbest );

        printf("\nBest solution: \r\n");
        heuristics_print_point( &bestdvar );
    }
        
    printf("\r\n -------- End simulation ------\r\n");
        
    wb_supervisor_simulation_set_mode( WB_SUPERVISOR_SIMULATION_MODE_PAUSE );
    wb_supervisor_simulation_reset(); 
    wbu_driver_cleanup(); 
    wbu_car_cleanup();
    
    return 0;  // ignored
}
