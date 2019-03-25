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
#include <webots/keyboard.h>
#include <webots/vehicle/driver.h>
#include <webots/distance_sensor.h>
#include <webots/supervisor.h>

#include <stdint.h>
#include <math.h>
#include <stdio.h>
#include <string.h>

#include "heuristics_algorithm.h"
// to be used as array indices
enum { X, Y, Z };

#define TIME_STEP 30
#define UNKNOWN 99999.99

// Line following PID
#define KP 0.75
#define KI 0.006
#define KD 2

// Size of the yellow line angle filter
#define FILTER_SIZE 3

// enabe various 'features'
bool enable_display = false;
bool has_gps = false;
bool has_camera = false;

// camera
WbDeviceTag camera;
int camera_width = -1;
int camera_height = -1;
double camera_fov = -1.0;


// speedometer
WbDeviceTag display;
int display_width = 0;
int display_height = 0;
WbImageRef speedometer_image = NULL;

// GPS
WbDeviceTag gps;
double gps_coords[3] = {0.0, 0.0, 0.0};
double gps_speed = 0.0;

// misc variables
double speed = 0.0;
double steering_angle = 0.0;

struct pid_param{
  double kp;
  double ki;
  double kd;
  bool reset;
};

static struct pid_param pidparam;



void print_help() {
  printf("You can drive this car!\n");
  printf("[UP]/[DOWN] - accelerate/slow down\n");
}





// set target speed
void set_speed(double kmh) {
  // max speed
  if (kmh > 250.0) kmh = 250.0;
  speed = kmh;
  //printf("setting speed to %g km/h\n", kmh);
  wbu_driver_set_cruising_speed(kmh);
}


// positive: turn right, negative: turn left
void set_steering_angle(double wheel_angle) {
  // limit the difference with previous steering_angle
  if (wheel_angle - steering_angle > 0.1)   wheel_angle = steering_angle + 0.1;
  if (wheel_angle - steering_angle < -0.1)  wheel_angle = steering_angle - 0.1;
  steering_angle = wheel_angle;
  // limit range of the steering angle
  if (wheel_angle > 0.5) wheel_angle = 0.5;
  else if (wheel_angle < -0.5) wheel_angle = -0.5;
  wbu_driver_set_steering_angle(wheel_angle);
}



void check_keyboard() {
  int key = wb_keyboard_get_key();
  switch (key) {
    case WB_KEYBOARD_UP:
      set_speed(speed + 5.0);
      break;
    case WB_KEYBOARD_DOWN:
      set_speed(speed - 5.0);
      break;
    case WB_KEYBOARD_RIGHT:
      break;
    case WB_KEYBOARD_LEFT:
      break;
    case 'A':
      break;
  }
}


// compute rgb difference
int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}


// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
double process_camera_image(const unsigned char *image) {
  int num_pixels = camera_height * camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camera_width - 0.5) * camera_fov;
}


// filter angle of the yellow line (simple average)
double filter_angle(double new_value) {
  static bool first_call = true;
  static double old_value[FILTER_SIZE];
  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN)
    return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}



void update_display() {
  const double NEEDLE_LENGTH = 60.0;

  // display background
  wb_display_image_paste(display, speedometer_image, 0, 0, false);

  // draw speedometer needle
  double speed = wbu_driver_get_current_speed();
  if (isnan(speed))
    speed = 0.0;
  double alpha = speed / 260.0 * 3.72 - 0.27;
  int x = -NEEDLE_LENGTH * cos(alpha);
  int y = -NEEDLE_LENGTH * sin(alpha);
  wb_display_draw_line(display, 100, 95, 100 + x, 95 + y);

  // draw text
  char txt[64];
  sprintf(txt, "GPS coords: %.1f %.1f", gps_coords[X], gps_coords[Z]);
  wb_display_draw_text(display, txt, 10, 130);
  sprintf(txt, "GPS speed:  %.1f", gps_speed);
  wb_display_draw_text(display, txt, 10, 140);
}


void compute_gps_speed() {
  const double *coords = wb_gps_get_values(gps);
  const double speed = wb_gps_get_speed(gps);
  // store into global variables
  gps_speed = speed * 3.6;  // convert from m/s to km/h
  memcpy(gps_coords, coords, sizeof(gps_coords));
}


double applyPID(double yellow_line_angle, struct pid_param* pid ) {
  static double oldValue = 0.0;
  static double integral = 0.0;

  if (pid->reset) {
    oldValue = yellow_line_angle;
    integral = 0.0;
    pid->reset = false;
  }

  // anti-windup mechanism
  if (signbit(yellow_line_angle) != signbit(oldValue))
    integral = 0.0;

  double diff = yellow_line_angle - oldValue;

  // limit integral
  if (integral < 30 && integral > -30)
    integral += yellow_line_angle;

  oldValue = yellow_line_angle;
  return pid->kp * yellow_line_angle + pid->ki * integral + pid->kd * diff;
}

void set_pid_param( struct pid_param* pid ){
  pid->kp = KP;
  pid->ki = KI;
  pid->kd = KD;
  pid->reset = true;
}

int main(int argc, char **argv) {
  wbu_driver_init();
  wb_robot_init();
  // check if there is a display
  int j = 0;
  for (j = 0; j < wb_robot_get_number_of_devices(); ++j) {
    WbDeviceTag device = wb_robot_get_device_by_index(j);
    const char *name = wb_device_get_name(device);
    if (strcmp(name, "display") == 0)
      enable_display = true;
    else if (strcmp(name, "gps") == 0)
      has_gps = true;
    else if (strcmp(name, "camera") == 0)
      has_camera = true;
  }

  // camera device
  if (has_camera) {
    camera = wb_robot_get_device("camera");
    wb_camera_enable(camera, TIME_STEP);
    camera_width = wb_camera_get_width(camera);
    camera_height = wb_camera_get_height(camera);
    camera_fov = wb_camera_get_fov(camera);
  }



  // initialize gps
  if (has_gps) {
    gps = wb_robot_get_device("gps");
    wb_gps_enable(gps, TIME_STEP);
  }

  // initialize display (speedometer)
  if (enable_display) {
    display = wb_robot_get_device("display");
    speedometer_image = wb_display_image_load(display, "speedometer.png");
  }

  // start engine
  if (has_camera)
    set_speed(50.0);  // km/h
  wbu_driver_set_hazard_flashers(true);
  wbu_driver_set_dipped_beams(true);
  wbu_driver_set_antifog_lights(true);
  wbu_driver_set_wiper_mode(SLOW);

  print_help();

  // allow to switch to manual control
  wb_keyboard_enable(TIME_STEP);
  
  WbNodeRef robot_node = wb_supervisor_node_get_self();
  WbFieldRef trans_field = wb_supervisor_node_get_field(robot_node, "translation");
  const double *initial_pose = wb_supervisor_field_get_sf_vec3f( trans_field );
  // main loop
  //while (wbu_driver_step() != -1) {
  while(true){
    double t;
    set_pid_param( &pidparam );
    printf("Param: %d \n",heuristics_loadParam() );
    for (t = 0.0; t < 90.0; t += TIME_STEP / 1000.0) {
      // get user input
      // updates sensors only every TIME_STEP milliseconds
      wb_robot_step(TIME_STEP);
      //wbu_driver_step( );
      
      check_keyboard();
      static int i = 0;

        // read sensors
        const unsigned char *camera_image = NULL; 
  
        if (has_camera) {
        
          camera_image = wb_camera_get_image(camera);
          double yellow_line_angle = filter_angle(process_camera_image(camera_image));
          if (yellow_line_angle != UNKNOWN) {
            // no obstacle has been detected, simply follow the line
            wbu_driver_set_brake_intensity(0.0);
            set_steering_angle( applyPID(yellow_line_angle, &pidparam ) );
            set_speed( speed );
          } else {
            // no obstacle has been detected but we lost the line => we brake and hope to find the line again
            wbu_driver_set_brake_intensity(0.4);
            pidparam.reset = true;
          }
        }
  
        // update stuff
        if (has_gps)
          compute_gps_speed();
        if (enable_display)
          update_display();

  
      ++i;
    }
    
    wb_supervisor_field_set_sf_vec3f(trans_field, initial_pose);
    wb_supervisor_node_reset_physics( robot_node );
    //wb_supervisor_simulation_reset();  
  }
  wbu_driver_cleanup();

  return 0;  // ignored
}
