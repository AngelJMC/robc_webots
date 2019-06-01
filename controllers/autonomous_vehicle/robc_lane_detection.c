

#include "robc_lane_detection.h"
#include <stdbool.h>

// Size of the yellow line angle filter
#define FILTER_SIZE 2
#define UNKNOWN 99999.99


static bool first_call = true;
static double old_value[FILTER_SIZE];
static int numFailImage = 0;
static double lastAngle = 0.0;

static double process_camera_image(const unsigned char *image, const struct camera_param* camParam);
static int color_diff(const unsigned char a[3], const unsigned char b[3]);
static double filter_angle(double new_value);



void robc_initLaneDetection( void ){
  first_call = true;
}

double robc_getAngleFromCamera( const unsigned char *image, const struct camera_param* camParam ){

  double angle = process_camera_image( image, camParam );
  if( angle == UNKNOWN ){
    ++numFailImage;
    angle = lastAngle;
  }
  else
    numFailImage = 0;

  lastAngle = angle;
  return filter_angle( numFailImage > 100 ? UNKNOWN : angle );
}


// filter angle of the yellow line (simple average)
static double filter_angle(double new_value) {


  int i;

  if (first_call || new_value == UNKNOWN) {  // reset all the old values to 0.0
    first_call = false;
    for (i = 0; i < FILTER_SIZE; ++i)
      old_value[i] = 0.0;
  } else {  // shift old values
    for (i = 0; i < FILTER_SIZE - 1; ++i)
      old_value[i] = old_value[i + 1];
  }

  if (new_value == UNKNOWN) return UNKNOWN;
  else {
    old_value[FILTER_SIZE - 1] = new_value;
    double sum = 0.0;
    for (i = 0; i < FILTER_SIZE; ++i)
      sum += old_value[i];
    return (double)sum / FILTER_SIZE;
  }
}

// returns approximate angle of yellow road line
// or UNKNOWN if no pixel of yellow line visible
static double process_camera_image(const unsigned char *image, const struct camera_param* camParam) {

  int num_pixels = camParam->camera_height * camParam->camera_width;  // number of pixels in the image
  const unsigned char REF[3] = {95, 187, 203};    // road yellow (BGR format)
  int sumx = 0;                                   // summed x position of pixels
  int pixel_count = 0;                            // yellow pixels count

  const unsigned char *pixel = image;
  int x;
  for (x = 0; x < num_pixels; x++, pixel += 4) {
    if (color_diff(pixel, REF) < 30) {
      sumx += x % camParam->camera_width;
      pixel_count++;  // count yellow pixels
    }
  }

  // if no pixels was detected...
  if (pixel_count == 0)
    return UNKNOWN;

  return ((double)sumx / pixel_count / camParam->camera_width - 0.5) * camParam->camera_fov;
}


// compute rgb difference
static int color_diff(const unsigned char a[3], const unsigned char b[3]) {
  
  int i, diff = 0;
  for (i = 0; i < 3; i++) {
    int d = a[i] - b[i];
    diff += d > 0 ? d : -d;
  }
  return diff;
}



