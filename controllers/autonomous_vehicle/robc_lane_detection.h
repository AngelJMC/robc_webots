#ifndef ROBC_LANE_DETECTION_H
#define ROBC_LANE_DETECTION_H

struct camera_param{
  int camera_width;
  int camera_height;
  double camera_fov;
};

void robc_initLaneDetection( void );

double robc_getAngleFromCamera( const unsigned char *image, const struct camera_param* camParam );

#endif /*ROBC_LANE_DETECTION_H*/