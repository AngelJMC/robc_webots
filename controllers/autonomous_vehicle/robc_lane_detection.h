
struct camera_param{
  int camera_width;
  int camera_height;
  double camera_fov;
};

double robc_getAngleFromCamera( const unsigned char *image, const struct camera_param* camParam );