#include <surface_perception/surface_perceptor.h>

int main(int argc, char **argv)
{
  ros::init(argc, argv, "surface_perceptor");

  SurfacePerceptor node;

  ros::spin();
}
