#ifndef SURFACE_PERCEPTOR_H
#define SURFACE_PERCEPTOR_H

#include <string>
#include <vector>

#include "Eigen/Eigen"
#include "pcl/common/common.h"
#include "pcl/filters/crop_box.h"
#include "pcl_conversions/pcl_conversions.h"
#include "ros/ros.h"
#include "sensor_msgs/PointCloud2.h"
#include "surface_perception/segmentation.h"
#include "surface_perception/surface_objects.h"
#include "surface_perception/typedefs.h"
#include "surface_perception/visualization.h"
#include "tf/transform_listener.h"
#include "tf_conversions/tf_eigen.h"

using surface_perception::SurfaceObjects;
using surface_perception::SurfaceViz;

class SurfacePerceptor {
 public:
  SurfacePerceptor();
  void Callback(const sensor_msgs::PointCloud2ConstPtr& cloud);

 private:
  ros::NodeHandle nh_;
  ros::Publisher marker_pub_; 
  
  ros::Publisher cropped_input_pub_;
  ros::Subscriber pc_sub_;
  std::string target_frame_;
  tf::TransformListener tf_listener_;
  SurfaceViz *viz_;
};

#endif // SURFACE_PERCEPTOR_H
