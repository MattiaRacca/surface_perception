#include "surface_perception/surface_perceptor.h"

SurfacePerceptor::SurfacePerceptor()
    : nh_("~"),
      tf_listener_() {
      marker_pub_ = nh_.advertise<visualization_msgs::Marker>("surface_objects", 100);
      cropped_input_pub_ = nh_.advertise<sensor_msgs::PointCloud2>("cropped_cloud", 1, true);
      nh_.getParam("target_frame", target_frame_);
      pc_sub_ = nh_.subscribe<sensor_msgs::PointCloud2>("/cloud_in", 1, &SurfacePerceptor::Callback, this);
    }

void SurfacePerceptor::Callback(const sensor_msgs::PointCloud2ConstPtr& cloud) {
  SurfaceViz viz_(marker_pub_);
  PointCloudC::Ptr pcl_cloud_raw(new PointCloudC);
  pcl::fromROSMsg(*cloud, *pcl_cloud_raw);
  PointCloudC::Ptr pcl_cloud(new PointCloudC);

  if (cloud->header.frame_id != target_frame_) {
    tf_listener_.waitForTransform(target_frame_, cloud->header.frame_id,
                                  ros::Time(0), ros::Duration(1));
    tf::StampedTransform transform;
    tf_listener_.lookupTransform(target_frame_, cloud->header.frame_id,
                                 ros::Time(0), transform);
    Eigen::Affine3d affine;
    tf::transformTFToEigen(transform, affine);
    pcl::transformPointCloud(*pcl_cloud_raw, *pcl_cloud, affine);
    pcl_cloud->header.frame_id = target_frame_;
  } else {
    pcl_cloud = pcl_cloud_raw;
  }

  std::vector<int> indices;
  pcl::removeNaNFromPointCloud(*pcl_cloud, *pcl_cloud, indices);

  pcl::PointIndices::Ptr point_indices(new pcl::PointIndices);
  PointCloudC::Ptr cropped_cloud(new PointCloudC);
  sensor_msgs::PointCloud2 msg_out;

  pcl::CropBox<PointC> crop;
  crop.setInputCloud(pcl_cloud);

  double max_x, max_y, max_z, min_x, min_y, min_z;
  nh_.param<double>("crop_min_x", min_x, 0.0);
  nh_.param<double>("crop_min_y", min_y, 0.0);
  nh_.param<double>("crop_min_z", min_z, 0.0);
  nh_.param<double>("crop_max_x", max_x, 2.0);
  nh_.param<double>("crop_max_y", max_y, 2.0);
  nh_.param<double>("crop_max_z", max_z, 2.0);
  // ros::param::param("crop_min_x", min_x, 0.0);

  Eigen::Vector4f min;
  min << min_x, min_y, min_z, 1;
  crop.setMin(min);
  Eigen::Vector4f max;
  max << max_x, max_y, max_z, 1;
  crop.setMax(max);
  crop.filter(point_indices->indices);
  crop.filter(*cropped_cloud);

  pcl::toROSMsg(*cropped_cloud, msg_out);
  cropped_input_pub_.publish(msg_out);

  double horizontal_tolerance_degrees;
  ros::param::param("horizontal_tolerance_degrees",
                    horizontal_tolerance_degrees, 10.0);
  double margin_above_surface;
  ros::param::param("margin_above_surface", margin_above_surface, 0.025);
  double max_point_distance;
  ros::param::param("max_point_distance", max_point_distance, 0.015);
  double cluster_distance;
  ros::param::param("cluster_distance", cluster_distance, 0.01);
  int min_cluster_size;
  ros::param::param("min_cluster_size", min_cluster_size, 100);
  int max_cluster_size;
  ros::param::param("max_cluster_size", max_cluster_size, 5000);
  int min_surface_size;
  ros::param::param("min_surface_size", min_surface_size, 5000);
  int min_surface_exploration_iteration;
  ros::param::param("min_surface_exploration_iteration",
                    min_surface_exploration_iteration, 1000);

  surface_perception::Segmentation seg;
  seg.set_input_cloud(pcl_cloud);
  seg.set_indices(point_indices);
  seg.set_horizontal_tolerance_degrees(horizontal_tolerance_degrees);
  seg.set_margin_above_surface(margin_above_surface);
  seg.set_max_point_distance(max_point_distance);
  seg.set_cluster_distance(cluster_distance);
  seg.set_min_cluster_size(min_cluster_size);
  seg.set_max_cluster_size(max_cluster_size);
  seg.set_min_surface_size(min_surface_size);
  seg.set_min_surface_exploration_iteration(min_surface_exploration_iteration);

  std::vector<SurfaceObjects> surface_objects;
  bool success = seg.Segment(&surface_objects);

  if (!success || surface_objects.size() == 0) {
    ROS_ERROR("Failed to segment scene!");
  } else {
    size_t object_count = 0;
    for (size_t i = 0; i < surface_objects.size(); i++) {
      object_count += surface_objects[i].objects.size();
    }
    ROS_INFO("Found %ld surfaces with %ld objects", surface_objects.size(),
             object_count);
  }

  viz_.Hide();
  viz_.set_surface_objects(surface_objects);
  viz_.Show();
}
