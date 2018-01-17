#ifndef _SURFACE_RANSAC_SURFACE_FINDER_H_
#define _SURFACE_RANSAC_SURFACE_FINDER_H_

#include <vector>
#include <map>

#include "pcl/point_cloud.h"
#include "pcl/point_types.h"
#include "pcl/PointIndices.h"
#include "pcl/ModelCoefficients.h"

namespace surface_ransac {
class SurfaceFinder {
 public:
  void setCloud(const pcl::PointCloud<pcl::PointXYZRGB>::Ptr& cloud);
  void setToleranceAngle(const double& degrees);
  void setMaxPointDistance(const double& dist);
  void setMaxIteration(const size_t& max_iter);
  void setSurfacePointThreshold(const size_t& min_point);
  void exploreSurfaces(const size_t& min_surface_amount,
                       const size_t& max_surface_amount,
                       std::vector<pcl::PointIndices::Ptr>* indices_internals,
                       std::vector<pcl::ModelCoefficients>* coeffs,
                       std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>* history = new std::vector<pcl::PointCloud<pcl::PointXYZRGB>::Ptr>());
 private:
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud_;
  double rad_;
  double dist_;
  size_t max_iter_;
  size_t min_point_;
  std::map<double, std::vector<int> > sortedIndices_;
};
}  // namespace surface_ransac


#endif  // _SURFACE_RANSAC_SURFACE_FINDER_H_
