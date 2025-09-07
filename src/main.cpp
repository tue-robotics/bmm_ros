#include "bayesian_gmm.h"
#include "ed/kinect/entity_update.h"
#include <Eigen/Dense>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/segmentation/extract_clusters.h>

GMMParams params;
EntityUpdate cluster;
geo::Pose3D sensor_pose;

int main() {
  MAPGMM gmm(2, cluster.points, params); // 2 components: object + outliers
  gmm.fit(cluster.points, sensor_pose);
  // Get component assignments and inlier component
  std::vector<int> labels = gmm.get_labels();
  int inlier_component = gmm.get_inlier_component();
  // Filter points
  std::vector<geo::Vec3> filtered_points;
  for (size_t i = 0; i < labels.size(); i++) {
    if (labels[i] == inlier_component) {
      filtered_points.push_back(cluster.points[i]);
    }
  }
  // Safety check
  if (filtered_points.size() > 10 &&
      filtered_points.size() > 0.1 * cluster.points.size()) {
    ROS_INFO("MAP-GMM filtering: kept %zu of %zu points (%.1f%%)",
             filtered_points.size(), cluster.points.size(),
             100.0 * filtered_points.size() / cluster.points.size());
    cluster.points = filtered_points;
  } else {
    ROS_WARN("MAP-GMM filtering: too few points kept, using original points");
  }
  return 0;
}