#include "bayesian_gmm.h"
#include "ed/kinect/entity_update.h"
#include <Eigen/Dense>
#include <random>
#include <algorithm>

// Generate a tight Gaussian cluster plus uniform noise in a box
static void populateSynthetic(EntityUpdate& cluster,
                              int n_cluster = 300,
                              int n_noise = 150) {
  cluster.points.clear();
  cluster.pixel_indices.clear();

  std::mt19937 rng(42);
  // Cluster at (1.0, 0.5, 0.8) with small std dev
  std::normal_distribution<double> ndx(1.0, 0.05);
  std::normal_distribution<double> ndy(0.5, 0.05);
  std::normal_distribution<double> ndz(0.8, 0.05);

  // Uniform noise in a larger cube around origin
  std::uniform_real_distribution<double> ud(-1.5, 1.5);

  cluster.points.reserve(static_cast<size_t>(n_cluster + n_noise));

  for (int i = 0; i < n_cluster; ++i) {
    geo::Vec3 v; v.x = ndx(rng); v.y = ndy(rng); v.z = ndz(rng);
    cluster.points.push_back(v);
  }

  for (int i = 0; i < n_noise; ++i) {
    geo::Vec3 v; v.x = ud(rng); v.y = ud(rng); v.z = ud(rng);
    cluster.points.push_back(v);
  }

  // Optional: shuffle points
  std::shuffle(cluster.points.begin(), cluster.points.end(), rng);
}

int main() {
  // Synthetic input
  GMMParams params; // defaults from header
  EntityUpdate cluster;
  populateSynthetic(cluster);

  // Use identity pose (default-constructed Pose3D is effectively identity)
  geo::Pose3D sensor_pose; // leave as-is; operator* acts as identity when unset

  // Fit and filter
  MAPGMM gmm(2, cluster.points, params); // 2 components: object + outliers
  gmm.fit(cluster.points, sensor_pose);

  const std::vector<int> labels = gmm.get_labels();
  const int inlier_component = gmm.get_inlier_component();

  std::vector<geo::Vec3> filtered_points;
  filtered_points.reserve(cluster.points.size());
  for (size_t i = 0; i < labels.size(); ++i) {
    if (labels[i] == inlier_component) filtered_points.push_back(cluster.points[i]);
  }

  if (filtered_points.size() > 10 &&
      filtered_points.size() > 0.1 * cluster.points.size()) {
    ROS_INFO("MAP-GMM filtering: kept %zu of %zu points (%.1f%%)",
             filtered_points.size(), cluster.points.size(),
             100.0 * filtered_points.size() / cluster.points.size());
    cluster.points.swap(filtered_points);
  } else {
    ROS_WARN("MAP-GMM filtering: too few points kept, using original points");
  }

  return 0;
}