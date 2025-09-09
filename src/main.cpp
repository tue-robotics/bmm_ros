#include "bayesian_gmm.h"
#include "ed/kinect/entity_update.h"
#include <Eigen/Dense>
#include <random>
#include <algorithm>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <tuple>
#include <numeric>

// Generate a tight Gaussian cluster plus uniform noise in a box
static void populateSynthetic(EntityUpdate& cluster,
                              std::vector<int>& gt_labels,  // 1 = cluster, 0 = noise
                              int n_cluster = 10000,
                              int n_noise = 1500) {
  cluster.points.clear();
  cluster.pixel_indices.clear();
  gt_labels.clear();

  std::mt19937 rng(42);

  // Cluster center and stddevs
  const double cx = 1.0, cy = 0.5, cz = 0.8;
  std::normal_distribution<double> ndx(cx, 0.05);
  std::normal_distribution<double> ndy(cy, 0.05);
  std::normal_distribution<double> ndz(cz, 0.05);

  // Noise box and exclusion radius (keep noise outside a sphere)
  std::uniform_real_distribution<double> ud(-2.0, 2.0);
  const double R = 0.30;                       // cluster radius you want to keep noise out of
  const double Re2 = (R + 0.02) * (R + 0.02);  // small margin

  cluster.points.reserve(static_cast<size_t>(n_cluster + n_noise));
  gt_labels.reserve(static_cast<size_t>(n_cluster + n_noise));

  // Cluster points: Gaussian ball
  for (int i = 0; i < n_cluster; ++i) {
    geo::Vec3 v; v.x = ndx(rng); v.y = ndy(rng); v.z = ndz(rng);
    cluster.points.push_back(v);
    gt_labels.push_back(1);
  }

  // Noise points: uniform in box, reject those inside the sphere around the cluster
  int added = 0;
  int tries = 0;
  const int max_tries = n_noise * 50;
  while (added < n_noise && tries++ < max_tries) {
    geo::Vec3 v; v.x = cx + ud(rng); v.y = cy + ud(rng); v.z = cz + ud(rng);
    double dx = v.x - cx, dy = v.y - cy, dz = v.z - cz;
    if (dx*dx + dy*dy + dz*dz <= Re2) continue;  // reject inside sphere
    cluster.points.push_back(v);
    gt_labels.push_back(0);
    ++added;
  }
  if (added < n_noise) {
    ROS_WARN("populateSynthetic: produced %d/%d noise points (tight box/exclusion)", added, n_noise);
  }

  // Shuffle points and labels with same permutation to avoid ordering bias
  std::vector<size_t> idx(cluster.points.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::shuffle(idx.begin(), idx.end(), rng);

  std::vector<geo::Vec3> pts_shuf; pts_shuf.reserve(cluster.points.size());
  std::vector<int>      lab_shuf; lab_shuf.reserve(gt_labels.size());
  for (size_t i = 0; i < idx.size(); ++i) {
    pts_shuf.push_back(cluster.points[idx[i]]);
    lab_shuf.push_back(gt_labels[idx[i]]);
  }
  cluster.points.swap(pts_shuf);
  gt_labels.swap(lab_shuf);
}

int main() {
  // Synthetic input
  GMMParams params; // defaults from header
  EntityUpdate cluster;
  std::vector<int> gt_labels;  // 1=cluster, 0=noise
  populateSynthetic(cluster, gt_labels);

  // Identity pose (no transform)
  geo::Pose3D sensor_pose = geo::Pose3D::identity();

  // Fit (2 components: object + outliers)
  MAPGMM gmm(2, cluster.points, params);
  gmm.fit(cluster.points, sensor_pose);

  const std::vector<int> labels = gmm.get_labels();
  const int inlier_component = gmm.get_inlier_component();

  // Metrics vs ground truth
  size_t N = std::min(labels.size(), cluster.points.size());
  if (gt_labels.size() != N) gt_labels.resize(N, 0);

  int TP=0, TN=0, FP=0, FN=0;
  for (size_t i = 0; i < N; ++i) {
    bool pred_in = (labels[i] == inlier_component);
    bool gt_in   = (gt_labels[i] == 1);
    if (pred_in && gt_in) ++TP;
    else if (!pred_in && !gt_in) ++TN;
    else if (pred_in && !gt_in) ++FP;
    else ++FN;
  }
  const int Pos = TP + FN, Neg = TN + FP;
  double prec   = (TP + FP) ? double(TP) / (TP + FP) : 0.0;
  double recall = (TP + FN) ? double(TP) / (TP + FN) : 0.0;
  double f1     = (prec + recall) ? 2.0 * prec * recall / (prec + recall) : 0.0;

  ROS_INFO("Pred inliers: %d | GT Pos: %d, Neg: %d | TP=%d FP=%d TN=%d FN=%d | P=%.3f R=%.3f F1=%.3f",
           std::count_if(labels.begin(), labels.end(), [&](int l){return l==inlier_component;}), Pos, Neg,
           TP, FP, TN, FN, prec, recall, f1);

  // Visualization: all points colored by prediction (green=inlier, red=outlier)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->reserve(N);
  for (size_t i = 0; i < N; ++i) {
    pcl::PointXYZRGB p;
    p.x = cluster.points[i].x; p.y = cluster.points[i].y; p.z = cluster.points[i].z;
    bool pred_in = (labels[i] == inlier_component);
    p.r = pred_in ? 0   : 255;
    p.g = pred_in ? 255 : 0;
    p.b = 0;
    cloud->push_back(p);
  }
  cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = false;
  if (pcl::io::savePCDFileBinary("clusters_pred.pcd", *cloud) == 0) {
    ROS_INFO("Wrote clusters_pred.pcd (green=inlier, red=outlier). View with: pcl_viewer clusters_pred.pcd");
  } else {
    ROS_WARN("Failed to save clusters_pred.pcd");
  }

  // Optional: inliers-only export for a quick sanity check
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>());
    inliers->reserve(N);
    for (size_t i = 0; i < N; ++i) {
      if (labels[i] != inlier_component) continue;
      pcl::PointXYZRGB p; p.x = cluster.points[i].x; p.y = cluster.points[i].y; p.z = cluster.points[i].z;
      p.r = p.g = p.b = 255;
      inliers->push_back(p);
    }
    inliers->width = inliers->size(); inliers->height = 1; inliers->is_dense = false;
    pcl::io::savePCDFileBinary("clusters_inliers_only.pcd", *inliers);
  }

  return 0;
}