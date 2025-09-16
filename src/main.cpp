#include "bmm/bayesian_mixture_model.hpp"

#include <console_bridge/console.h>

#include <Eigen/Dense>
#include <pcl/io/pcd_io.h>

#include <algorithm>
#include <random>
#include <numeric>

// Generate a tight Gaussian cluster plus uniform noise in a box
static void populateSynthetic(std::vector<geo::Vec3>& cluster,
                              std::vector<int>& gt_labels,  // 1 = cluster, 0 = noise
                              int n_cluster = 10000,
                              int n_noise = 1500) {
  cluster.clear();
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

  cluster.reserve(static_cast<size_t>(n_cluster + n_noise));
  gt_labels.reserve(static_cast<size_t>(n_cluster + n_noise));

  // Cluster points: Gaussian ball
  for (int i = 0; i < n_cluster; ++i) {
    geo::Vec3 v; v.x = ndx(rng); v.y = ndy(rng); v.z = ndz(rng);
    cluster.push_back(v);
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
    cluster.push_back(v);
    gt_labels.push_back(0);
    ++added;
  }
  if (added < n_noise) {
    CONSOLE_BRIDGE_logWarn("populateSynthetic: produced %d/%d noise points (tight box/exclusion)", added, n_noise);
  }

  // Shuffle points and labels with same permutation to avoid ordering bias
  std::vector<size_t> idx(cluster.size());
  std::iota(idx.begin(), idx.end(), 0);
  std::shuffle(idx.begin(), idx.end(), rng);

  std::vector<geo::Vec3> pts_shuf; pts_shuf.reserve(cluster.size());
  std::vector<int>      lab_shuf; lab_shuf.reserve(gt_labels.size());
  for (size_t i = 0; i < idx.size(); ++i) {
    pts_shuf.push_back(cluster[idx[i]]);
    lab_shuf.push_back(gt_labels[idx[i]]);
  }
  cluster.swap(pts_shuf);
  gt_labels.swap(lab_shuf);
}

int main() {
  // Synthetic input
  GMMParams params; // defaults from header
  params.alpha = 1.0;   // Dirichlet prior (1.0 = uniform)
  params.kappa0 = 0.0; // weak mean prior
  params.psi0 = 0.05;    // weak covariance prior
  params.nu0 = 4.0;     // min value for 3D
  std::vector<geo::Vec3> cluster;
  std::vector<int> gt_labels;  // 1=cluster, 0=noise
  populateSynthetic(cluster, gt_labels);

  // Identity pose (no transform)
  geo::Pose3D sensor_pose = geo::Pose3D::identity();

  // Fit (2 components: object + outliers)
  MAPGMM gmm(2, cluster, params);
  gmm.fit(cluster, sensor_pose);

  const std::vector<int> labels = gmm.get_labels();
  const int inlier_component = gmm.get_inlier_component();

  // Metrics vs ground truth
  size_t N = std::min(labels.size(), cluster.size());
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

  CONSOLE_BRIDGE_logInform("Pred inliers: %td | GT Pos: %d, Neg: %d | TP=%d FP=%d TN=%d FN=%d | P=%.3f R=%.3f F1=%.3f",
           std::count_if(labels.begin(), labels.end(), [&](int l){return l==inlier_component;}), Pos, Neg,
           TP, FP, TN, FN, prec, recall, f1);

  // Visualization: all points colored by prediction (green=inlier, red=outlier)
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZRGB>());
  cloud->reserve(N);
  for (size_t i = 0; i < N; ++i) {
    pcl::PointXYZRGB p;
    p.x = cluster[i].x; p.y = cluster[i].y; p.z = cluster[i].z;
    bool pred_in = (labels[i] == inlier_component);
    p.r = pred_in ? 0   : 255;
    p.g = pred_in ? 255 : 0;
    p.b = 0;
    cloud->push_back(p);
  }
  cloud->width = cloud->size(); cloud->height = 1; cloud->is_dense = false;
  if (pcl::io::savePCDFileBinary("clusters_pred.pcd", *cloud) == 0) {
    CONSOLE_BRIDGE_logInform("Wrote clusters_pred.pcd (green=inlier, red=outlier). View with: pcl_viewer clusters_pred.pcd");
  } else {
    CONSOLE_BRIDGE_logWarn("Failed to save clusters_pred.pcd");
  }

  // Optional: inliers-only export for a quick sanity check
  {
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr inliers(new pcl::PointCloud<pcl::PointXYZRGB>());
    inliers->reserve(N);
    for (size_t i = 0; i < N; ++i) {
      if (labels[i] != inlier_component) continue;
      pcl::PointXYZRGB p; p.x = cluster[i].x; p.y = cluster[i].y; p.z = cluster[i].z;
      p.r = p.g = p.b = 255;
      inliers->push_back(p);
    }
    inliers->width = inliers->size(); inliers->height = 1; inliers->is_dense = false;
    pcl::io::savePCDFileBinary("clusters_inliers_only.pcd", *inliers);
  }

  return 0;
}


// void applyDBSCANFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose, tue::Configuration config_) {
//     pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>());

//     // Build point cloud in map frame
//     for (const auto& point : cluster.points) {
//         geo::Vec3 p_map = sensor_pose * point;
//         cloud->push_back(pcl::PointXYZ(p_map.x, p_map.y, p_map.z));
//     }

//     // Create KdTree for search
//     pcl::search::KdTree<pcl::PointXYZ>::Ptr tree(new pcl::search::KdTree<pcl::PointXYZ>);
//     tree->setInputCloud(cloud);

//     // Initialize clustering parameters from config
//     double eps = 0.02;         // Default: 2cm
//     int min_samples = 30;
//     config_.value("eps", eps);
//     config_.value("min_samples", min_samples);


//     // Run clustering
//     std::vector<pcl::PointIndices> cluster_indices;
//     pcl::EuclideanClusterExtraction<pcl::PointXYZ> ec;
//     // This is the DBSCAN "epsilon" (ε) parameter.
//     // Defines the radius (2cm) in which to search for neighboring points
//     // Too small: Objects fragment into multiple clusters | Too large: Different objects merge together
//     ec.setClusterTolerance(eps);  // 2cm
//     // Minimum number of points required to form a valid cluster
//     // Smaller values: More sensitive to noise and small objects
//     // Larger values: Eliminates smaller objects but reduces noise
//     ec.setMinClusterSize(min_samples);
//     // Maximum number of points allowed in a cluster. Should be large enough for your largest expected object
//     ec.setMaxClusterSize(25000);
//     // Specifies the spatial indexing structure for neighbor searches
//     // KdTree is efficient for finding neighbors in 3D space
//     ec.setSearchMethod(tree);
//     ec.setInputCloud(cloud);
//     ec.extract(cluster_indices);

//     // Select largest cluster
//     size_t largest_idx = 0;
//     size_t largest_size = 0;

//     for (size_t i = 0; i < cluster_indices.size(); i++) {
//         if (cluster_indices[i].indices.size() > largest_size) {
//             largest_size = cluster_indices[i].indices.size();
//             largest_idx = i;
//         }
//     }

//     // Filter points
//     if (!cluster_indices.empty()) {
//         std::vector<geo::Vec3> filtered_points;
//         for (const auto& idx : cluster_indices[largest_idx].indices) {
//             filtered_points.push_back(cluster.points[idx]);
//         }
//         cluster.points = filtered_points;
//     }
// }
// // Visualization
// //#include <opencv2/highgui/highgui.hpp>

// // After collecting points in cluster.points:
// void applyGMMFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose) {
//     if (cluster.points.size() < 50) return;  // Too few points

//     // Convert points to OpenCV Mat (N x 3)
//     cv::Mat samples(cluster.points.size(), 3, CV_32F);
//     for (size_t i = 0; i < cluster.points.size(); i++) {
//         // Transform to map frame for consistent clustering
//         geo::Vec3 p_map = sensor_pose * cluster.points[i];
//         samples.at<float>(i, 0) = p_map.x;
//         samples.at<float>(i, 1) = p_map.y;
//         samples.at<float>(i, 2) = p_map.z;
//     }

//     // Create and train EM model
//     cv::Ptr<cv::ml::EM> em_model = cv::ml::EM::create();
//     em_model->setClustersNumber(2);  // Object + outliers
//     em_model->setCovarianceMatrixType(cv::ml::EM::COV_MAT_GENERIC);
//     em_model->setTermCriteria(cv::TermCriteria(cv::TermCriteria::COUNT+cv::TermCriteria::EPS, 100, 0.001));

//     // Train the model
//     cv::Mat labels, probs;
//     if (!em_model->trainEM(samples, cv::noArray(), labels, probs)) {
//         CONSOLE_BRIDGE_logWarn("GMM training failed, skipping filtering");
//         return;
//     }

//     // Count points per component
//     std::map<int, int> component_counts;
//     for (int i = 0; i < labels.rows; i++) {
//         component_counts[labels.at<int>(i, 0)]++;
//     }

//     // Find component with most points (argmax)
//     int main_component = 0;
//     int max_count = 0;
//     for (const auto& pair : component_counts) {
//         if (pair.second > max_count) {
//             max_count = pair.second;
//             main_component = pair.first;
//         }
//     }

//     // Keep all points from the largest component
//     std::vector<geo::Vec3> filtered_points;
//     for (int i = 0; i < labels.rows; i++) {
//         if (labels.at<int>(i, 0) == main_component) {
//             filtered_points.push_back(cluster.points[i]);
//         }
//     }

//     cluster.points = filtered_points;

//     // Only replace if we kept some points
//     //if (filtered_points.size() > cluster.points.size() * 0.1) {
//         //cluster.points = filtered_points;
//     //}
// }

// void applyVariationalBayesianGMMFiltering(EntityUpdate& cluster, const geo::Pose3D& sensor_pose) {
//     if (cluster.points.size() < 50) return;

//     // Create and fit VB-GMM
//     VBGMM vbgmm(2, cluster.points); // 2 components: object + outliers
//     vbgmm.fit(cluster.points, sensor_pose);

//     // Get results
//     std::vector<int> labels = vbgmm.get_labels();
//     int inlier_component = vbgmm.get_inlier_component();
//     double lower_bound = vbgmm.get_lower_bound();

//     CONSOLE_BRIDGE_logInform("VB-GMM lower bound: %.3f", lower_bound);

//     // Filter points based on component assignment
//     std::vector<geo::Vec3> filtered_points;
//     for (size_t i = 0; i < labels.size(); i++) {
//         if (labels[i] == inlier_component) {
//             filtered_points.push_back(cluster.points[i]);
//         }
//     }

//     if (!filtered_points.empty()) {
//         cluster.points = filtered_points;
//         CONSOLE_BRIDGE_logInform("VB filtering: kept %zu of %zu points",
//                 filtered_points.size(), cluster.points.size());
//     }
// }
