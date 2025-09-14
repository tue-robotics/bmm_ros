#include <gtest/gtest.h>
#include <random>
#include <numeric>
#include <algorithm>

#include "bayesian_mixture_model.hpp"  // MAPGMM, GMMParams
#include <geolib/datatypes.h>  // geo::Vec3, geo::Pose3D

namespace {

// Synthetic data generator similar to main.cpp::populateSynthetic but without EntityUpdate
static void generateSynthetic(std::vector<geo::Vec3>& points,
                              std::vector<int>& gt_labels,  // 1 = cluster, 0 = noise
                              int n_cluster,
                              int n_noise,
                              double stddev = 0.05,
                              unsigned seed = 42)
{
    points.clear();
    gt_labels.clear();
    std::mt19937 rng(seed);

    const double cx = 1.0, cy = 0.5, cz = 0.8;
    std::normal_distribution<double> ndx(cx, stddev);
    std::normal_distribution<double> ndy(cy, stddev);
    std::normal_distribution<double> ndz(cz, stddev);

    std::uniform_real_distribution<double> ud(-2.0, 2.0);
    const double R = 0.30;
    const double Re2 = (R + 0.02) * (R + 0.02);

    points.reserve(static_cast<size_t>(n_cluster + n_noise));
    gt_labels.reserve(static_cast<size_t>(n_cluster + n_noise));

    for (int i = 0; i < n_cluster; ++i) {
        geo::Vec3 v; v.x = ndx(rng); v.y = ndy(rng); v.z = ndz(rng);
        points.push_back(v);
        gt_labels.push_back(1);
    }

    int added = 0;
    int tries = 0;
    const int max_tries = n_noise * 50;
    while (added < n_noise && tries++ < max_tries) {
        geo::Vec3 v; v.x = cx + ud(rng); v.y = cy + ud(rng); v.z = cz + ud(rng);
        double dx = v.x - cx, dy = v.y - cy, dz = v.z - cz;
        if (dx*dx + dy*dy + dz*dz <= Re2) continue;
        points.push_back(v);
        gt_labels.push_back(0);
        ++added;
    }

    // Shuffle in unison
    std::vector<size_t> idx(points.size());
    std::iota(idx.begin(), idx.end(), 0);
    std::shuffle(idx.begin(), idx.end(), rng);

    std::vector<geo::Vec3> pts_shuf; pts_shuf.reserve(points.size());
    std::vector<int>      lab_shuf; lab_shuf.reserve(gt_labels.size());
    for (size_t i = 0; i < idx.size(); ++i) {
        pts_shuf.push_back(points[idx[i]]);
        lab_shuf.push_back(gt_labels[idx[i]]);
    }
    points.swap(pts_shuf);
    gt_labels.swap(lab_shuf);
}

struct Case {
    int n_cluster;
    int n_noise;
    double stddev;
    unsigned seed;
};

} // namespace

// Base fixture
class BmmInferenceTest : public ::testing::Test {
protected:
    void SetUp() override { }
    void TearDown() override { }

    // Runs the GMM and returns labels and inlier component
    void runGmm(const std::vector<geo::Vec3>& points,
                std::vector<int>& labels,
                int& inlier_component,
                int K = 2)
    {
        // Ensure deterministic init of rand() used inside MAPGMM for centers
        std::srand(1337);

        GMMParams params; // defaults from header
        params.alpha = 1.0;   // Dirichlet prior (1.0 = uniform)
        params.kappa0 = 0.0; // weak mean prior
        params.psi0 = 0.05;    // weak covariance prior
        params.nu0 = 4.0;     // min value for 3D
        MAPGMM gmm(K, points, params);
        geo::Pose3D sensor_pose = geo::Pose3D::identity();
        gmm.fit(points, sensor_pose);
        labels = gmm.get_labels();
        inlier_component = gmm.get_inlier_component();
    }

    // Computes P/R/F1
    static std::tuple<double,double,double> metrics(const std::vector<int>& labels,
                                                    const std::vector<int>& gt_labels,
                                                    int inlier_component)
    {
        size_t N = std::min(labels.size(), gt_labels.size());
        int TP=0, TN=0, FP=0, FN=0;
        for (size_t i = 0; i < N; ++i) {
            bool pred_in = (labels[i] == inlier_component);
            bool gt_in   = (gt_labels[i] == 1);
            if (pred_in && gt_in) ++TP;
            else if (!pred_in && !gt_in) ++TN;
            else if (pred_in && !gt_in) ++FP;
            else ++FN;
        }
        double prec   = (TP + FP) ? double(TP) / (TP + FP) : 0.0;
        double recall = (TP + FN) ? double(TP) / (TP + FN) : 0.0;
        double f1     = (prec + recall) ? 2.0 * prec * recall / (prec + recall) : 0.0;
        return std::make_tuple(prec, recall, f1);
    }
};

// Parameterized suite sweeps sizes/noise
class BmmInferenceParamTest : public BmmInferenceTest,
                              public ::testing::WithParamInterface<Case> {};

TEST_P(BmmInferenceParamTest, FittingQualityAboveThreshold)
{
    const auto& c = GetParam();
    std::vector<geo::Vec3> pts;
    std::vector<int> gt;
    generateSynthetic(pts, gt, c.n_cluster, c.n_noise, c.stddev, c.seed);

    std::vector<int> labels;
    int inlier_component = -1;
    runGmm(pts, labels, inlier_component, /*K=*/2);

    ASSERT_EQ(labels.size(), pts.size());
    ASSERT_NE(inlier_component, -1);
    // Ensure valid label range
    for (int l : labels) {
        ASSERT_GE(l, 0);
        ASSERT_LT(l, 2);
    }

    auto [prec, recall, f1] = metrics(labels, gt, inlier_component);

    // Be generous to allow for EM randomness but still meaningful
    EXPECT_GE(prec, 0.85);
    EXPECT_GE(recall, 0.85);
    EXPECT_GE(f1, 0.85);
}

TEST_F(BmmInferenceTest, InlierComponentIsNotUniformOutlier)
{
    // Hard case still should pick a Gaussian as inlier (k != 0)
    std::vector<geo::Vec3> pts;
    std::vector<int> gt;
    generateSynthetic(pts, gt, /*cluster*/2000, /*noise*/500, /*stddev*/0.06, /*seed*/7);

    std::vector<int> labels;
    int inlier_component = -1;
    runGmm(pts, labels, inlier_component, /*K=*/2);

    ASSERT_EQ(labels.size(), pts.size());
    EXPECT_NE(inlier_component, 0);
}

TEST_F(BmmInferenceTest, DeterministicUnderSeededRand)
{
    std::vector<geo::Vec3> pts;
    std::vector<int> gt;
    generateSynthetic(pts, gt, 4000, 500, 0.05, 123);

    std::vector<int> labels1, labels2;
    int inlier1=-1, inlier2=-1;

    // Same seed before each run -> same random centers -> same labels
    std::srand(2025);
    runGmm(pts, labels1, inlier1, 2);
    std::srand(2025);
    runGmm(pts, labels2, inlier2, 2);

    ASSERT_EQ(labels1.size(), labels2.size());
    EXPECT_EQ(inlier1, inlier2);
    EXPECT_EQ(labels1, labels2);
}

// Instantiate parameter cases
INSTANTIATE_TEST_SUITE_P(
    MAPGMM_Quality,
    BmmInferenceParamTest,
    ::testing::Values(
        Case{5000, 1000, 0.05, 42},
        Case{2000,  400, 0.05, 1337},
        Case{1000,  200, 0.06, 7}
    )
);
