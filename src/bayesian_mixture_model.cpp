#include "bmm/bayesian_mixture_model.hpp"

#include <console_bridge/console.h>
#include <omp.h>

MAPGMM::MAPGMM(int n_components, const std::vector<geo::Vec3>& points, const GMMParams & params) : K_(n_components), inlier_component_(0), params_(params)
{
    // Initialize all containers to proper sizes
    means_.resize(K_);
    covs_.resize(K_);
    weights_ = Eigen::VectorXd::Zero(K_);
    labels_.clear();  // Will be resized during fit()

    // Initialize prior parameter containers
    mu0_.resize(K_);
    kappa0_.resize(K_);
    Psi0_.resize(K_);
    nu0_.resize(K_);

    // Initialize with identity matrices to avoid uninitialized memory
    for (int k = 0; k < K_; k++)
    {
        means_[k] = Eigen::Vector3d::Zero();
        covs_[k] = Eigen::Matrix3d::Identity();
        mu0_[k] = Eigen::Vector3d::Zero();
        Psi0_[k] = Eigen::Matrix3d::Identity();
    }

    // Set up actual prior values
    setupPriors(points);
}

void MAPGMM::fit(const std::vector<geo::Vec3>& points, const geo::Pose3D& sensor_pose)
{
    // Convert points to Eigen matrix
    int N = points.size();
    Eigen::MatrixXd data(N, 3);

    // Transform points to sensor frame (parallelized for large N)
    #pragma omp parallel for if(N > 500)
    for (int i = 0; i < N; i++)
    {
        geo::Vec3 p_map = sensor_pose * points[i];
        data(i, 0) = p_map.x;
        data(i, 1) = p_map.y;
        data(i, 2) = p_map.z;
    }

    // Compute bounding volume for the data so to handle uniform outlier component
    computeBoundingVolume(data);

    // Initialize parameters (weights, means, covariances)
    weights_ = Eigen::VectorXd::Constant(K_, 1.0/K_);
    means_.resize(K_);
    covs_.resize(K_);

    // Simple initialization: random points as centers for means and small covariances
    for (int k = 0; k < K_; k++)
    {
        int idx = rand() % N;
        means_[k] = data.row(idx).transpose();
        covs_[k] = Eigen::Matrix3d::Identity() * 0.01;  // Covariance here is not the prior but the initial guess of the cluster.
    }


    // EM iterations - reduced max_iter for speed
    int max_iter = 50;  // Reduced from 100
    // relative change in log-likelihood for convergence - slightly relaxed
    double likelihood_change = 5e-4;  // Relaxed from 1e-4
    // previous log-likelihood is used to compute relative change
    double prev_log_likelihood = -1e10;


    resp_ = Eigen::MatrixXd::Zero(N, K_);

    for (int iter = 0; iter < max_iter; iter++)
    {
        // E-step: Calculate responsibilities
        double log_likelihood = eStep(data, resp_);

        // Check convergence
        double change = std::abs((log_likelihood - prev_log_likelihood) / (std::abs(prev_log_likelihood) + 1e-10));
        if (iter > 0 && change < likelihood_change)
        {
            CONSOLE_BRIDGE_logInform("MAP-GMM converged after %d iterations", iter);
            break;
        }
        prev_log_likelihood = log_likelihood;

        // M-step: Update parameters with MAP
        mStep(data, resp_);
    }

    // Assign labels based on highest responsibility (parallelized)
    labels_.resize(N);
    #pragma omp parallel for if(N > 500)
    for (int i = 0; i < N; i++)
    {
        Eigen::VectorXd r = resp_.row(i);
        int max_idx = 0;
        r.maxCoeff(&max_idx);
        labels_[i] = max_idx;
    }

    // Determine which component is the inlier
    determineInlierComponent();
}

std::vector<int> MAPGMM::get_labels() const
{
    return labels_;
}
int MAPGMM::get_inlier_component() const
{
    return inlier_component_;
}



void MAPGMM::setupPriors(const std::vector<geo::Vec3>& points)
{
    if (mu0_.size() != static_cast<size_t>(K_) ||
        kappa0_.size() != static_cast<size_t>(K_) ||
        Psi0_.size() != static_cast<size_t>(K_) ||
        nu0_.size() != static_cast<size_t>(K_))
    {
        CONSOLE_BRIDGE_logError("Prior vectors not properly initialized in MAPGMM constructor");
        return;
    }
    // Dirichlet prior for weights (alpha>1 favors more uniform weights)
    alpha_ = params_.alpha;
    Eigen::Vector3d means = Eigen::Vector3d::Zero();
    for (const auto& p : points)
    {
        means += Eigen::Vector3d(p.x, p.y, p.z);
    }
    means /= points.size();
    for (int k = 0; k < K_; k++)
    {
        // Same prior for all components initially
        mu0_[k] = means;

        kappa0_[k] = params_.kappa0;  // Weak prior

        // Same covariance prior for all components
        Eigen::Matrix3d psi = Eigen::Matrix3d::Identity() * params_.psi0;
        Psi0_[k] = psi;
        nu0_[k] = params_.nu0;  // Minimum value for 3D (d+1)
    }
}

void MAPGMM::computeBoundingVolume(const Eigen::MatrixXd& data)
{
    Eigen::Vector3d min_vals = data.colwise().minCoeff();
    Eigen::Vector3d max_vals = data.colwise().maxCoeff();
    volume_ = (max_vals - min_vals).prod();  // volume = (xmax - xmin) * (ymax - ymin) * (zmax - zmin)
    if (volume_ <= 1e-12)
    {
        volume_ = 1e-6;  // avoid division by zero in uniform component
        CONSOLE_BRIDGE_logWarn("MAP-GMM: bounding volume too small, clamping to %g", volume_);
    }
}

double MAPGMM::eStep(const Eigen::MatrixXd& data, Eigen::MatrixXd& resp_)
{
    // Number of data points
    int N = data.rows();

    // ========== OPTIMIZATION 1: Pre-compute Sigma inverses and log determinants ==========
    // Compute these ONCE per iteration, not N×K times
    std::vector<Eigen::Matrix3d> Sigma_inv(K_);
    std::vector<double> log_det(K_);
    std::vector<double> log_w(K_);

    double log_uniform = std::log(1.0 / volume_);
    log_w[0] = std::log(std::max(weights_[0], 1e-12));

    for (int k = 1; k < K_; k++)
    {
        // Regularize covariance for numerical stability
        Eigen::Matrix3d Sigma = covs_[k] + Eigen::Matrix3d::Identity() * 1e-9;
        double det = Sigma.determinant();
        if (det <= 1e-18 || !std::isfinite(det))
        {
            Sigma += Eigen::Matrix3d::Identity() * 1e-6;
            det = Sigma.determinant();
        }
        Sigma_inv[k] = Sigma.inverse();
        log_det[k] = std::log(std::max(det, 1e-24));
        log_w[k] = std::log(std::max(weights_[k], 1e-12));
    }

    // ========== OPTIMIZATION 2 & 3: Vectorized + OpenMP parallel computation ==========
    // Calculate log probabilities for each data point and component
    Eigen::MatrixXd log_probs(N, K_);

    // Use OpenMP reduction for log_likelihood to avoid race conditions
    double log_likelihood = 0.0;

    // Parallelize over points (N is typically large: 100-5000)
    // Use if(N > 200) to avoid overhead for small datasets
    #pragma omp parallel for reduction(+:log_likelihood) if(N > 200)
    for (int i = 0; i < N; i++)
    {
        Eigen::Vector3d x = data.row(i).transpose();

        // Component 0: uniform outlier
        log_probs(i, 0) = log_w[0] + log_uniform;

        // Components 1..K-1: Gaussian
        for (int k = 1; k < K_; k++)
        {
            Eigen::Vector3d diff = x - means_[k];
            // Use pre-computed inverse (OPTIMIZATION 1)
            double quad = diff.transpose() * Sigma_inv[k] * diff;
            if (!std::isfinite(quad)) quad = 1e6;

            double log_prob = -0.5 * quad - 0.5 * log_det[k] - 1.5 * std::log(2 * M_PI);
            log_probs(i, k) = log_w[k] + log_prob;
        }

        // ========== Responsibilities (softmax) - computed per-point to enable parallelism ==========
        // Numerical stability: subtract max value
        double max_log_prob = log_probs.row(i).maxCoeff();
        double sum_exp = 0.0;
        for (int k = 0; k < K_; k++)
        {
            double exp_val = std::exp(log_probs(i, k) - max_log_prob);
            resp_(i, k) = exp_val;
            sum_exp += exp_val;
        }
        // Normalize
        for (int k = 0; k < K_; k++)
        {
            resp_(i, k) /= sum_exp;
        }

        // Add to log likelihood (reduction handles thread-safety)
        log_likelihood += max_log_prob + std::log(sum_exp);
    }

    return log_likelihood;
}

void MAPGMM::mStep(const Eigen::MatrixXd& data, const Eigen::MatrixXd& resp_)
{
    int N = data.rows();

    // ========== OPTIMIZATION: Vectorized computation using Eigen ==========

    // Pre-compute responsibility sums for all components (vectorized)
    Eigen::VectorXd Nk = resp_.colwise().sum();

    // Update weights for all components at once (vectorized)
    weights_ = (Nk.array() + alpha_ - 1.0) / (N + K_ * alpha_ - K_);

    // Process each Gaussian component (k > 0, skip uniform outlier)
    // Note: K is small (typically 2), so no benefit from parallelizing this outer loop
    for (int k = 1; k < K_; k++)
    {
        double Nk_k = Nk[k];
        if (Nk_k < 1e-10)
        {
            // Component has no responsibility, skip update
            continue;
        }

        // ========== Vectorized mean computation ==========
        // mean_data = sum(resp_ik * x_i) / Nk
        // Using Eigen: (resp_.col(k).transpose() * data) gives weighted sum
        Eigen::Vector3d mean_data = (resp_.col(k).transpose() * data).transpose() / Nk_k;

        // Mean update with prior
        means_[k] = (Nk_k * mean_data + kappa0_[k] * mu0_[k]) / (Nk_k + kappa0_[k]);

        // ========== Vectorized covariance computation ==========
        // Center the data around mean_data
        Eigen::MatrixXd centered = data.rowwise() - mean_data.transpose();  // N x 3

        // Weight by sqrt of responsibilities for efficient computation
        // cov = sum(resp_ik * (x_i - mu)(x_i - mu)^T) / Nk
        // This is equivalent to: centered.T * diag(resp) * centered / Nk
        Eigen::VectorXd sqrt_resp = resp_.col(k).array().sqrt();
        Eigen::MatrixXd weighted_centered = sqrt_resp.asDiagonal() * centered;  // N x 3
        Eigen::Matrix3d cov_data = (weighted_centered.transpose() * weighted_centered) / Nk_k;

        // Additional term from the mean update
        Eigen::Vector3d mean_diff = mean_data - mu0_[k];
        Eigen::Matrix3d mean_cov = (kappa0_[k] * Nk_k / (kappa0_[k] + Nk_k)) *
                                    mean_diff * mean_diff.transpose();

        // MAP update for covariance (Inverse-Wishart prior)
        covs_[k] = (Psi0_[k] + Nk_k * cov_data + mean_cov) / (Nk_k + nu0_[k] + 3 + 1);

        // Add small regularization to ensure positive definiteness
        covs_[k] += Eigen::Matrix3d::Identity() * 1e-6;
    }

    // Normalize weights
    weights_ /= weights_.sum();
}

void MAPGMM::determineInlierComponent()
{
    // ========== OPTIMIZATION: Vectorized computation ==========
    // Compute N_k: total responsibility mass for each component
    Eigen::VectorXd Nk = resp_.colwise().sum();

    // Find the component with max Nk
    // Skip the outlier uniform distribution component (k=0)
    inlier_component_ = 1;
    for (int k = 2; k < K_; k++)
    {
        if (Nk[k] > Nk[inlier_component_])
        {
            inlier_component_ = k;
        }
    }

    CONSOLE_BRIDGE_logInform("Bayesian selection: inlier component = %d (Nk = %.2f)", inlier_component_, Nk[inlier_component_]);

}
