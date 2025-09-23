# bmm_ros
Bayesian mixture model for 3D pointcloud denoising

It follows the implementation of: https://nbviewer.org/github/bertdv/BMLIP/blob/master/lessons/notebooks/Latent-Variable-Models-and-VB.ipynb

Next feature update of this function (optional):
Enable fully Variatiational Bayesian inference (use 
For the variational_gmm.cpp
Note that:

src/variational_gmm.cpp
Comment on lines +226 to +227
    // Add other terms for complete lower bound...
    // (Gaussian-Wishart terms omitted for brevity but should be included)

Review comment: The lower bound computation is incomplete as indicated by the comment. For a production VB-GMM implementation, the Gaussian-Wishart terms should be included for correct convergence monitoring.
