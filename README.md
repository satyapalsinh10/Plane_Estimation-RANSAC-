# RANSAC Plane Estimation in 3D Point Clouds using Open3D

## Overview

This repository contains a Python implementation of the RANSAC algorithm for robust plane fitting in 3D point clouds using the Open3D library. The code iteratively samples points, calculates plane parameters, and identifies inliers based on a specified distance threshold. The best-fit plane and corresponding inliers are extracted from a demo point cloud, and inliers are colored in red for visualization.

## Dependencies

Ensure you have the following dependencies installed:

- Open3D
- NumPy

You can install them using:

```bash
$ pip install numpy
$ pip install open3d
```

## Verify installation

```
$ python -c "import open3d as o3d; print(o3d.__version__)"
```


## Usage

1. **Clone the repository:**

```
$ git clone https://github.com/satyapalsinh10/Plane_Estimation_RANSAC.git
$ cd Plane_Estimation_RANSAC
```


2. **Run the example script:**

```
$ python ransac_plane_est.py
```

This script reads a demo point cloud, fits a plane using the RANSAC algorithm, and visualizes the original point cloud with the best-fit plane highlighted.

## Customization

You can customize the RANSAC parameters such as max_iterations and distance_threshold in the fit_plane_ransac function to suit your specific use case.

## File Structure

- ransac_plane_est.py: The main script containing the RANSAC plane fitting implementation.

- data/: Directory containing the demo point cloud file (in PCD format).

- README.md: This file provides an overview and instructions.

## Visualization

The visualization is achieved using the Open3D library. Adjust the parameters in the visualization section of the script for a better view of the point cloud and the fitted plane.

## Acknowledgments

- The implementation is based on the RANSAC algorithm.

- The demo point cloud is provided by Open3D.
