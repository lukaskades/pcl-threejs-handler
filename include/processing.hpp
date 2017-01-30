#ifndef _processing_hpp_
#define _processing_hpp_

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <string>
#include <cmath>

#include <pcl/point_types.h>
#include <pcl/kdtree/kdtree_flann.h>
#include <pcl/features/normal_3d.h>
#include <pcl/features/normal_3d_omp.h>
#include <pcl/features/integral_image_normal.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <Eigen/Core>
#include <pcl/surface/poisson.h>
#include <pcl/filters/fast_bilateral.h>
#include <pcl/filters/bilateral.h>
#include <pcl/surface/gp3.h>
#include <pcl/filters/median_filter.h>

#include "tools.hpp"
#include "visualization.hpp"

// Median filter
void medianFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, std::string method, int window_size, float max_allowed_movement);

// BilateralFilter
void bilateralFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, std::string method, double sigmaS, double sigmaR);

// Normal estimation via least-square plane fitting
pcl::PointCloud<pcl::Normal>::Ptr leastsquareNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr, std::string method, int k, double radius = 0, bool openmp = false);

// Normal estimation via integral images
pcl::PointCloud<pcl::Normal>::Ptr integralimagesNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr, std::string method, int art, float max_depth_change_factor, float normal_smoothing_size, bool depth_dependent_smoothing);

// Generating PolygonMesh using GreedyProjectionTriangulation
pcl::PolygonMesh gptGenerateMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr pcnormptr, std::string method = "gpt", double mu = 2.5, double radius = 2, int nnn = 100, double minimum_angle = M_PI/18, double maximum_angle = 2*M_PI/3, double eps_angle = M_PI/4, bool consistent = false, bool consistent_ordering = false);

// Generating PolygonMesh using Poisson
pcl::PolygonMesh poissonGenerateMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr pcnormptr, std::string method, int depth, int min_depth, float point_weight, float scale = 1.1, int solver_divide = 8, int iso_divide = 8, float samples_per_node = 2, bool confidence = 1, bool output_polygons = 0, int degree = 2, bool manifold = 0);

#endif
