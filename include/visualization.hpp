#ifndef _visualization_hpp_
#define _visualization_hpp_

#include <pcl/visualization/pcl_visualizer.h>
#include <boost/thread/thread.hpp>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/common/transforms.h>
#include <Eigen/Core>

#include "tools.hpp"

void viewCloud (pcl::PolygonMesh &mesh);

void viewCloud (pcl::TextureMesh &texturemesh);

void viewCloud (pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr);

void viewCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgbptr);

void viewCloud (pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcrgbptr, pcl::PointCloud<pcl::Normal>::ConstPtr normptr);

#endif
