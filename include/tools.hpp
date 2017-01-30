#ifndef _tools_hpp_
#define _tools_hpp_

#include <opencv2/opencv.hpp>
#include <highgui.h>
#include <Eigen/Core>
#include <pcl/search/kdtree.h>
#include <iostream>
#include <string>
#include <cmath>
#include <pcl/common/common_headers.h>
#include <pcl/filters/crop_box.h>

#include "visualization.hpp"

//Counts how many points are actually in the PointCloud (without the ones with z=0)
int countEntries(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr);

int countEntries(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcptr);

// Determine the min and max values of x,y,z
void getSize(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr);

// Crops a point cloud with center x,y,z and edge length dx,dy,dz
void crop(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, float x, float y, float dx, float dy);

// Crops a colored point cloud with center x,y,z and edge length dx,dy,dz
void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcrgbptr, float x, float y, float dx, float dy);

// Function for finding the right parameters for cropping
void cropAndShow(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcrgbptr);

// Returns the fraction of points with the same coordinates in two pointclouds
double equals(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr2);

// Calculates the center of the point cloud using only the points with z!=0
void center(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, Eigen::Vector3f &centroid);

// Calculates the mean minimum distance and its standard deviation between the points and their k-nearest neighbours
double calcMeanMin(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr,int k);

void info(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr);
#endif
