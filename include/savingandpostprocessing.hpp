#ifndef _savingandpostprocessing_hpp_
#define _savingandpostprocessing_hpp_

#include <cv.h>
#include <highgui.h>
#include <string>
#include <sstream>
#include <stdlib.h>
#include <fstream>
#include <iostream>
#include <typeinfo>

#include <pcl/common/common_headers.h>
#include <pcl/PolygonMesh.h>
#include <pcl/TextureMesh.h>
#include <pcl/io/ply_io.h>
#include <limits>

std::string to_string(int num);

int stoi(std::string text);

void save(std::string filename, pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr xyzrgb, pcl::PointCloud<pcl::PointNormal>::Ptr xyznormal, pcl::PolygonMesh &mesh);

void load(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb, pcl::PointCloud<pcl::PointNormal>::Ptr xyznormal, pcl::PolygonMesh &mesh);

// save xyz-data to threejsfiles
void compute_xyz(std::string filename, const int width, const int height) ;

// save rgb-data to threejsfiles
void compute_rgb(std::string filename, const int width, const int height) ;

// save faces to threejsfiles
void compute_faces(std::string filename, const int width, const int height);

// save normals to threejsfiles
void compute_normals(std::string filename, const int width, const int height);

// generate threejsfiles, if plyfiles are avaible
void buildthreejsdata(std::string filename);

#endif
