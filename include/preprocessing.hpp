#ifndef _preprocessing_hpp_
#define _preprocessing_hpp_

#include <opencv2/opencv.hpp>
#include <cstdlib>
#include <cstdio>
#include <iostream>
#include <cstring>
#include <ctime>
#include <cmath>
#include <fstream>
#include <complex>
#include <vector>

//void show(std::vector<std::vector<double> > &a(height, std::vector<double>(width)));

void interpolateDepthData(std::string filename, std::string newfilename, int scale = 1);

#endif
