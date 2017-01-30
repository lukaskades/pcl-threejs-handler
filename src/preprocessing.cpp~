#include "preprocessing.hpp"

void interpolateDepthData(std::string filename, std::string newfilename, int scale)
{
  //Load rgb-image
  cv::Mat3b color_image = cv::imread("./../data/undistfiles/"+filename+"_rgb.ppm", CV_LOAD_IMAGE_COLOR);
  
  //Load disparity image
  cv::Mat1s depth_image = cv::imread("./../data/undistfiles/"+filename+"_depth.pgm", CV_LOAD_IMAGE_ANYDEPTH);

  const int width = (int)(color_image.cols/scale);
  const int height = (int)(color_image.rows/scale);
  
  cv::Mat3b newcolor_image(height,width); // scaled image
  std::vector<std::vector<double> > data(height, std::vector<double>(width)); // Array for loading the depth data
  std::vector<std::vector<double> > datax(height, std::vector<double>(width)); // Array for storage of the x index
  std::vector<std::vector<double> > datay(height, std::vector<double>(width)); // Array for storage of the y index
  
  //Loading the data
  for(int v=0; v< color_image.rows; v++)     //2-D indexing
  {
      for(int u=0; u< color_image.cols; u++) {
	if((u%scale==0)&&(v%scale==0))
	{
          datax[v/scale][u/scale]=u/scale;
          datay[v/scale][u/scale]=v/scale;
          data[v/scale][u/scale]=depth_image(v,u);
	  newcolor_image(v/scale,u/scale)=color_image(v,u);
	}
      }
  }
  cv::imwrite("./../data/undistfiles/"+newfilename+"_rgb.ppm",newcolor_image);
  
  std::vector<std::vector<double> > zeros; // Array for saving the coordinates of occuring pixels with no depth data (zeros) and the corresponding number of neighbours without depth data 
  std::vector<std::vector<int> > num(5, std::vector<int>(0)); // Array for counting the number of pixels with zero to four neighbours without depth data
  int counter = 0;
  
  // count zeros and save their positions
  for(int v = 0; v < height; v++)
    for(int u = 0; u < width; u++)
      if(data[v][u]==0) {
	    if(v-1<0 & u-1<0) {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v][u+1]==0)+2};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );	      
	      zeros.push_back(a);
	    }
	    else if(v-1<0 & u+1>width-1) {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v][u-1]==0)+2};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if(v-1<0) {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v][u+1]==0)+(double)(data[v][u-1]==0)+1};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if(v+1>height-1 & u-1<0) {
	      double values[] =  {(double)v,(double)u,(double)(data[v-1][u]==0)+(double)(data[v][u+1]==0)+2};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if(v+1>height-1 & u+1>width-1) {
	      double values[] = {(double)v,(double)u,(double)(data[v-1][u]==0)+(double)(data[v][u-1]==0)+2};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if (v+1>height-1) {
	      double values[] = {(double)v,(double)u,(double)(data[v-1][u]==0)+(double)(data[v][u+1]==0)+(double)(data[v][u-1]==0)+1};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if (u-1<0) {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v-1][u]==0)+(double)(data[v][u+1]==0)+1};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	    else if (u+1>width-1) {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v-1][u]==0)+(double)(data[v][u-1]==0)+1};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	    }
	  else {
	      double values[] = {(double)v,(double)u,(double)(data[v+1][u]==0)+(double)(data[v-1][u]==0)+(double)(data[v][u+1]==0)+(double)(data[v][u-1]==0)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      zeros.push_back(a);
	  }
          num[(int)zeros[counter][2]].push_back(counter);
	  counter++;
      }
      
  // interpolation like described in the report until no pixel with undefind depth information remains
  while(zeros.size()>0)
  {
    int k = 0;
    while(num[k].empty()) k++; // search for pixels with the lowest number of neighbours without depth information
    std::vector<std::vector<double> > dat; // array as buffer memory for data (parallel updates)
    int change = num[k].size()-1;
    while(!num[k].empty())
    {
	  int y = (int)zeros[num[k][change]][0];
	  int x = (int)zeros[num[k][change]][1];
	    if(y-1<0 & x-1<0) {
	      double values[] = {(double)y,(double)x,(data[y+1][x]*(data[y+1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(y-1<0 & x+1>width-1) {
	      double values[] = {(double)y,(double)x,(data[y+1][x]*(data[y+1][x]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(y-1<0) {
	      double values[] = {(double)y,(double)x,(data[y+1][x]*(data[y+1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(y+1>height-1 & x-1<0) {
	      double values[] = {(double)y,(double)x,(data[y-1][x]*(data[y-1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(y+1>height-1 & x+1>width-1) {
	      double values[] = {(double)y,(double)x,(data[y-1][x]*(data[y-1][x]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(y+1>height-1) {
	      double values[] = {(double)y,(double)x,(data[y-1][x]*(data[y-1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(x-1<0) {
	      double values[] = {(double)y,(double)x,(data[y-1][x]*(data[y-1][x]!=0)+data[y+1][x]*(data[y+1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	    else if(x+1>width-1) {
	      double values[] = {(double)y,(double)x,(data[y-1][x]*(data[y-1][x]!=0)+data[y+1][x]*(data[y+1][x]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	      std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	      dat.push_back(a);
	    }
	  else {
	    double values[] = {(double)y,(double)x,(data[y+1][x]*(data[y+1][x]!=0)+data[y-1][x]*(data[y-1][x]!=0)+data[y][x+1]*(data[y][x+1]!=0)+data[y][x-1]*(data[y][x-1]!=0))/(4.0-k)};
  	    std::vector<double> a (values, values + sizeof(values) / sizeof(double) );
	    dat.push_back(a);
	  }
	  zeros.erase(zeros.begin()+num[k][change]);
	  num[k].erase(num[k].begin()+change);
	  change--;
    }
    
    // allocate interpolated values to original data
    for(int i = 0; i < (int)dat.size(); i++)
      data[(int)dat[i][0]][(int)dat[i][1]] = (int)round(dat[i][2]);
    dat.clear();
    for(int i = 0; i < (int)num.size(); i++)
      num[i].clear();
    counter = 0;
    // reallocate adjusted values to the zeros and num arrays
    for(int i = 0; i < (int)zeros.size(); i++)
    {
      int y = (int)zeros[i][0];
      int x = (int)zeros[i][1];
      if(data[y][x]==0) {
	    if(y-1<0 & x-1<0) {
	      num[(int)(data[y+1][x]==0)+(int)(data[y][x+1]==0)+2].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y][x+1]==0)+2;
	    }
	    else if(y-1<0 & x+1>width-1) {
	      num[(int)(data[y+1][x]==0)+(int)(data[y][x-1]==0)+2].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y][x-1]==0)+2;
	    }
	    else if(y-1<0) {
	      num[(int)(data[y+1][x]==0)+(int)(data[y][x+1]==0)+(int)(data[y][x-1]==0)+1].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y][x+1]==0)+(double)(data[y][x-1]==0)+1;
	    }
	    else if(y+1>height-1 & x-1<0) {
	      num[(int)(data[y-1][x]==0)+(int)(data[y][x+1]==0)+2].push_back(counter);
	      zeros[i][2] = (double)(data[y-1][x]==0)+(double)(data[y][x+1]==0)+2;
	    }
	    else if(y+1>height-1 & x+1>width-1) {
	      num[(int)(data[y-1][x]==0)+(int)(data[y][x-1]==0)+2].push_back(counter);
	      zeros[i][2] = (double)(data[y-1][x]==0)+(double)(data[y][x-1]==0)+2;
	    }
	    else if(y+1>height-1) {
	      num[(int)(data[y-1][x]==0)+(int)(data[y][x+1]==0)+(int)(data[y][x-1]==0)+1].push_back(counter);
	      zeros[i][2] = (double)(data[y-1][x]==0)+(double)(data[y][x+1]==0)+(double)(data[y][x-1]==0)+1;
	    }
	    else if(x-1<0) {
	      num[(int)(data[y+1][x]==0)+(int)(data[y-1][x]==0)+(int)(data[y][x+1]==0)+1].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y-1][x]==0)+(double)(data[y][x+1]==0)+1;
	    }
	    else if(x+1>width-1) {
	      num[(int)(data[y+1][x]==0)+(int)(data[y-1][x]==0)+(int)(data[y][x-1]==0)+1].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y-1][x]==0)+(double)(data[y][x-1]==0)+1;
	    }
	else {
	      num[(int)(data[y+1][x]==0)+(int)(data[y-1][x]==0)+(int)(data[y][x+1]==0)+(int)(data[y][x-1]==0)].push_back(counter);
	      zeros[i][2] = (double)(data[y+1][x]==0)+(double)(data[y-1][x]==0)+(double)(data[y][x+1]==0)+(double)(data[y][x-1]==0);
	}
	counter++;
      }
    }
    int i = (int)num.size()-1;
    while(i >=0)
    {
      if(num[i].empty())
	num.pop_back();
      else
	break;
      i--;
    }
  }
  cv::Mat1s newdepth_image(height,width);
  for(int v=0; v< height; v++)     //2-D indexing
  {
      for(int u=0; u< width; u++) {
          newdepth_image(v,u) = data[v][u];
	}
  }
  cv::Mat pgmdepth_image ( height, width, CV_16UC1 );
  newdepth_image.convertTo(pgmdepth_image, CV_16UC1);
  cv::imwrite("./../data/undistfiles/"+newfilename+"_depth.pgm",pgmdepth_image);
  std::cout<<"ready"<<std::endl;
}
