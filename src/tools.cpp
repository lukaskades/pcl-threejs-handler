#include "tools.hpp"

//Counts how many points are actually in the PointCloud (without the ones with z=0)
int countEntries(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr)
{
    int counter = 0;
    for(size_t i = 0; i < pcptr->points.size(); i++)
        if(pcptr->points[i].z!=0)
            counter++;
    return counter;
}

int countEntries(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr pcptr)
{
    int counter = 0;
    for(size_t i = 0; i < pcptr->points.size(); i++)
        if(pcptr->points[i].z!=0)
            counter++;
    return counter;
}

// Determine the min and max values of x,y,z
void getSize(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr)
{
    Eigen::Vector4f min_pt;
    Eigen::Vector4f max_pt;
    pcl::getMinMax3D(*pcptr,min_pt,max_pt);
    std::cout<<"\tmin: " << min_pt[0] << ", " << min_pt[1] << ", " << min_pt[2] << std::endl; 
    std::cout<<"\tmax: " << max_pt[0] << ", " << max_pt[1] << ", " << max_pt[2] << std::endl;
}

// Crops a point cloud with center x,y and edge length dx,dy
void crop(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, float x, float y, float dx, float dy)
{
    std::cout<<"\ncrop cube with center x="<<x<<", y= "<<y<<", and edge lengths dx="<<dx<<", dy="<<dy<<std::endl;
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    pcl::getMinMax3D(*pcptr,minPoint,maxPoint);
    minPoint[0]=x-dx/2.0;
    minPoint[1]=y-dy/2.0;
    minPoint[3]=0;
    maxPoint[0]=x+dx/2.0;
    maxPoint[1]=y+dy/2.0;
    maxPoint[3]=0;
    
    //Eigen::Vector3f boxTranslation(x,y,z);
    
    pcl::PointCloud<pcl::PointXYZ>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZ>());
    pcl::CropBox<pcl::PointXYZ> cropp;
    cropp.setInputCloud(pcptr);
    cropp.setMin(minPoint);
    cropp.setMax(maxPoint);
    //cropp.setTranslation(boxTranslation);
    cropp.setKeepOrganized (true);
    std::vector<int> indices;
    cropp.filter(indices);
    
    int uu_max = 0;
    int vv_max = 0;
    int uu_min = pcptr->points.size();
    int vv_min = pcptr->points.size();
    for(int i = 0; i < indices.size(); i++)
    {
      int uu = indices[i]%pcptr->width;
      int vv = (int)((indices[i]-indices[i]%pcptr->width)/pcptr->width);
      if(uu<=uu_min)
	uu_min=uu;
      if(vv<=vv_min)
	vv_min=vv;
      if(uu>=uu_max)
	uu_max=uu;
      if(vv>=vv_max)
	vv_max=vv;
    }
    
    cropped->width = uu_max-uu_min+1;
    cropped->height = vv_max-vv_min+1;
    cropped->resize(cropped->width*cropped->height);
    
    for(int u = uu_min; u<=uu_max; u++)
      for(int v = vv_min; v<=vv_max; v++)
	//if((z-dz/2.0<(*pcptr)(u,v).z)&&(z+dz/2.0>(*pcptr)(u,v).z))
	{
	  (*cropped)(u-uu_min,v-vv_min).x = (*pcptr)(u,v).x;
	  (*cropped)(u-uu_min,v-vv_min).y = (*pcptr)(u,v).y;
	  (*cropped)(u-uu_min,v-vv_min).z = (*pcptr)(u,v).z;
	}
    
    for(size_t i = 0; i< cropped->points.size();i++)
      //if x = inf or nan
      if(cropped->points[i].x!=cropped->points[i].x)
      {
	cropped->points[i].x = 0;
	cropped->points[i].y = 0;
	cropped->points[i].z = 0;
      }
    pcptr.swap(cropped);
    std::cout<<"...cube generated and returned"<<std::endl;
}

// Crops a colored point cloud with center x,y and edge length dx,dy
void crop(pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcrgbptr, float x, float y, float dx, float dy)
{
    std::cout<<"\ncrop cube with center x="<<x<<", y= "<<y<<", and edge lengths dx="<<dx<<", dy="<<dy<<std::endl;
    Eigen::Vector4f minPoint;
    Eigen::Vector4f maxPoint;
    pcl::getMinMax3D(*pcrgbptr,minPoint,maxPoint);
    minPoint[0]=x-dx/2.0;
    minPoint[1]=y-dy/2.0;
    minPoint[3]=0;
    maxPoint[0]=x+dx/2.0;
    maxPoint[1]=y+dy/2.0;
    maxPoint[3]=0;
    
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cropped(new pcl::PointCloud<pcl::PointXYZRGB>());
    pcl::CropBox<pcl::PointXYZRGB> cropp;
    cropp.setInputCloud(pcrgbptr);
    cropp.setMin(minPoint);
    cropp.setMax(maxPoint);
    //cropp.setTranslation(boxTranslation);
    cropp.setKeepOrganized (true);
    std::vector<int> indices;
    cropp.filter(indices);
    
    int uu_max = 0;
    int vv_max = 0;
    int uu_min = pcrgbptr->points.size();
    int vv_min = pcrgbptr->points.size();
    for(int i = 0; i < indices.size(); i++)
    {
      int uu = indices[i]%pcrgbptr->width;
      int vv = (int)((indices[i]-indices[i]%pcrgbptr->width)/pcrgbptr->width);
      if(uu<=uu_min)
	uu_min=uu;
      if(vv<=vv_min)
	vv_min=vv;
      if(uu>=uu_max)
	uu_max=uu;
      if(vv>=vv_max)
	vv_max=vv;
    }
    
    cropped->width = uu_max-uu_min+1;
    cropped->height = vv_max-vv_min+1;
    cropped->resize(cropped->width*cropped->height);
    
    for(int u = uu_min; u<=uu_max; u++)
      for(int v = vv_min; v<=vv_max; v++)
	//if((z-dz/2.0<(*pcrgbptr)(u,v).z)&&(z+dz/2.0>(*pcrgbptr)(u,v).z))
	{
	  (*cropped)(u-uu_min,v-vv_min).x = (*pcrgbptr)(u,v).x;
	  (*cropped)(u-uu_min,v-vv_min).y = (*pcrgbptr)(u,v).y;
	  (*cropped)(u-uu_min,v-vv_min).z = (*pcrgbptr)(u,v).z;
	  (*cropped)(u-uu_min,v-vv_min).r = (*pcrgbptr)(u,v).r;
	  (*cropped)(u-uu_min,v-vv_min).g = (*pcrgbptr)(u,v).g;
	  (*cropped)(u-uu_min,v-vv_min).b = (*pcrgbptr)(u,v).b;
	}
    
    for(size_t i = 0; i< cropped->points.size();i++)
      //if x = inf or nan
      if(cropped->points[i].x!=cropped->points[i].x)
      {
	cropped->points[i].x = 0;
	cropped->points[i].y = 0;
	cropped->points[i].z = 0;
      }
    pcrgbptr.swap(cropped);
    std::cout<<"...cube generated and returned"<<std::endl;
}

// Function for finding the right parameters for cropping
void cropAndShow(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr,pcl::PointCloud<pcl::PointXYZRGB>::Ptr &pcrgbptr)
{
    float x,y,dx,dy;
    x= 0;
    int done = 0;
    std::cout<<"\n#### Crop a cube with center x,y and edge length dx, dy and see the result ####\n\n\tdimension of the cloud:"<<std::endl;
getSize(pcptr);
std::cout<<"\tnumber of vertices of the cloud= " <<countEntries(pcptr)<<std::endl;
    while(done==0)
    {

        viewCloud(pcptr);
        std::cout<<"\n\tentry center of the desired cube x, y (for example: 0.25 0.1):";
        std::cin>>x>>y;
	std::cout<<"\tenter edge lengths of the desired cube dx, dy (for example 0.6 0.8):";
	std::cin>>dx>>dy;
        pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloudrgb (new pcl::PointCloud<pcl::PointXYZRGB>);
        
        cloudrgb->width = pcptr->width;  //Dimensions must be initialized to use 2-D indexing
        cloudrgb->height = pcptr->height;
        cloudrgb->resize(cloudrgb->width*cloudrgb->height);
        
        int counter = 0;
        for(int v=0; v< cloudrgb->height; v++)     //2-D indexing
        {
            for(int u=0; u< cloudrgb->width; u++)
            {
                (*cloudrgb)(u,v).x = (*pcptr)(u,v).x;
                (*cloudrgb)(u,v).y = (*pcptr)(u,v).y;
                (*cloudrgb)(u,v).z = (*pcptr)(u,v).z;
                float xx = (*cloudrgb)(u,v).x;
                float yy = (*cloudrgb)(u,v).y;
                float zz = (*cloudrgb)(u,v).z;
                if((x-dx/2.0<xx)&&(x+dx/2.0>xx)&&(y-dy/2.0<yy)&&(y+dy/2.0>yy))
                {
                    counter++;
                    (*cloudrgb)(u,v).b = 0;
                    (*cloudrgb)(u,v).g = 0;
                    (*cloudrgb)(u,v).r = 255;
                }
                else
                {
                    (*cloudrgb)(u,v).b = 255;
                    (*cloudrgb)(u,v).g = 255;
                    (*cloudrgb)(u,v).r = 255;
                }
            }
        }
        std::cout << "\tnumber of verties of the cube = " << counter;
        viewCloud(cloudrgb);
	
        std::cout << "\n\tcrop again with new parameters [a], crop cloud and exit crop menu[c], exit crop menu without cropping [e]: ";
        char answer;
        std::cin>>answer;
	if(answer=='c') {
	    done = 1;
            crop(pcrgbptr,x,y,dx,dy);
  	    crop(pcptr,x,y,dx,dy);
	}
	else if(answer=='a') {}
        else
	    done = 1;
    }
    std::cout<<"\n##################################### DONE ####################################"<<std::endl;
}

// Returns the fraction of points with the same coordinates in two pointclouds
double equals(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr1, pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr2)
{
    int width = pcptr1->width;
    int height = pcptr1->height;
    double frac = 0;
    for(int v=0; v< height; v++)     //2-D indexing
        for(int u=0; u< width; u++)
            if(((*pcptr1)(u,v).x==(*pcptr2)(u,v).x)&&((*pcptr1)(u,v).y==(*pcptr2)(u,v).y)&&((*pcptr1)(u,v).z==(*pcptr2)(u,v).z)&&((*pcptr1)(u,v).z!=0))
                frac++;
    frac = frac/countEntries(pcptr1);
    return frac;
}

// Calculates the center of the point cloud using only the points with z!=0
void center(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, Eigen::Vector3f &centroid)
{
    centroid[0]=0;
    centroid[1]=0;
    centroid[2]=0;
    for(int i = 0; i< pcptr->points.size(); i++)
        if(pcptr->points[i].z!=0)
        {
            centroid[0]=centroid[0]+pcptr->points[i].x;
            centroid[1]=centroid[1]+pcptr->points[i].y;
            centroid[2]=centroid[2]+pcptr->points[i].z;
        }
    centroid[0]=centroid[0]/countEntries(pcptr);
    centroid[1]=centroid[1]/countEntries(pcptr);
    centroid[2]=centroid[2]/countEntries(pcptr);
}

// Calculates the mean minimum distance and its standard deviation between the points and their k-nearest neighbours
double calcMeanMin(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr,int k)
{
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pcptr);
    double mean_min = 0;
    std::vector<std::vector<float> > k_sqr_all;
    for(int i = 0; i < pcptr->points.size(); i++)
        if(pcptr->points[i].z!=0)
        {
            std::vector<int> k_indices;
            std::vector<float> k_sqr_distances;
            tree->nearestKSearch(pcptr->points[i],k,k_indices,k_sqr_distances);
            for(unsigned j = 1; j<k_sqr_distances.size(); j++)
                mean_min=mean_min+sqrt(k_sqr_distances[j]);
            k_sqr_all.push_back(k_sqr_distances);
        }
    mean_min=mean_min/((k-1)*k_sqr_all.size());
    double stdev_min = 0;
    for(unsigned i = 0; i<k_sqr_all.size(); i++)
        for(unsigned j = 1; j<k; j++)
            stdev_min=stdev_min+(sqrt(k_sqr_all[i][j])-mean_min)*(sqrt(k_sqr_all[i][j])-mean_min);
    stdev_min=sqrt(stdev_min/((k-1)*k_sqr_all.size()-1));
    return mean_min;
}

void info(pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr)
{
  std::cout<< "\nsize:" << pcptr->size() << " = " << pcptr->width << "x" << pcptr->height << "\nnumber of entries with depthdata: " << countEntries(pcptr) << std::endl;
  std::cout<< "min and max values of x,y,z in the point cloud:" << std::endl;
  getSize(pcptr);
  Eigen::Vector3f centroid;
  center(pcptr,centroid);
  std::cout << "center ignoring z!=0 points: " << centroid[0] << ", " << centroid[1] << ", " << centroid[2] << std::endl;
  Eigen::Vector4f centroidall;
  pcl::compute3DCentroid(*pcptr,centroidall);
  std::cout << "center computed with pcl function: " << centroidall[0] << ", " << centroidall[1] << ", " << centroidall[2] << std::endl;
  std::cout << "minimum mean distance (to k=5 Neighbors): " << calcMeanMin(pcptr,5) << std::endl;
}
