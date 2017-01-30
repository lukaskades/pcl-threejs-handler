#include "processing.hpp"

// Median filter
void medianFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, std::string method, int window_size, float max_allowed_movement)
{
  std::cout<<"\nfilter with method: " << method << "..." << std::endl;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  pcl::MedianFilter<pcl::PointXYZ> median;
  median.setInputCloud(pcptr);
  median.setWindowSize(window_size);
  median.setMaxAllowedMovement(max_allowed_movement); //10*calc_mean_min(pcptr)
  median.filter(*filtered);
  pcptr.swap(filtered);
  std::cout<<"...finished"<<std::endl;
  std::cout<<"number of entries of input cloud: " << countEntries(filtered) << std::endl;
  std::cout<<"number of entries of output cloud: " << countEntries(pcptr) << std::endl;
  std::cout<<"percentage of unchanged vertices: " << equals(filtered,pcptr)*100<<std::endl;
  std::cout<<"\nProceed with 'p' and Enter!\n";
  char c = 'a';
  while(c!='p') std::cin>>c;
}

// BilateralFilter
void bilateralFilter (pcl::PointCloud<pcl::PointXYZ>::Ptr &pcptr, std::string method, double sigmaS, double sigmaR)
{
  //-> Normal implementation of bilateral filter leads to no change of the point cloud
  /*pcl::PointCloud<pcl::PointXYZ>::Ptr pcptrcomp (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::PointCloud<pcl::PointXYZI>::Ptr pcptri (new pcl::PointCloud<pcl::PointXYZI>);

  pcptri->width = pcptr->width;  //Dimensions must be initialized to use 2-D indexing
  pcptri->height = pcptr->height;
  pcptri->resize(pcptri->width*pcptri->height);

  pcptrcomp->width = pcptr->width;  //Dimensions must be initialized to use 2-D indexing
  pcptrcomp->height = pcptr->height;
  pcptrcomp->resize(pcptrcomp->width*pcptrcomp->height);

  for(int v=0; v< pcptr->height; v++)     //2-D indexing
  {
      for(int u=0; u< pcptr->width; u++) {
	       //3-D position
	       (*pcptri)(u,v).x = (*pcptr)(u,v).x;
	       (*pcptri)(u,v).y = (*pcptr)(u,v).y;
	       (*pcptri)(u,v).z = (*pcptr)(u,v).z;
	       (*pcptrcomp)(u,v).x = (*pcptr)(u,v).x;
	       (*pcptrcomp)(u,v).y = (*pcptr)(u,v).y;
	       (*pcptrcomp)(u,v).z = (*pcptr)(u,v).z;
	       //Intensity
	       (*pcptri)(u,v).intensity = (*pcptr)(u,v).z;
      }
  }
  std::cout<<"\nfiltering with method: " << method << "..." << std::endl;
  pcl::PointCloud<pcl::PointXYZI>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZI>());
  pcl::BilateralFilter<pcl::PointXYZI> bilateral;
  bilateral.setInputCloud(pcptri);
  bilateral.setHalfSize(half_size);
  bilateral.setStdDev(dev_parameter);
  pcl::search::KdTree<pcl::PointXYZI>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZI>);
  tree->setInputCloud (pcptri);
  bilateral.setSearchMethod(tree);
  bilateral.filter(*filtered);
  pcptri.swap(filtered);
  std::cout<<"...finished"<<std::endl;
  for(int v=0; v< pcptr->height; v++)     //2-D indexing
  {
      for(int u=0; u< pcptr->width; u++) {
	       //3-D position
	       (*pcptr)(u,v).x = (*pcptri)(u,v).x;
	       (*pcptr)(u,v).y = (*pcptri)(u,v).y;
	       (*pcptr)(u,v).z = (*pcptri)(u,v).z;
      }
  }*/
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcptrcomp (new pcl::PointCloud<pcl::PointXYZ>);

  pcptrcomp->width = pcptr->width;  //Dimensions must be initialized to use 2-D indexing
  pcptrcomp->height = pcptr->height;
  pcptrcomp->resize(pcptrcomp->width*pcptrcomp->height);

  for(int v=0; v< pcptr->height; v++)     //2-D indexing
  {
      for(int u=0; u< pcptr->width; u++) {
	       //3-D position
	       (*pcptrcomp)(u,v).x = (*pcptr)(u,v).x;
	       (*pcptrcomp)(u,v).y = (*pcptr)(u,v).y;
	       (*pcptrcomp)(u,v).z = (*pcptr)(u,v).z;
      }
  }

  std::cout<<"\nfiltering with method: " << method << "..." << std::endl;
  pcl::FastBilateralFilter<pcl::PointXYZ> filter;
  pcl::PointCloud<pcl::PointXYZ>::Ptr filtered(new pcl::PointCloud<pcl::PointXYZ>());
  filter.setInputCloud(pcptr);
  // As SigmaS increases, larger features are smoothed out
  filter.setSigmaS(sigmaS);
  // As SigmaR increases, the bilateral filter becomes closer to a Gaussian blur
  filter.setSigmaR(sigmaR);
  filter.applyFilter(*filtered);
  pcptr.swap(filtered); 
  std::cout<<"...finished"<<std::endl;
  std::cout<<"number of entries of input cloud: " << countEntries(pcptrcomp) << std::endl;
  std::cout<<"number of entries of output cloud: " << countEntries(pcptr) << std::endl;
  std::cout<<"percentage of unchanged vertices: " << equals(pcptrcomp,pcptr)*100<<std::endl;
  std::cout<<"\nProceed with 'p' and Enter!\n";
  char c = 'a';
  while(c!='p') std::cin>>c;
}

// Normal estimation via least-square plane fitting
pcl::PointCloud<pcl::Normal>::Ptr leastsquareNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr, std::string method, int k, double radius, bool openmp)
{
  pcl::PointCloud<pcl::Normal>::Ptr normptr (new pcl::PointCloud<pcl::Normal>);
  if(openmp == false)
  {
    std::cout<<"\nnormal estimation with method: " << method << "..." << std::endl;
    pcl::NormalEstimation<pcl::PointXYZ, pcl::Normal> n;
    pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
    tree->setInputCloud (pcptr);
    n.setInputCloud (pcptr);
    n.setSearchMethod (tree);
    if(radius!=0) n.setRadiusSearch (radius);
    else n.setKSearch (k);
    n.setViewPoint(0, 0, 0);
    n.compute (*normptr);
  }
  else //with OpenMP
  {
    std::cout<<"\nnormal estimation with method: " << method << std::endl;
    pcl::NormalEstimationOMP<pcl::PointXYZ, pcl::Normal> n;
    n.setNumberOfThreads(8);
    n.setInputCloud (pcptr);
    if(radius!=0) n.setRadiusSearch (radius);
    else n.setKSearch (k);
    n.setViewPoint(0, 0, 0);
    n.compute (*normptr);
  }
  std::cout<<"...finished"<<std::endl;
  return normptr;
}

// Normal estimation via integral images
pcl::PointCloud<pcl::Normal>::Ptr integralimagesNormalEstimation(pcl::PointCloud<pcl::PointXYZ>::ConstPtr pcptr, std::string method, int art, float max_depth_change_factor, float normal_smoothing_size, bool depth_dependent_smoothing)
{
  std::cout<<"\nnormal estimation with method: " << method << "..." << std::endl;
  pcl::PointCloud<pcl::Normal>::Ptr normptr (new pcl::PointCloud<pcl::Normal>);
  pcl::IntegralImageNormalEstimation<pcl::PointXYZ, pcl::Normal> n;
  if(art == 0) n.setNormalEstimationMethod (n.COVARIANCE_MATRIX);
  else if(art == 1) n.setNormalEstimationMethod (n.AVERAGE_3D_GRADIENT);
  else if(art == 2) n.setNormalEstimationMethod (n.AVERAGE_DEPTH_CHANGE);
  else n.setNormalEstimationMethod (n.SIMPLE_3D_GRADIENT);
  n.setMaxDepthChangeFactor(max_depth_change_factor);
  n.setNormalSmoothingSize(normal_smoothing_size);
  n.setDepthDependentSmoothing(depth_dependent_smoothing);
  n.setInputCloud(pcptr);
  n.setViewPoint(0, 0, 0);
  n.compute(*normptr);
  std::cout<<"...finished"<<std::endl;
  return normptr;
}

// Generating PolygonMesh using GreedyProjectionTriangulation
pcl::PolygonMesh gptGenerateMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr pcnormptr, std::string method, double mu, double radius, int nnn, double minimum_angle, double maximum_angle, double eps_angle, bool consistent, bool consistent_ordering)
{
  std::cout<<"\ngenerating mesh with method: " << method << "..." << std::endl;
  pcl::PolygonMesh mesh;
  
  // Create search tree
  pcl::search::KdTree<pcl::PointNormal>::Ptr tree (new pcl::search::KdTree<pcl::PointNormal>);
  tree->setInputCloud (pcnormptr);

  // Initialize objects
  pcl::GreedyProjectionTriangulation<pcl::PointNormal> gp3;
  
  // Set parameters
  gp3.setMu (mu); // 2.5
  gp3.setSearchRadius(radius); // 2
  
  // Set typical parameters
  gp3.setMaximumNearestNeighbors(nnn); //100
  gp3.setMaximumSurfaceAngle(eps_angle); // M_Pi/4, 45 degrees
  gp3.setMinimumAngle(minimum_angle); // M_PI/18, 10 degrees
  gp3.setMaximumAngle(maximum_angle); // 2*M_PI/3, 120 degrees
  gp3.setNormalConsistency(consistent); // false
  gp3.setConsistentVertexOrdering(consistent_ordering); // false

  // Get result
  std::cout<<"\treconstruct..."<<std::endl;
  gp3.setInputCloud(pcnormptr);
  gp3.setSearchMethod(tree);
  gp3.reconstruct(mesh);
  
  // Additional vertex information
  //std::vector<int> parts = gp3.getPartIDs();
  //std::vector<int> states = gp3.getPointStates();
  
  std::cout<<"finished"<<std::endl;
  return mesh;
}

// Generating PolygonMesh using Poisson
pcl::PolygonMesh poissonGenerateMesh(pcl::PointCloud<pcl::PointNormal>::ConstPtr pcnormptr, std::string method, int depth, int min_depth, float point_weight, float scale, int solver_divide, int iso_divide, float samples_per_node, bool confidence, bool output_polygons, int degree, bool manifold)
{
  std::cout<<"\ngenerating mesh with method: " << method << "..." << std::endl;
  pcl::PolygonMesh mesh;

  pcl::Poisson<pcl::PointNormal> poisson;
  poisson.setDepth(depth);
  poisson.setMinDepth(min_depth);
  poisson.setPointWeight(point_weight);
  poisson.setScale(scale); // 1.1
  poisson.setSolverDivide(solver_divide); // 8
  poisson.setIsoDivide(iso_divide); // 8
  poisson.setSamplesPerNode(samples_per_node); // 2
  poisson.setConfidence(confidence); // 1
  poisson.setOutputPolygons(output_polygons); // 0
  poisson.setDegree(degree); // 5
  poisson.setManifold(manifold); // 0
  
  poisson.setInputCloud(pcnormptr);
  poisson.reconstruct(mesh);
  
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcptr (new pcl::PointCloud<pcl::PointXYZ>);
  copyPointCloud(*pcnormptr, *pcptr);
  pcl::search::KdTree<pcl::PointXYZ>::Ptr tree (new pcl::search::KdTree<pcl::PointXYZ>);
  tree->setInputCloud(pcptr);
  std::vector<int> k_all;
  pcl::PointCloud<pcl::PointXYZ>::Ptr pcmeshptr (new pcl::PointCloud<pcl::PointXYZ>);
  pcl::fromPCLPointCloud2(mesh.cloud, *pcmeshptr);
  for(int i = 0; i < pcmeshptr->points.size(); i++)
  {
     std::vector<int> k_indices;
     std::vector<float> k_sqr_distances;
     tree->nearestKSearch(pcmeshptr->points[i],1,k_indices,k_sqr_distances);
     k_all.push_back(k_indices[0]);
     //std::cout<<k_indices[0]<<": ("<<pcmeshptr->points[i].x<<", "<<pcmeshptr->points[i].y<<", "<<pcmeshptr->points[i].z<<") <-> ("<<pcptr->points[k_indices[0]].x<<", "<<pcptr->points[k_indices[0]].y<<", "<<pcptr->points[k_indices[0]].z<<")\t";
  }
  for(int i = 0; i < mesh.polygons.size(); i++)
  {
     pcl::Vertices v;
     v = mesh.polygons[i];
     for(int index = 0; index <v.vertices.size(); index++)
        v.vertices[index] = k_all[v.vertices[index]];
     mesh.polygons[i] = v;
  }
  pcl::toPCLPointCloud2(*pcptr, mesh.cloud); 
  std::cout<<"finished"<<std::endl;
  return mesh;
}
