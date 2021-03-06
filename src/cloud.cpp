#include <cloud.hpp>

Cloud::Cloud(std::string name_, std::string newname_, int scale_)
{
  name = name_;
  newname = newname_;
  scale = scale_;
  this->createCloud();
  if(scale!=-1) {
    createPointCloud(this->pcptr,this->name,this->scale);
    this->pcrgbptr = colorPointCloud(this->pcptr,this->name);
    meshmethod = "none";
    normalmethod = "none";
    filtermethod = "none";
    texmeshstatus = "none";
    croppedstatus = "no";
  }
  else {
    this->texmeshstatus = "none";
    pcl::PointCloud<pcl::PointXYZRGB>::Ptr cpcrgbptr (new pcl::PointCloud<pcl::PointXYZRGB>);
    this->pcrgbptr.swap(cpcrgbptr);
    //pcl::PointCloud<pcl::PointNormal>::Ptr cpcnormptr (new pcl::PointCloud<pcl::PointNormal>);
    //this->pcnormptr.swap(cpcnormptr);
    load(this->name, this->pcptr, this->pcrgbptr, this->pcnormptr, this->mesh);
    if(mesh.cloud.data.size()!=0)
      this->meshmethod = "unknown";
    else meshmethod = "none";
    if(this->pcnormptr->empty()==0) {
      normalmethod = "unknown";
      pcl::PointCloud<pcl::Normal>::Ptr cnormptr (new pcl::PointCloud<pcl::Normal>);
      cnormptr->width = this->pcnormptr->width;  //Dimensions must be initialized to use 2-D indexing
      cnormptr->height = this->pcnormptr->height;
      cnormptr->resize(cnormptr->width*cnormptr->height);
      for(int v=0; v< cnormptr->height; v++)     //2-D indexing
      {
	for(int u=0; u< cnormptr->width; u++)
	{
	  (*cnormptr)(u,v).normal_x = (*(this->pcnormptr))(u,v).normal_x;
	  (*cnormptr)(u,v).normal_y = (*(this->pcnormptr))(u,v).normal_y;
	  (*cnormptr)(u,v).normal_z = (*(this->pcnormptr))(u,v).normal_z;
	}
      }
      this->normptr.swap(cnormptr);
    }
    else normalmethod = "none";
    filtermethod = "unknown";
    croppedstatus = "unknown";
  }
}

void Cloud::createCloud() {
  pcl::PointCloud<pcl::PointXYZ>::Ptr cpcptr (new pcl::PointCloud<pcl::PointXYZ>);
  this->pcptr.swap(cpcptr);
  pcl::PointCloud<pcl::PointNormal>::Ptr cpcnormptr (new pcl::PointCloud<pcl::PointNormal>);
  this->pcnormptr.swap(cpcnormptr);
}

void Cloud::visualize() {
  std::cout<<"\tVisualize:\n\t\tuncolored cloud[0]\n\t\tcolored cloud[1]\n\t\tcloud with normals[2]\n\t\tmesh[3]\n\t\tmesh with texture[4]\n\t\tgo back[5]\n\n###>";
  int art;
  std::cin>>art;
  if(art==0 & this->pcptr->empty()==0)
    viewCloud(this->pcptr);
  else if(art==1 & this->pcrgbptr->empty()==0)
    viewCloud(this->pcrgbptr);
  else if(art==2 & this->pcrgbptr->empty()==0 & this->normptr!=NULL)
    viewCloud(this->pcrgbptr,this->normptr);
  else if(art==3 & meshmethod!="none")//this->mesh.cloud.data.size()!=0)
    viewCloud(this->mesh);
  else if(art==4) {
    if(this->texmeshstatus == "exists")
      viewCloud(this->texMesh);
    else {
      std::cout<<"Error: Texture image missing (Only non-cropped clouds can be visualized, yet). Back with 'b' and Enter!\n";
      char c = 'a';
      while(c!='b') std::cin>>c;
    }
  }
  else if(art<5) {
    std::cout<<"Error: Can't visualize, object not created, yet. Back with 'b' and Enter!\n";
    char c = 'a';
    while(c!='b') std::cin>>c;
  }
}

void Cloud::filterCloud() {
  if(this->normptr!=NULL) {
    std::cout<<"\nAlready processed objects (normals, mesh, etc.) are deleted. Proceed?[y/n]\n\n###>";
    char proceed;
    std::cin>>proceed;
    switch(proceed) {
      case 'y': {
	//this->pcrgbptr.reset();
	this->normalmethod = "none";
	this->normptr.reset();
	this->meshmethod = "none";
	this->pcnormptr.reset();
	this->texmeshstatus = "none";
	this->filterCloud();
        pcl::PointCloud<pcl::PointNormal>::Ptr cpcnormptr (new pcl::PointCloud<pcl::PointNormal>);
    	this->pcnormptr.swap(cpcnormptr);
	break;
      }
    }
  }
  else {
    std::cout<<"\tFilter:\n\t\tmedian filter[0]\n\t\tbilateral filter[1]\n\t\tgo back[2]\n\n###>";
    int art;
    std::cin>>art;
    switch(art) {
      case 0: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
          int window_size;
          float max_allowed_movement;
	  std::cout<<"window size (default = 10) = ";
	  std::cin>>window_size;
	  std::cout<<"max allowed movement (default = 0.5) = ";
	  std::cin>>max_allowed_movement;
	  medianFilter(this->pcptr,"median filter", (int)window_size, (float)max_allowed_movement);
	}
	else 
	  medianFilter(this->pcptr,"median filter", 10, 0.5);
	this->filtermethod = "median";
	break;
      }
      case 1: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
          double half_size, dev_parameter;
	  std::cout<<"sigmaS (default = 1) = ";
	  std::cin>>half_size;
	  std::cout<<"sigmaR (default = 1) = ";
	  std::cin>>dev_parameter;
	  bilateralFilter(this->pcptr,"bilateral filter", (double)half_size, (double)dev_parameter);
	}
	else
	  bilateralFilter(this->pcptr,"bilateral filter", 1, 1);
	this->filtermethod = "bilateral";
	break;
      }
    }
    for(int i = 0; i< this->pcptr->points.size(); i++) {
       this->pcrgbptr->points[i].x = this->pcptr->points[i].x;
       this->pcrgbptr->points[i].y = this->pcptr->points[i].y;
       this->pcrgbptr->points[i].z = this->pcptr->points[i].z;
    }
  }
}

void Cloud::generateNormals() {
  if(this->normptr==NULL) {
    std::cout<<"\nGenerate Normals, choose method:\n\t\tleast square fitting[0]\n\t\tleast square fitting with OpenMP[1]\n\t\tintegral images - covariance matrix[2]\n\t\tintegral images - average gradient[3]\n\t\tintegral images - average depth change[4]\n\t\tintegral images - simple 3D gradient[5]\n\t\tdefault[6]\n\t\tgo back[7]\n\n###>";
    int art;
    std::cin>>art;
    switch(art) {
      case 0: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  int k;
	  std::cout<<"k (default = 100) = ";
	  std::cin>>k;
	  double radius;
	  std::cout<<"radius (default 0) = ";
	  std::cin>>radius;
	  this->normptr = leastsquareNormalEstimation(this->pcptr, "least square fitting", (int)k,(double)radius,false);
	}
	else
	  this->normptr = leastsquareNormalEstimation(this->pcptr, "least square fitting", 100,0,false);
	this->normalmethod = "least square fitting";
	break;
      }
      case 1: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  int k;
	  std::cout<<"k (default = 100) = ";
	  std::cin>>k;
	  double radius;
	  std::cout<<"radius (default 0.0) = ";
	  std::cin>>radius;
	  this->normptr = leastsquareNormalEstimation(this->pcptr, "least square fitting with OpenMP", (int)k,(double)radius,true);
	}
	else
	  this->normptr = leastsquareNormalEstimation(this->pcptr, "least square fitting with OpenMP", 100,0,true);
	this->normalmethod = "least square fitting with OpenMP";
	break;
      }
      case 2: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  float max_depth_change_factor;
	  std::cout<<"max depth change factor (default = 0.02) = ";
	  std::cin>>max_depth_change_factor;
	  float normal_smoothing_size;
	  std::cout<<"normal smoothing size (default 4.0) = ";
	  std::cin>>normal_smoothing_size;
	  bool depth_dependent_smoothing;
	  std::cout<<"depth dependent smoothing (default = true)[true/false] = ";
	  std::cin>>depth_dependent_smoothing;
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - covariance matrix",0, (float)max_depth_change_factor, (float)normal_smoothing_size, (bool)depth_dependent_smoothing);
	}
	else
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - covariance matrix",0, 0.02f, 4.0f, true);
	this->normalmethod = "integral images - covariance matrix";
	break;
      }
      case 3: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  float max_depth_change_factor;
	  std::cout<<"max depth change factor (default = 0.02) = ";
	  std::cin>>max_depth_change_factor;
	  float normal_smoothing_size;
	  std::cout<<"normal smoothing size (default 4.0) = ";
	  std::cin>>normal_smoothing_size;
	  bool depth_dependent_smoothing;
	  std::cout<<"depth dependent smoothing (default = true)[true/false] = ";
	  std::cin>>depth_dependent_smoothing;
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - average gradient",1, (float)max_depth_change_factor, (float)normal_smoothing_size, (bool)depth_dependent_smoothing);
	}
	else
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - average gradient",1, 0.02f, 4.0f, true);
	this->normalmethod = "integral images - average gradient";
	break;
      }
      case 4: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  float max_depth_change_factor;
	  std::cout<<"max depth change factor (default = 0.02) = ";
	  std::cin>>max_depth_change_factor;
	  float normal_smoothing_size;
	  std::cout<<"normal smoothing size (default 4.0) = ";
	  std::cin>>normal_smoothing_size;
	  bool depth_dependent_smoothing;
	  std::cout<<"depth dependent smoothing (default = true)[true/false] = ";
	  std::cin>>depth_dependent_smoothing;
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - average depth change",2, (float)max_depth_change_factor, (float)normal_smoothing_size, (bool)depth_dependent_smoothing);
	}
	else
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - average depth change",2, 0.02f, 4.0f, true);
	this->normalmethod = "integral images - average depth change";
	break;
      }
      case 5: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter parameters[0]\n\t\tdefault[1]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  float max_depth_change_factor;
	  std::cout<<"max depth change factor (default = 0.02) = ";
	  std::cin>>max_depth_change_factor;
	  float normal_smoothing_size;
	  std::cout<<"normal smoothing size (default 4.0) = ";
	  std::cin>>normal_smoothing_size;
	  bool depth_dependent_smoothing;
	  std::cout<<"depth dependent smoothing (default = true)[true/false] = ";
	  std::cin>>depth_dependent_smoothing;
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - simple 3D gradient",3, (float)max_depth_change_factor, (float)normal_smoothing_size, (bool)depth_dependent_smoothing);
	}
	else
	  this->normptr = integralimagesNormalEstimation(this->pcptr, "integral images - simple 3D gradient",3, 0.02f, 4.0f, true);
	this->normalmethod = "integral images - simple 3D gradient";
	break;
      }
      case 6: {
	this->normptr = leastsquareNormalEstimation(this->pcptr, "least square fitting", 100,0,false);
	this->normalmethod = "least square fitting";
	break;
      }
    }
    if(art<7) {
      pcl::PointCloud<pcl::PointNormal>::Ptr cpcnormptr (new pcl::PointCloud<pcl::PointNormal>);
      this->pcnormptr.swap(cpcnormptr);
      pcl::concatenateFields (*(this->pcptr), *(this->normptr), *(this->pcnormptr));
    }
  }
  else {
    std::cout<<"\nNormals were already computed. Compute new normals, already processed objects (mesh, etc.) could be deleted?[y/n]\n\n###>";
    char proceed;
    std::cin>>proceed;
    switch(proceed) {
      case 'y': {
	this->normalmethod = "none";
	this->normptr.reset();
	this->meshmethod = "none";
	this->pcnormptr.reset();
	this->texmeshstatus = "none";
	this->generateNormals();
	break;
      }
    }
  }
}

void Cloud::constructMesh() {
  if(this->meshmethod=="none" & this->pcnormptr->empty()==0) {
    std::cout<<"\nGenerate Mesh, choose method:\n\t\tgreedy projection triangulation[0]\n\t\tpoisson reconstruction[1]\n\t\tdefault[2]\n\t\tgo back[3]\n\n###>";
    int art;
    std::cin>>art;
    switch(art) {
      case 0: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter only important parameters[0]\n\t\tenter all parameters[1]\n\t\tdefault[2]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  double mu, radius, minimum_angle, maximum_angle, eps_angle;
	  bool consistent, consistent_ordering;
	  int nnn;
	  std::cout<<"mu (default = 2.5) = ";
	  std::cin>>mu;
	  std::cout<<"radius (default = 2) = ";
	  std::cin>>radius;
	  this->mesh = gptGenerateMesh(this->pcnormptr, "greedy projection triangulation", (double)mu, (double)radius);
	}
	else if(para==1)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  double mu, radius, minimum_angle, maximum_angle, eps_angle;
	  bool consistent, consistent_ordering;
	  int nnn;
	  std::cout<<"mu (default = 2.5) = ";
	  std::cin>>mu;
	  std::cout<<"radius (default = 2) = ";
	  std::cin>>radius;
	  std::cout<<"nnn (default = 100) = ";
	  std::cin>>nnn;
	  std::cout<<"eps angle (default = M_PI/4) = ";
	  std::cin>>eps_angle;
	  std::cout<<"minimum angle (default = M_PI/18) = ";
	  std::cin>>minimum_angle;
	  std::cout<<"maximum angle (default = 2*M_PI/3) = ";
	  std::cin>>maximum_angle;
	  std::cout<<"consistent (default = false)[true/false] = ";
	  std::cin>>consistent;
	  std::cout<<"consistent ordering (default = false)[true/false] = ";
	  std::cin>>consistent_ordering;
	  this->mesh = gptGenerateMesh(this->pcnormptr, "greedy projection triangulation", (double)mu, (double)radius, (int)nnn, (double)minimum_angle, (double)maximum_angle, (double)eps_angle, (bool)consistent, (bool)consistent_ordering);
	}
	else
	  this->mesh = gptGenerateMesh(this->pcnormptr, "greedy projection triangulation");
	this->meshmethod = "greedy projection triangulation";
	break;
      }
      case 1: {
	std::cout<<std::string(25,'\n');
	std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	std::cout<<"\nparameters:\n\t\tenter only important parameters[0]\n\t\tenter all parameters[1]\n\t\tdefault[2]\n\n###>";
	int para;
	std::cin>>para;
	if(para==0)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  int depth, min_depth, solver_divide, iso_divide, degree; 
	  float point_weight, scale, samples_per_node;
	  bool confidence, output_polygons, manifold;
	  std::cout<<"depth (default = 8) = ";
	  std::cin>>depth;
	  std::cout<<"min depth (default = 2) = ";
	  std::cin>>min_depth;
	  std::cout<<"point weight (default = 10.0) = ";
	  std::cin>>point_weight;
	  this->mesh = poissonGenerateMesh(this->pcnormptr, "poisson reconstruction", (int)depth, (int)min_depth, (float)point_weight);
	}
	else if(para==1)
	{
	  std::cout<<std::string(25,'\n');
	  std::cout<<"%%%%%%%%%%%%%%%%%%%%%%%%%%%% PCL Processing Programm %%%%%%%%%%%%%%%%%%%%%%%%%%%\n"<<std::endl;
	  int depth, min_depth, solver_divide, iso_divide, degree; 
	  float point_weight, scale, samples_per_node;
	  bool confidence, output_polygons = true, manifold;
	  std::cout<<"depth (default = 8) = ";
	  std::cin>>depth;
	  std::cout<<"min depth (default = 2) = ";
	  std::cin>>min_depth;
	  std::cout<<"point weight (default = 10.0) = ";
	  std::cin>>point_weight;
	  std::cout<<"scale (default = 1.1) = ";
	  std::cin>>scale;
	  std::cout<<"solver divide (default = 8) = ";
	  std::cin>>solver_divide;
	  std::cout<<"iso divide (default = 8) = ";
	  std::cin>>iso_divide;
	  std::cout<<"samples per node (default = 2) = ";
	  std::cin>>samples_per_node;
	  std::cout<<"confidence (default = true)[true/false] = ";
	  std::cin>>confidence;
	  std::cout<<"degree (default = 2) = ";
	  std::cin>>degree;
	  std::cout<<"manifold (default = false)[true/false] = ";
	  std::cin>>manifold;
	  this->mesh = poissonGenerateMesh(this->pcnormptr, "poisson reconstruction", (int)depth, (int)min_depth, (float)point_weight, (float)scale, (int)solver_divide, (int)iso_divide, (float)samples_per_node, (bool)confidence, (bool)output_polygons, (int)degree, (bool)manifold);
	}
	else
	  this->mesh = poissonGenerateMesh(this->pcnormptr, "poisson reconstruction",8,2,10.0);
	this->meshmethod = "poisson reconstruction";
	break;	
      }
      case 2: {
	this->mesh = gptGenerateMesh(this->pcnormptr, "greedy projection triangulation");
	this->meshmethod = "greedy projection triangulation";
	break;
      }
    }
    if(this->mesh.polygons.size()==0) {
       this->meshmethod = "none";
       this->texmeshstatus = "none";
       std::cout<<"\nError: Computed normals have to many undefined normals for generating a mesh. Try to use another normal estimation method. Back with 'b' and Enter!\n";
       char c = 'a';
       while(c!='b') std::cin>>c;
    }
    else {
       std::ifstream FileTest(("./../data/undistfiles/"+this->name+"_rgb.ppm").c_str());
       if(FileTest) {
          if(this->croppedstatus!="yes") {
	     this->texMesh = textureMesh(this->mesh, this->name, this->scale);
	     this->texmeshstatus = "exists";
          }
       }
    }
  }
  else if(this->meshmethod == "none" & this->pcnormptr->empty()!=0) {
    std::cout<<"\nCannot construct mesh. Normals have to be computed in advance. Compute normals?[y/n]\n\n###>";
    char proceed;
    std::cin>>proceed;
    switch(proceed) {
      case 'y': {
	this->generateNormals();
	break;
      }
    }
  }
  else {
    std::cout<<"\nMesh was already computed. Compute new mesh, already processed objects (mesh, etc.) could be deleted?[y/n]\n\n###>";
    char proceed;
    std::cin>>proceed;
    switch(proceed) {
      case 'y': {
	this->meshmethod = "none";
	this->texmeshstatus = "none";
	this->constructMesh();
	break;
      }
    }
  }
}

void Cloud::cropCloud() {
  int siz =  this->pcptr->points.size();
  cropAndShow(this->pcptr, this->pcrgbptr);
  if(siz!= this->pcptr->points.size()) {
    this->filtermethod = "none";
    this->normalmethod = "none";
    this->normptr.reset();
    this->meshmethod = "none";
    this->pcnormptr.reset();
    this->texmeshstatus = "none";
    this->croppedstatus = "yes";
  }
}

void Cloud::getinfo() {
  std::cout<<"\n########################### INFORMATION TO POINTCLOUD ##########################"<<std::endl;
  std::cout<<"\nFilter: " << this->filtermethod<<std::endl;
  std::cout<<"Normals: " << this->normalmethod<<std::endl;
  std::cout<<"Mesh: " << this->meshmethod<<std::endl;
  std::cout<<"Texture Mesh: " << this->texmeshstatus<<std::endl;
  std::cout<<"Cropped Cloud: " << this->croppedstatus<<std::endl;
  std::cout<<"Cloud Data:";
  info(this->pcptr);
  std::cout<<"\n################################ INFORMATION END ###############################"<<std::endl;
  std::cout<<"\nBack with 'b' and Enter!\n";
  char c = 'a';
  while(c!='b') std::cin>>c;
}

void Cloud::saveCloud() {
  if(meshmethod=="none") {
    pcl::PolygonMesh m;
    save(this->newname, this->pcptr, this->pcrgbptr, this->pcnormptr, m);
  }
  else 
    save(this->newname, this->pcptr, this->pcrgbptr, this->pcnormptr, this->mesh);
  std::cout<<"\nSaving successful. Back with 'b' and Enter!\n";
  char c = 'a';
  while(c!='b') std::cin>>c;
}

void Cloud::buildthreejsdatafromCloud() {
  std::ifstream FileTest(("./../data/"+this->newname+".pcl").c_str());
  if(FileTest)
     buildthreejsdata(this->newname);
  else {
    std::cout<<"\nCloud has to be saved before being able to build the three.js files. Back with 'b' and Enter!\n";
    char c = 'a';
    while(c!='b') std::cin>>c;
  }
}

bool Cloud::getstatusofzeros() {
  if(countEntries(this->pcptr)!=this->pcptr->points.size())
    return true;
  else
    return false;
}
