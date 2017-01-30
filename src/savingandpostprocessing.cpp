#include "savingandpostprocessing.hpp"

std::string to_string(int num) {
    std::stringstream ss;
    ss << num;
    return ss.str();
}

int stoi(std::string text) {
    return atoi(text.c_str());
}

void save(std::string filename, pcl::PointCloud<pcl::PointXYZ>::ConstPtr xyz, pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr xyzrgb, pcl::PointCloud<pcl::PointNormal>::Ptr xyznormal, pcl::PolygonMesh &mesh)
{
  std::ostringstream wi;
  wi << xyz->width;
  std::string strwi = wi.str();
  std::ostringstream he;
  he << xyz->height;
  std::string strhe = he.str();

  std::ifstream is;
  std::vector<std::string> files;
  std::string line;
  int counter = 0;
  is.open(("./../data/"+filename+".pcl").c_str());
  std::ofstream os;
  os.open(("./../data/"+filename+".pcl").c_str());
  
  if(xyz->empty()==0) {
    pcl::io::savePLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_xyz.ply", *xyz);
    os << filename+"_"+strwi+"x"+strhe+"_xyz.ply" <<std::endl;
  }
  if(xyzrgb->empty()==0) {
    pcl::io::savePLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_rgb.ply", *xyzrgb);
    os << filename+"_"+strwi+"x"+strhe+"_rgb.ply" <<std::endl;
  }
  if(xyznormal->empty()==0) {
    for(int i = 0; i< xyznormal->points.size(); i++) {
       if(xyznormal->points[i].normal_x!=xyznormal->points[i].normal_x | xyznormal->points[i].normal_y!=xyznormal->points[i].normal_y | xyznormal->points[i].normal_z!=xyznormal->points[i].normal_z) {
          xyznormal->points[i].normal_x = 0;
          xyznormal->points[i].normal_y = 0;
          xyznormal->points[i].normal_z = 0;
       }
       if(xyznormal->points[i].curvature!=xyznormal->points[i].curvature)
          xyznormal->points[i].curvature = 0;
    }
    pcl::io::savePLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_normals.ply", *xyznormal);
    os << filename+"_"+strwi+"x"+strhe+"_normals.ply" <<std::endl;
  }
  if(mesh.cloud.data.size()!=0) {
    pcl::io::savePLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_mesh.ply", mesh);
    os << filename+"_"+strwi+"x"+strhe+"_mesh.ply" <<std::endl;
  }
  os.close();
}

void load(std::string filename, pcl::PointCloud<pcl::PointXYZ>::Ptr xyz, pcl::PointCloud<pcl::PointXYZRGB>::Ptr xyzrgb, pcl::PointCloud<pcl::PointNormal>::Ptr xyznormal, pcl::PolygonMesh &mesh)
{
  std::ifstream is;
  is.open(("./../data/"+filename+".pcl").c_str());
  std::string line;
  std::vector<std::string> files;
  while (!is.eof()) {
      getline (is,line);
      files.push_back(line);
  }
  char key;
  int counter = (int)files[0].size()-8;
  while(key!='x') {
    key = files[0][counter];
    counter--;
  }
  const int height = stoi(files[0].substr(counter+2,files[0].size()-8-counter-2));
  int a = counter;
  while(key!='_') {
    key = files[0][counter];
    counter--;
  }
  const int width = stoi(files[0].substr(counter+2,a-counter));
  
  std::ostringstream wi;
  wi << width;
  std::string strwi = wi.str();
  std::ostringstream he;
  he << height;
  std::string strhe = he.str();
  
  for(int i = 0; i < (int)files.size()-1; i++)
  {
    if(files[i].substr((int)files[i].size()-7,3)=="xyz") {
      //std::cout<<"xyz"<<std::endl;
      pcl::io::loadPLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_xyz.ply", *xyz);
    }
    else if(files[i].substr((int)files[i].size()-7,3)=="rgb") {
      //std::cout<<"rgb"<<std::endl;
      pcl::io::loadPLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_rgb.ply", *xyzrgb);
    }
    else if(files[i].substr((int)files[i].size()-8,3)=="mes") {
      //std::cout<<"mes"<<std::endl;      
      pcl::io::loadPLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_mesh.ply", mesh);
    }
    else if(files[i].substr((int)files[i].size()-11,3)=="nor") {
      //std::cout<<"nor"<<std::endl;
      pcl::io::loadPLYFile ("./../data/plyfiles/"+filename+"_"+strwi+"x"+strhe+"_normals.ply", *xyznormal);
      for(int i = 0; i< xyznormal->points.size(); i++) {
       if(xyznormal->points[i].normal_x==0 & xyznormal->points[i].normal_y==0 & xyznormal->points[i].normal_z==0) {
          xyznormal->points[i].normal_x = std::numeric_limits<double>::quiet_NaN();
          xyznormal->points[i].normal_y = std::numeric_limits<double>::quiet_NaN();
          xyznormal->points[i].normal_z = std::numeric_limits<double>::quiet_NaN();
       }
       if(xyznormal->points[i].curvature==0)
          xyznormal->points[i].curvature = std::numeric_limits<double>::quiet_NaN();
    }
    }
  }
}

// save xyz-data to threejsfiles
void compute_xyz(std::string filename, const int width, const int height) 
{
  std::ifstream is;
  is.open(("./../data/plyfiles/"+filename).c_str());
  std::string line;
  std::vector<std::string> text;
  int counter = 0;
  while (counter<width*height+31)
  {
    getline (is,line);
    text.push_back(line);
    counter++;
  }
  is.close();
  
  std::ofstream os;
  os.open(("./../data/threejsfiles/"+filename.substr(0,filename.size()-7)+"xyz.three").c_str());
  for(int i = 0; i < (int)text.size()-1; i++)
    os << text[i] << "\n";
  os << text[text.size()-1];
  os.close();
}

// save rgb-data to threejsfiles
void compute_rgb(std::string filename, const int width, const int height) 
{
  std::ifstream is;
  is.open(("./../data/plyfiles/"+filename).c_str());
  std::string line;
  std::vector<std::string> text;
  std::vector<std::vector<double> > datar;
  std::vector<std::vector<double> > datag;
  std::vector<std::vector<double> > datab;
  std::vector<double> rowr;
  std::vector<double> rowg;
  std::vector<double> rowb;
  
  int counter = 0;
  int u = 0;
  while (counter<width*height+33)
  {
    if(counter<33) {
      getline (is,line);
      text.push_back(line);
    }
    else {
      double x,y,z;
      double r, g, b;
      is >> x >> y >> z >> r >> g >> b;
      rowr.push_back(r);
      rowg.push_back(g);
      rowb.push_back(b);
      u++;
    }
    if(u==width) {
      datar.push_back(rowr);
      datag.push_back(rowg);
      datab.push_back(rowb);
      rowr.clear();
      rowg.clear();
      rowb.clear();
      u = 0;
    }
    counter++;
  }
  getline (is,line);
  getline (is,line);
  text.push_back(line);
  is.close();
  
  std::ofstream os;
  os.open(("./../data/threejsfiles/"+filename.substr(0,filename.size()-7)+"rgb.three").c_str());
  for(int i = 0; i < 7; i++)
    os << text[i] << "\n";
  for(int i = 10; i < (int)text.size()-1; i++)
    os << text[i] << "\n";
  
  for(int v = 0; v < height; v++)
    for(int u = 0; u < width; u++)
      os << datar[v][u] << " " << datag[v][u] << " " << datab[v][u] << "\n";
  os << text[text.size()-1];
  os.close();
}

// save faces to threejsfiles
void compute_faces(std::string filename, const int width, const int height) {
  std::ifstream is;
  is.open(("./../data/plyfiles/"+filename).c_str());
  std::string line;
  std::vector<std::string> text;
  std::vector<int> dataa;
  std::vector<int> datab;
  std::vector<int> datac;
  int offset = 11;
  int counter = 0;
  int filelength = 14;
  while (counter<filelength)
  {
    if(counter<offset) {
      getline (is,line);
    }
    else if(counter==offset) {
      getline (is,line);
      filelength = width*height+stoi(line.substr(13,line.size()-13))+14;
    }
    else if(counter<width*height+14) { getline (is,line); }
    else {
      is.clear();
      int x;
      int a,b,c;
      is >> x >> a >> b >> c;
      dataa.push_back(a);
      datab.push_back(b);
      datac.push_back(c);
    }
    counter++;
  }
  is.close();
  is.open(("./../data/plyfiles/"+filename.substr(0,filename.size()-8)+"xyz.ply").c_str());
  offset = 30;
  counter = 0;
  while(counter<width*height+offset)
  {
    if(counter==3) {
      getline (is,line);
      text.push_back("element vertex " + to_string((int)dataa.size()));
    }
    else if(counter < offset) {
      getline (is,line);
      text.push_back(line);
    }
    else
    {
      getline (is, line);
    }
    counter++;
  }
  getline (is,line);
  text.push_back(line);
  is.close();
  
  std::ofstream os;
  os.open(("./../data/threejsfiles/"+filename.substr(0,filename.size()-8)+"mesh.three").c_str());
  for(int i = 0; i < (int)text.size()-1; i++)
    os << text[i] << "\n";
  for(int i = 0; i < (int)dataa.size(); i++)
    os << dataa[i] << " " << datab[i] << " " << datac[i] << "\n";
  os << text[text.size()-1];
  os.close();
}

// save normals to threejsfiles
void compute_normals(std::string filename, const int width, const int height) {
  std::ifstream is;
  is.open(("./../data/plyfiles/"+filename).c_str());
  std::string line;
  std::vector<std::string> text;
  std::vector<std::vector<double> > dataa;
  std::vector<std::vector<double> > datab;
  std::vector<std::vector<double> > datac;
  std::vector<double> rowa;
  std::vector<double> rowb;
  std::vector<double> rowc;
  
  const int offset = 34;
  int counter = 0;
  int u = 0;
  
  while (counter<width*height+offset)
  {
    if(counter<offset) {
      getline (is,line);
      text.push_back(line);
    }
    else {
      double x,y,z,k;
      double a,b,c;
      is >> x;
      is >> y;
      is >> z;
      if(z==0) {
	  getline(is,line);
	  rowa.push_back(0);
	  rowb.push_back(0);
	  rowc.push_back(0);
	  k=0;
      }
      else {
      is >> a;
      is >> b;
      is >> c;
      is >> k;
      rowa.push_back(a);
      rowb.push_back(b);
      rowc.push_back(c);
      }
      u++;
    }
    if(u==width) {
      dataa.push_back(rowa);
      datab.push_back(rowb);
      datac.push_back(rowc);
      rowa.clear();
      rowb.clear();
      rowc.clear();
      u = 0;
    }
    counter++;
  }
  getline (is,line);
  text.push_back(line);

  is.close();
  
  std::ofstream os;
  os.open(("./../data/threejsfiles/"+filename.substr(0,filename.size()-12)+"_normals.three").c_str());
  for(int i = 0; i < 7; i++)
    os << text[i] << "\n";
  for(int i = 11; i < (int)text.size()-1; i++)
    os << text[i] << "\n";
  for(int v = 0; v < height; v++)
    for(int u = 0; u < width; u++)
      os << dataa[v][u] << " " << datab[v][u] << " " << datac[v][u] << "\n";
  os << text[text.size()-1];
  os.close();
}

// generate threejsfiles, if plyfiles are avaible
void buildthreejsdata(std::string filename) {
  std::ifstream is;
  is.open(("./../data/"+filename+".pcl").c_str());
  std::string line;
  std::vector<std::string> files;
  while (!is.eof()) {
      getline (is,line);
      files.push_back(line);
  }
  char key;
  int counter = (int)files[0].size()-8;
  while(key!='x') {
    key = files[0][counter];
    counter--;
  }
  const int height = stoi(files[0].substr(counter+2,files[0].size()-8-counter-2));
  int a = counter;
  while(key!='_') {
    key = files[0][counter];
    counter--;
  }
  const int width = stoi(files[0].substr(counter+2,a-counter));
  std::string x = "o";
  std::string c = "o";
  std::string n = "o";
  std::string f = "o";
  for(int i = 0; i < (int)files.size()-1; i++)
  {
    if(files[i].substr((int)files[i].size()-7,3)=="xyz") {
      compute_xyz(files[i],width,height);
      x = "x";
    }
    else if(files[i].substr((int)files[i].size()-7,3)=="rgb") {
      compute_rgb(files[i],width,height);
      c = "c";
    }
    else if(files[i].substr((int)files[i].size()-8,3)=="mes") {
      compute_faces(files[i],width,height);
      f = "f";
    }
    else if(files[i].substr((int)files[i].size()-11,3)=="nor") {
      compute_normals(files[i],width,height);
      n = "n";
    }
  }
  std::ofstream os;
  os.open(("./../data/"+filename+"_"+to_string(width)+"x"+to_string(height)+"_"+x+c+n+f+".three").c_str());
  for(int i = 0; i < (int)files.size()-1; i++)
  {
    if(files[i].substr((int)files[0].size()-7,3)=="xyz")
      os << files[i].substr(0,files[i].size()-7)+"xyz.three";
    if(files[i].substr((int)files[0].size()-7,3)=="rgb")
      os << "\n" << files[i].substr(0,files[i].size()-7)+"rgb.three";
    else if(files[i].substr((int)files[0].size()-7,3)=="nor")
      os << "\n" << files[i].substr(0,files[i].size()-11)+"normals.three";
    else if(files[i].substr((int)files[0].size()-7,3)=="mes")
      os << "\n" << files[i].substr(0,files[i].size()-8)+"mesh.three";
  }
  os.close();
}
