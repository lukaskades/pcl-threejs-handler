<!--
  Title: PCL and Three.js Handler  
  Description: An application for visualisation, reconstruction and processing of point clouds via the PCL Point Cloud Library and Three.js
  Author: lukas
  -->

# pcl-threejs-handler

![alt tag](https://cloud.githubusercontent.com/assets/22341762/25317575/7c800580-287b-11e7-9bcb-86c74c3e9ce6.png)
Three.js Depth Data Viewer in the Browser with Normals and Wireframe - adapted and extended from https://github.com/spite/android-lens-blur-depth-extractor

## An application for visualisation, reconstruction and processing of point clouds via the PCL Point Cloud Library and Three.js

Simple example for a usage of the **PCL Point Cloud Library** and **Three.js**. An easy console interface was programmed for first insights to the PCL Point Cloud Library. It is possible to safe the cloud with a rendered mesh and texture. This cloud can than be loaded with the help of Three.js in a Browser.

# Description

In this project possibilities and constraints of two open projects are explored, the Point Cloud Library (PCL) and the Three.js library. The main focus is based on the illustration of point clouds and the surface reconstruction and normal estimation methods respectively. PCL is written in C++ and offers a host of different possibilities for image processing. In contrast Three.js is mainly suitable for illustrating 3D data in a fast easily usable way with javascript in an html framework. The powerful javascript library provides furthermore many useful tools concerning reality effects like reflection, shadow, fog, etc.,
different textures and geometric basic objects, which makes it attractive for certain applications in webdesign and advertising.

I did this project in the course of a software project at the university and hope that other can profit from my experiences with PCL and Three.js. For a **detailed information** one can find my project report in the doc folder. At this point I want to thank spite for his Depth Data Viewer at https://github.com/spite/android-lens-blur-depth-extractor, which helped me a lot and from whom I have used a lot of code for the Depth Data Viewer of this project.

# Compiling

For the build process one needs to install pcl and opencv. For Ubuntu 16.04 one can follow these links:

  - PCL:    
      https://larrylisky.com/2014/03/03/installing-pcl-on-ubuntu/    
      http://pointclouds.org/downloads/linux.html
  - OpenCV:    
      http://docs.opencv.org/2.4/doc/tutorials/introduction/linux_install/linux_install.html

After a successful installation one has to build the project in the cmake directory via:    

    cmake .    
    make

# Running

The simple console interface can be started via the 'Main' executable in the bin directory with   

    ./Main

After saving the PointCloud, a .pcl file is created in the data directory and can be loaded again with the console interface. With the 'Build Three.js Data' option in the console one can create a .three js file. One can than view the cloud with the Depth Data Viewer in the root directory of the project by simply open **depthdataviewer.html** in a browser and loading the corresponding .three file in the data directory. The 'xcfn' in the file name indicates whether spatial coordinates (x), color information (c), normals (n) and faces (f ) are loadable.

![alt tag](https://cloud.githubusercontent.com/assets/22341762/25317596/d4bf5ebc-287b-11e7-9ff3-641867e1afab.png)
PCL 3D Viewer with User Interface on the console

# Comments

The implementation is of course not perfect and especially for the data viewer there might occur several bugs (for example is it not possible to load a textured mesh at the moment). However, in principle most of the applications should work.
