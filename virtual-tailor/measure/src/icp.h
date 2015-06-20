#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>

#include <string>
#include <iostream>
#include <fstream>
#include <vector>
using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

typedef pcl::PointXYZRGBA PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;


Eigen::Matrix4f
icp_cl (PointCloud<PointXYZRGBA>& input1, PointCloud<PointXYZRGBA>& input2,PointCloud<PointXYZRGBA>& out,double dist,double rans,int iter,bool nonLinear);
