#include <pcl/console/parse.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/registration/icp.h>
#include <pcl/registration/icp_nl.h>
#include <string>
#include <iostream>
#include <fstream>
#include <vector>
/*
typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;
*/
using namespace pcl;
using namespace std;

int rotate_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,int axe,float angle);

int rotate_matrix(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out, Eigen::Matrix4f t) ;
