#include <iostream>
#include <pcl/point_types.h>
#include <pcl/filters/passthrough.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/io/pcd_io.h>

using namespace pcl;
using namespace std;

int filter_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& outsol,PointCloud<PointXYZRGBA>& outsag,PointCloud<PointXYZRGBA>& out,int axe,float filter_start,float filter_end);
