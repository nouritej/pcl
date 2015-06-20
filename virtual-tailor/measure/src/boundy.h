#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/boundary.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <iostream>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/filters/statistical_outlier_removal.h>

#include <pcl/PCLPointCloud2.h>
#include <pcl/io/pcd_io.h>
#include <pcl/features/boundary.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

int boundary_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,
		int k, double angle);


using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;
using namespace std;

int boundary_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,int k,
		double angle);
