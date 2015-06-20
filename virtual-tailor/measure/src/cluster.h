#include <pcl/PCLPointCloud2.h>
#include <pcl/point_types.h>
#include <pcl/io/pcd_io.h>
#include <pcl/segmentation/extract_clusters.h>
#include <pcl/filters/extract_indices.h>
#include <pcl/console/print.h>
#include <pcl/console/parse.h>
#include <pcl/console/time.h>
#include <vector>
#include "boost.h"
#include <string>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int cluster_cl(pcl::PCLPointCloud2::Ptr &in, PointCloud<PointXYZRGBA>& out,
		Eigen::Vector4f translation, Eigen::Quaternionf orientation, int min,
		int max, double tolerance);

