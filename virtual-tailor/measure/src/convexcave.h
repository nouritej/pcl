#include <pcl/ModelCoefficients.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/filters/passthrough.h>
#include <pcl/filters/project_inliers.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/surface/concave_hull.h>

using namespace pcl;
using namespace pcl::io;
using namespace pcl::console;

int
convexcave(PointCloud<PointXYZRGBA>& input, PointCloud<PointXYZRGBA>& output2,
		float threshold, float alpha);

