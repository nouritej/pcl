#include "filter.h"

int filter_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& outsol,
		PointCloud<PointXYZRGBA>& outsag, PointCloud<PointXYZRGBA>& out,
		int axe, float filter_start, float filter_end) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	*cloud = in;
	pcl::PointXYZRGBA minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	// Create the filtering object
	pcl::PassThrough < pcl::PointXYZRGBA > pass;
	pass.setInputCloud(cloud);
// cut axe x.
	if (axe == 0) {
		pass.setFilterFieldName("x");
		pass.setFilterLimits(minPt.x, maxPt.x - 2 * (maxPt.x - minPt.x) / 3);
		pass.filter(*cloud_filtered);
		outsol = *cloud_filtered;
		pass.setFilterLimits(minPt.x + 2 * (maxPt.x - minPt.x) / 3, maxPt.x);
		pass.filter(*cloud_filtered);
		outsag = *cloud_filtered;
	}

// cut axe y.
	else if (axe == 1) {
		pass.setFilterFieldName("y");
		pass.setFilterLimits(filter_start, filter_end);
		pass.filter(*cloud_filtered);
		out = *cloud_filtered;
	}

	//pass.setFilterLimitsNegative (true);

	return (0);
}

