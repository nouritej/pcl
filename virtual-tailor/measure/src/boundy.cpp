#include "boundy.h"

int boundary_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,
		int k, double angle) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::StatisticalOutlierRemoval < pcl::PointXYZRGBA > sor;
	*cloud = in;
	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << in << std::endl;

	// Create the filtering object

	sor.setInputCloud(cloud);
	sor.setMeanK(k);
	sor.setStddevMulThresh(angle);
	sor.filter(*cloud_filtered);

	std::cerr << "Cloud after filtering: " << std::endl;
	std::cerr << *cloud_filtered << std::endl;

	sor.setNegative(true);
	sor.filter(*cloud_filtered);
	out = *cloud_filtered;

	return 0;
}

