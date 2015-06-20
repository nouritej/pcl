#include "convexcave.h"

int convexcave(PointCloud<PointXYZRGBA>& input,
		PointCloud<PointXYZRGBA>& output, float threshold, float alpha) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_projected(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::PCDReader reader;

	// Build a filter to remove spurious NaNs
	*cloud = input;

	for (size_t i = 0; i < cloud->points.size(); ++i) {
		cloud->points[i].y = cloud->points[0].y;
	}
	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
	// Create the segmentation object
	pcl::SACSegmentation < pcl::PointXYZRGBA > seg;
	// Optional
	seg.setOptimizeCoefficients(true);
	// Mandatory
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	seg.setDistanceThreshold(threshold);

	seg.setInputCloud(cloud);
	seg.segment(*inliers, *coefficients);
	std::cerr << "PointCloud after segmentation has: "
			<< inliers->indices.size() << " inliers." << std::endl;

	// Project the model inliers
	pcl::ProjectInliers < pcl::PointXYZRGBA > proj;
	proj.setModelType(pcl::SACMODEL_PLANE);
	proj.setIndices(inliers);
	proj.setInputCloud(cloud);
	proj.setModelCoefficients(coefficients);
	proj.filter(*cloud_projected);
	std::cerr << "PointCloud after projection has: "
			<< cloud_projected->points.size() << " data points." << std::endl;

	// Create a Concave Hull representation of the projected inliers
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_hull(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	pcl::ConcaveHull < pcl::PointXYZRGBA > chull;
	chull.setInputCloud(cloud_projected);
	chull.setAlpha(alpha);
	chull.reconstruct(*cloud_hull);

	std::cerr << "Concave hull has: " << cloud_hull->points.size()
			<< " data points." << std::endl;

	output = *cloud_hull;
	return (0);
}

