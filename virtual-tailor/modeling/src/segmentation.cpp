/*
 * main.cpp
 *
 *  Created on: May 5, 2015
 *      Author: mc
 */
#include "segmentation.h"

int segmentation(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out) {

	pcl::ModelCoefficients::Ptr coefficients(new pcl::ModelCoefficients);
	pcl::PointIndices::Ptr inliers(new pcl::PointIndices);
// Create the segmentation object
	pcl::SACSegmentation < pcl::PointXYZRGBA > seg;
	pcl::VoxelGrid < pcl::PCLPointCloud2 > sor;
	pcl::PCLPointCloud2::Ptr cloud_blob(new pcl::PCLPointCloud2),
			cloud_filtered_blob(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	pcl::ExtractIndices < pcl::PointXYZRGBA > extract;

	pcl::toPCLPointCloud2(in, *cloud_blob);
	sor.setInputCloud(cloud_blob);
//0.005f orjinali
	sor.setLeafSize(0.005f, 0.005f, 0.005f);
	sor.filter(*cloud_filtered_blob);
	pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

	std::cerr << "PointCloud after filtering: "
			<< cloud_filtered->width * cloud_filtered->height << " data points."
			<< std::endl;

	seg.setOptimizeCoefficients(true);
	seg.setModelType(pcl::SACMODEL_PLANE);
	seg.setMethodType(pcl::SAC_RANSAC);
	//seg.setMaxIterations (1000);
	seg.setDistanceThreshold(0.1);

	int i = 0, nr_points = (int) cloud_filtered->points.size();
	// While 30% of the original cloud is still there

	seg.setInputCloud(cloud_filtered);

	seg.segment(*inliers, *coefficients);
	if (inliers->indices.size() == 0) {
		std::cerr << "Could not estimate a planar model for the given dataset."
				<< std::endl;
		return -1;
	}

	// Extract the inliers
	extract.setInputCloud(cloud_filtered);
	extract.setIndices(inliers);
	extract.setNegative(true);
	extract.filter(out);
	std::cerr << "PointCloud representing the planar component: "
			<< out.width * out.height << " data points." << std::endl;

	i++;

}

