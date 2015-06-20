#include "cluster.h"

void compute(pcl::PCLPointCloud2::Ptr &input,
		std::vector<pcl::PCLPointCloud2::Ptr> &output, int min, int max,
		double tolerance, Eigen::Vector4f translation,
		Eigen::Quaternionf orientation) {
	// Convert data to PointCloud<T>

	PointCloud<pcl::PointXYZRGBA>::Ptr xyz(new PointCloud<pcl::PointXYZRGBA>);
	fromPCLPointCloud2(*input, *xyz);

	// Estimate
	TicToc tt;
	tt.tic();

	print_highlight(stderr, "Computing ");

	// Creating the KdTree object for the search method of the extraction
	pcl::search::KdTree<pcl::PointXYZRGBA>::Ptr tree(
			new pcl::search::KdTree<pcl::PointXYZRGBA>);
	tree->setInputCloud(xyz);

	std::vector < pcl::PointIndices > cluster_indices;
	pcl::EuclideanClusterExtraction < pcl::PointXYZRGBA > ec;
	ec.setClusterTolerance(tolerance);
	ec.setMinClusterSize(min);
	ec.setMaxClusterSize(max);
	ec.setSearchMethod(tree);
	ec.setInputCloud(xyz);
	ec.extract(cluster_indices);

	print_info("[done, ");
	print_value("%g", tt.toc());
	print_info(" ms : ");
	print_value("%d", cluster_indices.size());
	print_info(" clusters]\n");

	output.reserve(cluster_indices.size());
	for (std::vector<pcl::PointIndices>::const_iterator it =
			cluster_indices.begin(); it != cluster_indices.end(); ++it) {
		pcl::ExtractIndices < pcl::PCLPointCloud2 > extract;
		extract.setInputCloud(input);
		extract.setIndices(boost::make_shared<const pcl::PointIndices>(*it));
		extract.setNegative(true);
		pcl::PCLPointCloud2::Ptr outz(new pcl::PCLPointCloud2);
		extract.filter(*outz);
		output.push_back(outz);
	}
}

int cluster_cl(pcl::PCLPointCloud2::Ptr &in, PointCloud<PointXYZRGBA>& out,
		Eigen::Vector4f translation, Eigen::Quaternionf orientation, int min,
		int max, double tolerance) {

	// Perform the feature estimation
	std::vector < pcl::PCLPointCloud2::Ptr > output;
	compute(in, output, min, max, tolerance, translation, orientation);
	int counter = 0, sayi = 0;
	for (size_t i = 0; i < output.size(); i++) {
		std::string basename = "cabbar";
		std::string clustername = basename + boost::lexical_cast < std::string
				> (i) + ".pcd";
		print_highlight("Saving ");
		print_value("%s ", clustername.c_str());

		pcl::io::savePCDFile(clustername, *(output[i]), translation,
				orientation, false);
		if (counter < output[i]->width * output[i]->height)
			sayi = i;
	}
	fromPCLPointCloud2(*(output[sayi]), out);

}

