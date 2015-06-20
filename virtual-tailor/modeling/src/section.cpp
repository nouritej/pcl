#include "section.h"

#include "cluster.h"
#include "section.h"
#include "icp.h"
#include "convexcave.h"
#include "measure.h"

struct Point {
	double x;
	double y;
	double z;
};
bool myfunction(Point a, Point b) {
	return (a.z < b.z);
}

int default_min = 100;
int default_max = 25000;
double default_tolerance = 0.04;
int section_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,
		int type) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_projected(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	*cloud = in;

	pcl::PointXYZRGBA minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);

	std::vector<Point> xV;
	xV.resize(cloud->points.size());
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		xV[i].y = cloud->points[i].y;
		xV[i].x = cloud->points[i].x;
		xV[i].z = cloud->points[i].z;
	}
	std::sort(xV.begin(), xV.end(), myfunction);
	double minZ = xV[0].z;

// Build a filter to remove spurious NaNs
	pcl::PassThrough < pcl::PointXYZRGBA > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("x");
	if (type == 0)
		pass.setFilterLimits(minPt.x + 2 * (maxPt.x - minPt.x) / 3 - 0.01,
				minPt.x + 2 * (maxPt.x - minPt.x) / 3 + 0.01);
	else
		pass.setFilterLimits(maxPt.x - 2 * (maxPt.x - minPt.x) / 3 - 0.01,
				maxPt.x - 2 * (maxPt.x - minPt.x) / 3 + 0.01);
	pass.filter(*cloud_filtered);
	std::cerr << "PointCloud after filtering has: "
			<< cloud_filtered->points.size() << " data points." << std::endl;
	pcl::PCDWriter writer;
	writer.write("filtered.pcd", *cloud_filtered, false);
	out = *cloud_filtered;

	return (0);
}

int section_cb(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out) {

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_projected(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	*cloud = in;

	pcl::PointXYZRGBA minPt, maxPt;
	pcl::getMinMax3D(*cloud, minPt, maxPt);
	Eigen::Vector4f translation;
	Eigen::Quaternionf orientation;
// Build a filter to remove spurious NaNs
	pcl::PassThrough < pcl::PointXYZRGBA > pass;
	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pcl::PCDWriter writer;
	cout << "layt " << minPt.y << "," << maxPt.y << endl;
	pcl::PCLPointCloud2::Ptr cloud166(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud16(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
	std::ofstream outfile;

	outfile.open("test.txt", std::ios_base::app);
	for (float i = minPt.y; i < maxPt.y - (maxPt.y - minPt.y) / 500;
			i += (maxPt.y - minPt.y) / 500) {
		pass.setFilterLimits(i - (maxPt.y - minPt.y) / 500,
				i + (maxPt.y - minPt.y) / 500);
		pass.filter(*cloud_filtered);
		std::ostringstream strs;
		strs << i << ".pcd";
		std::string str = strs.str();
		writer.write(str, *cloud_filtered, false);
		loadPCDFile(str, *cloud166, translation, orientation);
//cluster_cl(cloud166,*cloud16,translation, orientation,default_min, default_max,default_tolerance );
		convexcave(*cloud_filtered, *cloud16, 0.01, 1);
		std::ostringstream strs2;
		strs2 << i << "_" << ".pcd";
		str = strs2.str();
		if (cloud16->points.size() != 0)
			writer.write(str, *cloud16, false);
		float sayi;
		measure(*cloud16, sayi);
		cout << "measure:" << sayi << endl;
		outfile << sayi << " " << i << "\n";
	}

	std::cerr << "PointCloud after filtering has: "
			<< cloud_filtered->points.size() << " data points." << std::endl;
	out = *cloud_filtered;
	for (size_t i = 0; i < out.points.size(); ++i) {
		out.points[i].y = out.points[0].y;
	}
	return (0);
}

