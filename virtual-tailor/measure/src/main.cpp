//#ifdef PCL_NO_PRECOMPILE  

#include "segmentation.h"
#include "rotate.h"
#include "boundy.h"
#include "filter.h"
#include "cluster.h"
#include "section.h"
#include "icp.h"
#include "convexcave.h"
#include "measure.h"
//#endif

int grandest(int x, int y, int z) {
	return std::max(std::max(x, y), z);
}

char const*array(int i) {
	if (i == 1)
		return "small";
	else if (i == 2)
		return "medium";
	else if (i == 3)
		return "large";
	else if (i == 4)
		return "xlarge";
	else
		return "xxlarge";

}

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(
		new pcl::PointCloud<pcl::PointXYZRGBA>);

int main(int argc, char** argv) {
	int size1, size2, size3;
	double x1, y1, x2, y2, x3, y3, x4, y4, x5, y5, x6, y6;
	x1 = atof(argv[1]);
	y1 = atof(argv[2]);
	x2 = atof(argv[3]);
	y2 = atof(argv[4]);
	x3 = atof(argv[5]);
	y3 = atof(argv[6]);
	x4 = atof(argv[7]);
	y4 = atof(argv[8]);
	x5 = atof(argv[9]);
	y5 = atof(argv[10]);
	x6 = atof(argv[11]);
	y6 = atof(argv[12]);

	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_filtered2(
			new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_projected(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	io::loadPCDFile < PointXYZRGBA > ("file.pcd", *cloud);

	Eigen::Vector4f translation;
	Eigen::Quaternionf orientation;
	pcl::PCLPointCloud2::Ptr cloud166(new pcl::PCLPointCloud2);
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud16(
			new pcl::PointCloud<pcl::PointXYZRGBA>);
// Build a filter to remove spurious NaNs
	pcl::PassThrough < pcl::PointXYZRGBA > pass;
	pcl::PCDWriter writer;
	pcl::PassThrough < pcl::PointXYZRGBA > pass2;

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y1 - 0.01, y1 + 0.01);
	pass.filter(*cloud_filtered);
	writer.write("filtered.pcd", *cloud_filtered, false);
	pass2.setInputCloud(cloud_filtered);
	pass2.setFilterFieldName("x");

	pass2.setFilterLimits(x1, x2);
	pass2.filter(*cloud_filtered2);
	writer.write("filtered2.pcd", *cloud_filtered2, false);
	convexcave(*cloud_filtered2, *cloud16, 0.01, 1);

	if (cloud16->points.size() != 0)
		writer.write("cloud16.pcd", *cloud16, false);
	float sayi;
	measure(*cloud16, sayi);
	cout << "measure:" << sayi << endl;

//2. kısım

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y3 - 0.01, y3 + 0.01);
	pass.filter(*cloud_filtered);
	writer.write("filtered3.pcd", *cloud_filtered, false);
	pass2.setInputCloud(cloud_filtered);
	pass2.setFilterFieldName("x");

	pass2.setFilterLimits(x3, x4);
	pass2.filter(*cloud_filtered2);
	writer.write("filtered4.pcd", *cloud_filtered2, false);
	convexcave(*cloud_filtered2, *cloud16, 0.01, 1);

	if (cloud16->points.size() != 0)
		writer.write("cloud162.pcd", *cloud16, false);
	float sayi2;
	measure(*cloud16, sayi2);
	cout << "measure:" << sayi << endl;

// 3. kısım

	pass.setInputCloud(cloud);
	pass.setFilterFieldName("y");
	pass.setFilterLimits(y5 - 0.01, y5 + 0.01);
	pass.filter(*cloud_filtered);
	writer.write("filtered5.pcd", *cloud_filtered, false);
	pass2.setInputCloud(cloud_filtered);
	pass2.setFilterFieldName("x");

	pass2.setFilterLimits(x5, x6);
	pass2.filter(*cloud_filtered2);
	writer.write("filtered6.pcd", *cloud_filtered2, false);
	convexcave(*cloud_filtered2, *cloud16, 0.01, 0.3);

	if (cloud16->points.size() != 0)
		writer.write("cloud163.pcd", *cloud16, false);
	float sayi3;
	measure(*cloud16, sayi3);
	cout << "measure:" << sayi << sayi2 << sayi3 << endl;

	sayi *= 100;
	sayi2 *= 100;
	sayi3 *= 100;

	if (sayi < 95)
		size1 = 1;
	else if (sayi < 104)
		size1 = 2;
	else if (sayi < 112)
		size1 = 3;
	else if (sayi < 123)
		size1 = 4;
	else if (sayi < 136)
		size1 = 5;

	if (sayi2 < 81)
		size2 = 1;
	else if (sayi2 < 89)
		size2 = 2;
	else if (sayi2 < 97)
		size2 = 3;
	else if (sayi2 < 109)
		size2 = 4;
	else if (sayi2 < 121)
		size2 = 5;

	if (sayi3 < 95)
		size3 = 1;
	else if (sayi3 < 104)
		size3 = 2;
	else if (sayi3 < 112)
		size3 = 3;
	else if (sayi3 < 119)
		size3 = 4;
	else if (sayi3 < 128)
		size3 = 5;

	cout << "beden: " << array(grandest(size1, size2, size3)) << endl;
	cout << "gogus: " << array(size1) << " bel: " << array(size2) << " basen: "
			<< array(size3) << endl;

}

