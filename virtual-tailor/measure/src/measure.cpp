#include "measure.h"

int measure(PointCloud<PointXYZRGBA>& input, float & measure) {
	pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud(
			new pcl::PointCloud<pcl::PointXYZRGBA>);

	*cloud = input;
	float uzunluk = 0;
	Eigen::Vector3f a, b;
	for (size_t i = 0; i < cloud->points.size(); ++i) {
		if (i != cloud->points.size() - 1) {
			a = Eigen::Vector3f(cloud->at(i).getArray3fMap());
			b = Eigen::Vector3f(cloud->at(i + 1).getArray3fMap());
		} else {
			a = Eigen::Vector3f(cloud->at(i).getArray3fMap());
			b = Eigen::Vector3f(cloud->at(0).getArray3fMap());
		}
		cout << a[0] << "," << a[1] << "," << a[2] << endl;
		cout << b[0] << "," << b[1] << "," << b[2] << endl;
		uzunluk += pcl::geometry::distance(a, b);
		cout << uzunluk << endl;
	}

	measure = uzunluk;

	return 0;
}

