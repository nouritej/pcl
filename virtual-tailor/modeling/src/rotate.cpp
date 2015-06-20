#include "rotate.h"

typedef pcl::PointXYZRGB PointType;
typedef pcl::PointCloud<PointType> Cloud;
typedef Cloud::ConstPtr CloudConstPtr;
typedef Cloud::Ptr CloudPtr;

int rotate_cl(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,
		int axe, float angle) {

	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	// eger axe 0sa x etrafında dönüyordur.
	if (axe == 0) {
		t(0, 0) = 1;
		t(0, 1) = 0;
		t(0, 2) = 0;
		t(0, 3) = 0;
		t(1, 0) = 0;
		t(1, 1) = cos(angle);
		t(1, 2) = -sin(angle);
		t(1, 3) = 0;
		t(2, 0) = 0;
		t(2, 1) = sin(angle);
		t(2, 2) = cos(angle);
		t(2, 3) = 0;
	}

	// eger axe 1 ise y etrafında dönüyordur.
	else if (axe == 1) {
		t(0, 0) = cos(angle);
		t(0, 1) = 0;
		t(0, 2) = sin(angle);
		t(0, 3) = 0;
		t(1, 0) = 0;
		t(1, 1) = 1;
		t(1, 2) = 0;
		t(1, 3) = 0;
		t(2, 0) = -sin(angle);
		t(2, 1) = 0;
		t(2, 2) = cos(angle);
		t(2, 3) = 0;
	}

	// eger axe 2 ise z etrafında dönüyordur.
	else if (axe == 2) {
		t(0, 0) = cos(angle);
		t(0, 1) = -sin(angle);
		t(0, 2) = 0;
		t(0, 3) = 0;
		t(1, 0) = sin(angle);
		t(1, 1) = cos(angle);
		t(1, 2) = 0;
		t(1, 3) = 0;
		t(2, 0) = 0;
		t(2, 1) = 0;
		t(2, 2) = 1;
		t(2, 3) = 0;
	}

	t(3, 0) = 0;
	t(3, 1) = 0;
	t(3, 2) = 0;
	t(3, 3) = 1;

	pcl::transformPointCloud(in, out, t);
	return 0;
}

int rotate_matrix(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out,
		Eigen::Matrix4f t) {
	pcl::transformPointCloud(in, out, t);
	return 0;
}


