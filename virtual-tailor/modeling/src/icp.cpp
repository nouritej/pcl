#include "icp.h"

Eigen::Matrix4f icp_cl(PointCloud<PointXYZRGBA>& input1,
		PointCloud<PointXYZRGBA>& input2, PointCloud<PointXYZRGBA>& out,
		double dist = 0.05, double rans = 0.05, int iter = 50, bool nonLinear =
				false) {

	CloudPtr model(new Cloud);
	*model = input1;
	std::cout << " width: " << model->width << " height: " << model->height
			<< std::endl;

	Eigen::Matrix4f t(Eigen::Matrix4f::Identity());

	CloudPtr data(new Cloud);
	*data = input2;
	std::cout << " width: " << data->width << " height: " << data->height
			<< std::endl;

	pcl::IterativeClosestPoint < PointType, PointType > *icp;

	if (nonLinear) {
		std::cout << "Using IterativeClosestPointNonLinear" << std::endl;
		icp = new pcl::IterativeClosestPointNonLinear<PointType, PointType>();
	} else {
		std::cout << "Using IterativeClosestPoint" << std::endl;
		icp = new pcl::IterativeClosestPoint<PointType, PointType>();
	}

	icp->setMaximumIterations(iter);
	icp->setMaxCorrespondenceDistance(dist);
	icp->setRANSACOutlierRejectionThreshold(rans);

	icp->setInputTarget(model);

	icp->setInputSource(data);

	CloudPtr tmp(new Cloud);
	icp->align(*tmp);

	t = t * icp->getFinalTransformation();

	pcl::transformPointCloud(*data, *tmp, t);

	std::cout << "has converged:" << icp->hasConverged() << " score: "
			<< icp->getFitnessScore() << std::endl;

	std::cout << icp->getFinalTransformation() << std::endl;

	pcl::io::savePCDFileBinary("tmp.pcd", *tmp);
	std::cout << "saving result to " << "tmp.pcd" << std::endl;

//out = *tmp;

	return icp->getFinalTransformation();

}

