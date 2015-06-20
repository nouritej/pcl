/*
 * main.cpp
 *
 *  Created on: May 5, 2015
 *      Author: mc
 */

#include <pcl/io/openni_grabber.h>
#include <pcl/io/pcd_io.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/console/parse.h>
#include <pcl/io/io.h>
#include <iostream>
#include <pcl/ModelCoefficients.h>
#include <pcl/point_types.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/extract_indices.h>

using namespace pcl;
using namespace std;

int segmentation(PointCloud<PointXYZRGBA>& in, PointCloud<PointXYZRGBA>& out);
