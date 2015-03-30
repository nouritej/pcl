// Original code by Geoffrey Biggs, taken from the PCL tutorial in
// http://pointclouds.org/documentation/tutorials/pcl_visualizer.php

// Simple OpenNI viewer that also allows to write the current scene to a .pcd
// when pressing SPACE.

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

PointCloud<PointXYZRGBA>::Ptr cloud(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZRGBA>::Ptr cloudptr(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZRGBA>::Ptr cloudptr2(new PointCloud<PointXYZRGBA>); // A cloud that will store color info.
PointCloud<PointXYZ>::Ptr fallbackCloud(new PointCloud<PointXYZ>);    // A fallback cloud with just depth data.
boost::shared_ptr<visualization::CloudViewer> viewer;                 // Point cloud viewer object.
Grabber* openniGrabber;                                               // OpenNI grabber that takes data from the device.
unsigned int filesSaved = 0;                                          // For the numbering of the clouds saved to disk.
bool saveCloud(false), noColor(false);                                // Program control.

pcl::ModelCoefficients::Ptr coefficients (new pcl::ModelCoefficients);
			  pcl::PointIndices::Ptr inliers (new pcl::PointIndices);
			  // Create the segmentation object
			  pcl::SACSegmentation<pcl::PointXYZRGBA> seg;
pcl::VoxelGrid<pcl::PCLPointCloud2> sor;
pcl::PCLPointCloud2::Ptr cloud_blob (new pcl::PCLPointCloud2), cloud_filtered_blob (new pcl::PCLPointCloud2);
 pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud_filtered (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_p (new pcl::PointCloud<pcl::PointXYZRGBA>), cloud_f (new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::ExtractIndices<pcl::PointXYZRGBA> extract;

int
main(int argc, char** argv)
{
	
// Try with color information...
		try
		{
			io::loadPCDFile<PointXYZRGBA>("file.pcd", *cloud);

		}
		catch (PCLException e1)
		{
			try
			{
				// ...and if it fails, fall back to just depth.
				io::loadPCDFile<PointXYZ>("file.pcd", *fallbackCloud);
			}
			catch (PCLException e2)
			{
				return -1;
			}

			noColor = true;
		}



pcl::toPCLPointCloud2(*cloud,*cloud_blob);
sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_filtered_blob);
pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  //writer.write<pcl::PointXYZRGBA> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

		  seg.setOptimizeCoefficients (true);
			  seg.setModelType (pcl::SACMODEL_PLANE);
			  seg.setMethodType (pcl::SAC_RANSAC);
  			//seg.setMaxIterations (1000);
			  seg.setDistanceThreshold (0.1);




int i = 0, nr_points = (int) cloud_filtered->points.size ();
  // While 30% of the original cloud is still there
  while (cloud_filtered->points.size () > 0.2 * nr_points)
  {
 seg.setInputCloud (cloud_filtered);
seg.segment (*inliers, *coefficients);
if (inliers->indices.size () == 0)
    {
      std::cerr << "Could not estimate a planar model for the given dataset." << std::endl;
      break;
    }

    // Extract the inliers
    extract.setInputCloud (cloud_filtered);
    extract.setIndices (inliers);
    extract.setNegative (false);
    extract.filter (*cloud_p);
    std::cerr << "PointCloud representing the planar component: " << cloud_p->width * cloud_p->height << " data points." << std::endl;

 std::stringstream ss;
    ss << "table_scene_lms400_plane_" << i << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss.str (), *cloud_p, false);

	extract.setNegative (true);
    extract.filter (*cloud_f);
    cloud_filtered.swap (cloud_f);
 std::stringstream ss2;
    ss2 << "table_scene_lms400_f_" << i << ".pcd";
    writer.write<pcl::PointXYZRGBA> (ss2.str (), *cloud_f, false);
    i++;





}





	
}
