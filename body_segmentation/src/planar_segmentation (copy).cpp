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

void
printUsage(const char* programName)
{
	cout << "Usage: " << programName << " [options]"
		 << endl
		 << endl
		 << "Options:\n"
		 << endl
		 << "\t<none>     start capturing from an OpenNI device.\n"
		 << "\t-v FILE    visualize the given .pcd file.\n"
		 << "\t-h         shows this help.\n";
}

// This function is called every time the device has new data.
void
grabberCallback(const PointCloud<PointXYZRGBA>::ConstPtr& cloud)
{
	if (! viewer->wasStopped())
{
  pcl::toPCLPointCloud2(*cloud,*cloud_blob);
sor.setInputCloud (cloud_blob);
  sor.setLeafSize (0.005f, 0.005f, 0.005f);
  sor.filter (*cloud_filtered_blob);
pcl::fromPCLPointCloud2(*cloud_filtered_blob, *cloud_filtered);

  std::cerr << "PointCloud after filtering: " << cloud_filtered->width * cloud_filtered->height << " data points." << std::endl;

  // Write the downsampled version to disk
  pcl::PCDWriter writer;
  writer.write<pcl::PointXYZRGBA> ("table_scene_lms400_downsampled.pcd", *cloud_filtered, false);

		  seg.setOptimizeCoefficients (true);
			  seg.setModelType (pcl::SACMODEL_PLANE);
			  seg.setMethodType (pcl::SAC_RANSAC);
  			seg.setMaxIterations (1000);
			  seg.setDistanceThreshold (0.1);

			 viewer->showCloud(cloud_filtered);


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

boost::this_thread::sleep(boost::posix_time::seconds(1));
}
			  

}
		

	if (saveCloud)
	{
		stringstream stream;
		stream << "inputCloud" << filesSaved << ".pcd";
		string filename = stream.str();
		if (io::savePCDFile(filename, *cloud, true) == 0)
		{
			filesSaved++;
			cout << "Saved " << filename << "." << endl;
		}
		else PCL_ERROR("Problem saving %s.\n", filename.c_str());

		saveCloud = false;
	}
}

// For detecting when SPACE is pressed.
void
keyboardEventOccurred(const visualization::KeyboardEvent& event,
					  void* nothing)
{
	if (event.getKeySym() == "space" && event.keyDown())
		saveCloud = true;
}

// Creates, initializes and returns a new viewer.
boost::shared_ptr<visualization::CloudViewer>
createViewer()
{
	boost::shared_ptr<visualization::CloudViewer> v
	(new visualization::CloudViewer("OpenNI viewer"));
	v->registerKeyboardCallback(keyboardEventOccurred);

	return (v);
}

int
main(int argc, char** argv)
{
	if (console::find_argument(argc, argv, "-h") >= 0)
	{
		printUsage(argv[0]);
		return -1;
	}

	bool justVisualize(false);
	string filename;
	if (console::find_argument(argc, argv, "-v") >= 0)
	{
		if (argc != 3)
		{
			printUsage(argv[0]);
			return -1;
		}

		filename = argv[2];
		justVisualize = true;
	}
	else if (argc != 1)
	{
		printUsage(argv[0]);
		return -1;
	}

	// First mode, open and show a cloud from disk.
	if (justVisualize)
	{
		// Try with color information...
		try
		{
			io::loadPCDFile<PointXYZRGBA>(filename.c_str(), *cloudptr);

		}
		catch (PCLException e1)
		{
			try
			{
				// ...and if it fails, fall back to just depth.
				io::loadPCDFile<PointXYZ>(filename.c_str(), *fallbackCloud);
			}
			catch (PCLException e2)
			{
				return -1;
			}

			noColor = true;
		}

		cout << "Loaded " << filename << "." << endl;
		if (noColor)
			cout << "This cloud has no RGBA color information present." << endl;
		else cout << "This cloud has RGBA color information present." << endl;
	}
	// Second mode, start fetching and displaying frames from the device.
	else
	{
		openniGrabber = new OpenNIGrabber();
		if (openniGrabber == 0)
			return -1;
		boost::function<void (const PointCloud<PointXYZRGBA>::ConstPtr&)> f =
			boost::bind(&grabberCallback, _1);
		openniGrabber->registerCallback(f);
	}

	viewer = createViewer();

	if (justVisualize)
	{
		if (noColor)
			viewer->showCloud(cloudptr);
		else 
			viewer->showCloud(cloudptr);
			
	}
	else openniGrabber->start();

	// Main loop.
	while (! viewer->wasStopped())
		boost::this_thread::sleep(boost::posix_time::seconds(1));

	if (! justVisualize)
		openniGrabber->stop();
}
