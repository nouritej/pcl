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

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud1(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud2(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud3(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud4(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud5(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud6(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud7(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud8(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud9(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud10(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud11(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud12(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr empty(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud15(
		new pcl::PointCloud<pcl::PointXYZRGBA>);
pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloud16(
		new pcl::PointCloud<pcl::PointXYZRGBA>);

pcl::PointCloud<pcl::PointXYZRGBA>::Ptr cloudozel(
		new pcl::PointCloud<pcl::PointXYZRGBA>);

int main(int argc, char** argv) {
	TicToc tt;
	io::loadPCDFile < PointXYZRGBA > ("cloud1.pcd", *cloud1);
	io::loadPCDFile < PointXYZRGBA > ("cloud2.pcd", *cloud2);
	io::loadPCDFile < PointXYZRGBA > ("cloud3.pcd", *cloud3);
	io::loadPCDFile < PointXYZRGBA > ("cloud4.pcd", *cloud4);

	segmentation(*cloud1, *cloud1);
	segmentation(*cloud2, *cloud2);
	segmentation(*cloud3, *cloud3);
	segmentation(*cloud4, *cloud4);
	rotate_cl(*cloud1, *cloud1, 0, M_PI);
	rotate_cl(*cloud1, *cloud1, 2, M_PI / 2);
	rotate_cl(*cloud2, *cloud2, 0, M_PI);
	rotate_cl(*cloud2, *cloud2, 2, M_PI / 2);
	//rotate_cl(*cloud3, *cloud3, 0, M_PI);
	rotate_cl(*cloud3, *cloud3, 2, M_PI / 2);
	rotate_cl(*cloud4, *cloud4, 0, M_PI);
	rotate_cl(*cloud4, *cloud4, 2, M_PI / 2);
	pcl::PCDWriter writer;
	writer.write("segmented1.pcd", *cloud1, false);
	writer.write("segmented2.pcd", *cloud2, false);
	writer.write("segmented3.pcd", *cloud3, false);
	writer.write("segmented4.pcd", *cloud4, false);
	tt.tic();
	boundary_cl(*cloud1, *cloud5, 200, 1.2);
	print_info("[done, ");
	print_value("%g", tt.toc());
	boundary_cl(*cloud3, *cloud8, 200, 1.2);

	// sol sag ve orta parcaların alındığı yer
	filter_cl(*cloud5, *cloud6, *cloud7, *empty, 0, 0, 0);
	filter_cl(*cloud8, *cloud9, *cloud10, *empty, 0, 0, 0);
	// kesit alınıyor.
	section_cl(*cloud2, *cloud11, 1);
	// burda ufak bir sorun var !!
	section_cl(*cloud4, *cloud12, 0);

	writer.write("leftboundaryoffront.pcd", *cloud6, false);
	writer.write("rightboundaryoffront.pcd", *cloud7, false);
	writer.write("leftboundaryofback.pcd", *cloud9, false);
	writer.write("rightboundaryofback.pcd", *cloud10, false);/**/
	writer.write("centerboundaryofrightside.pcd", *cloud11, false);
	writer.write("centerboundaryofleftside.pcd", *cloud12, false);

	int default_min = 100;
	int default_max = 25000;
	double default_tolerance = 0.04;
	Eigen::Vector4f translation;
	Eigen::Quaternionf orientation;
	pcl::PCLPointCloud2::Ptr cloud66(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud77(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud99(new pcl::PCLPointCloud2);
	pcl::PCLPointCloud2::Ptr cloud110(new pcl::PCLPointCloud2);
	loadPCDFile("leftboundaryoffront.pcd", *cloud66, translation, orientation);
	cluster_cl(cloud66, *cloud6, translation, orientation, default_min,
			default_max, default_tolerance);
	writer.write("leftboundaryoffrontclean.pcd", *cloud6, false);
	loadPCDFile("rightboundaryoffront.pcd", *cloud77, translation, orientation);
	cluster_cl(cloud77, *cloud7, translation, orientation, default_min,
			default_max, default_tolerance);
	writer.write("rightboundaryoffrontclean.pcd", *cloud7, false);
	loadPCDFile("leftboundaryofback.pcd", *cloud99, translation, orientation);
	cluster_cl(cloud99, *cloud9, translation, orientation, default_min,
			default_max, default_tolerance);
	loadPCDFile("rightboundaryofback.pcd", *cloud110, translation, orientation);
	cluster_cl(cloud110, *cloud10, translation, orientation, default_min,
			default_max, 0.02);
	Eigen::Matrix4f t;
	t = icp_cl(*cloud6, *cloud11, *cloud11, 2, 0.05, 50, false);
	cout << t << endl;
	rotate_matrix(*cloud2, *cloud2, t);
	writer.write("rotatedleft-icp1.pcd", *cloud2, false);
	rotate_matrix(*cloud11, *cloud11, t);
	t = icp_cl(*cloud1, *cloud2, *cloud2, 2, 0.05, 50, false);
	cout << t << endl;
	rotate_matrix(*cloud2, *cloud2, t);
	rotate_matrix(*cloud11, *cloud11, t);
	writer.write("rotatedleft-icp2.pcd", *cloud2, false);
	t = icp_cl(*cloud7, *cloud12, *cloud12, 2, 0.05, 50, false);
	cout << t << endl;
	//Eigen::Matrix4f t(Eigen::Matrix4f::Identity());
	rotate_matrix(*cloud12, *cloud12, t);
	rotate_matrix(*cloud4, *cloud4, t);
	t = icp_cl(*cloud1, *cloud4, *cloud4, 2, 0.05, 50, false);
	cout << t << endl;
	rotate_matrix(*cloud4, *cloud4, t);
	rotate_matrix(*cloud12, *cloud12, t);
	writer.write("rotatedright.pcd", *cloud4, false);

	t = icp_cl(*cloud11, *cloud9, *cloud9, 3, 0.05, 50, false);
	cout << t << endl;
	rotate_matrix(*cloud3, *cloud3, t);
	rotate_matrix(*cloud9, *cloud9, t);
	//t=icp_cl(*cloud12,*cloud10,*cloud10,3,0.05,50,false);
	//cout << t << endl;
	//rotate_matrix(*cloud3, *cloud3, t);
	//rotate_matrix(*cloud10, *cloud10, t);	

	*cloud15 += *cloud1;
	writer.write("front.pcd", *cloud15, false);
	*cloud15 += *cloud2;
	writer.write("left.pcd", *cloud2, false);
	*cloudozel += *cloud2;
	*cloudozel += *cloud4;
	writer.write("right.pcd", *cloud4, false);
	*cloud15 += *cloud4;
	writer.write("frnt+left+right.pcd", *cloud15, false);
	t = icp_cl(*cloud15, *cloud3, *cloud3, 0.5, 0.05, 50, false);
	cout << t << endl;
	rotate_matrix(*cloud3, *cloud3, t);
	writer.write("back.pcd", *cloud3, false);
	*cloud15 += *cloud3;
	*cloudozel += *cloud3;

	t = icp_cl(*cloudozel, *cloud1, *cloud1, 0.05, 0.05, 100, false);
	cout << t << endl;
	rotate_matrix(*cloud1, *cloud1, t);
	*cloudozel += *cloud1;
	writer.write("model.pcd", *cloudozel, false);

	writer.write("model-notaligned.pcd", *cloud15, false);
	/*writer.write("cloudozel.pcd", *cloudozel, false);
	section_cb(*cloudozel,*cloud16);
	 writer.write ("cikti16.pcd", *cloud16, false);
	 pcl::PCLPointCloud2::Ptr cloud166 (new pcl::PCLPointCloud2);
	 loadPCDFile ("cikti16.pcd",*cloud166, translation, orientation);
	 cluster_cl(cloud166,*cloud16,translation, orientation,default_min, default_max, default_tolerance);	
	 writer.write ("cikti166.pcd", *cloud16, false);
	 loadPCDFile ("cikti166.pcd",*cloud166, translation, orientation);
	 cluster_cl(cloud166,*cloud16,translation, orientation,default_min, default_max, default_tolerance);
	 writer.write ("cikti16666.pcd", *cloud16, false);
	 convexcave(*cloud16,*cloud16,0.01,1);
	 writer.write ("cikti1666.pcd", *cloud16, false);
	 float sayi;
	 measure(*cloud16,sayi);
	 cout << sayi << endl;
	 */
	return 0;
}

