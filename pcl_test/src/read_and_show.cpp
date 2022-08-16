// load pc ->  do [R T] -> show
#include <iostream>
#include <string>
#include <vector>
#include <ctime>

#include <ros/ros.h>
#include <sensor_msgs/PointCloud2.h>
#include <pcl_conversions/pcl_conversions.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_cloud.h>
#include <pcl/point_types.h>
#include <pcl/common/common.h>
#include <pcl/common/transforms.h>
#include <pcl/ModelCoefficients.h>
#include <pcl/sample_consensus/method_types.h>
#include <pcl/sample_consensus/model_types.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>
#include <pcl/segmentation/progressive_morphological_filter.h>
#include <pcl/segmentation/sac_segmentation.h>
#include <pcl/filters/voxel_grid.h>
#include <pcl/filters/statistical_outlier_removal.h>
#include <pcl/filters/extract_indices.h>


void readPCDAndShow(std::string path)
{
    pcl::PointCloud<pcl::PointXYZ>::Ptr cloudin(new pcl::PointCloud<pcl::PointXYZ>);
    pcl::io::loadPCDFile(path,*cloudin);

    // //点云旋转
	// Eigen::Matrix4f transform_1 = Eigen::Matrix4f::Zero();//单位矩阵

	// //给矩阵赋值
    // float theta = M_PI;
	// transform_1(0, 0) = cos(theta);
	// transform_1(0, 2) = sin(theta);
	// transform_1(2, 0) = -sin(theta);
	// transform_1(2, 2) = cos(theta);
    // transform_1(1, 1) = 1.0;
	// transform_1(0, 3) = 1485.35;
    // transform_1(1, 3) = 0;
    // transform_1(2, 3) = 2090.35;
	// transform_1(3, 3) = 1.0;
    // std::cout << transform_1 << std::endl;

    // pcl::PointCloud<pcl::PointXYZ>::Ptr cloud(new pcl::PointCloud<pcl::PointXYZ>);
	// pcl::transformPointCloud(*cloudin, *cloud, transform_1);

    // pcl::PointXYZ min_p, max_p;
    // pcl::getMinMax3D(*cloud, min_p, max_p);

    // std::cout << min_p << " " << max_p << std::endl;

    // show
    boost::shared_ptr<pcl::visualization::PCLVisualizer> viewer(new pcl::visualization::PCLVisualizer("show"));
    
    viewer->addCoordinateSystem(1000, 0.0, 0.0, 0.0);
    viewer->setBackgroundColor(255,255,255);
    pcl::visualization::PointCloudColorHandlerCustom<pcl::PointXYZ> cloud_color(cloudin,0,0,255);
    viewer->addPointCloud(cloudin,cloud_color,"cloud");

	while (!viewer->wasStopped())
	{
		viewer->spinOnce(100);
		boost::this_thread::sleep(boost::posix_time::microseconds(100000));
	}
}

int main (int argc, char **argv)
{
    readPCDAndShow(argv[1]);

    return 0;
}