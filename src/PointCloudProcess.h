#ifndef POINTCLOUDPROCESS_H
#define POINTCLOUDPROCESS_H

#include <iostream>
#include <string>
#include <vector> 
#include <pthread.h>

#include "opencv2/opencv.hpp"

#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/passthrough.h>

#include "Kinect_Input.h"
#include "DataCenter.hpp"

using namespace cv;
using namespace std;
using namespace pcl;

class PointCloudProcess
{
	public:
		cv::Mat m_cameraMatrix;
		//pcl::visualization::CloudViewer* viewer;
		pcl::visualization::PCLVisualizer viewer;
	public:
		PointCloudProcess();
		bool pointcloud_generation(FRAME& in_frame);
		bool showPointCloud(FRAME& in_frame);
};

#endif
