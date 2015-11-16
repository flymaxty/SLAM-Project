#ifndef __POINTCLOUDPROCESS_H__
#define __POINTCLOUDPROCESS_H__

#include <opencv2/opencv.hpp>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "Kinect_Input.h"
#include "DataCenter.hpp"

using namespace cv;
using namespace std;
using namespace pcl;

class PointCloudProcess
{
	public:
		cv::Mat m_cameraMatrix;
		pcl::visualization::CloudViewer viewer;
		//pcl::visualization::PCLVisualizer viewer;
	public:
		PointCloudProcess();
		bool pointcloud_generation(FRAME& in_frame);
		bool showPointCloud(FRAME& in_frame);
};

#endif	/*__POINTCLOUDPROCESS_H__*/
