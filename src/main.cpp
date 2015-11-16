#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>
#include <pcl/visualization/pcl_visualizer.h>
#include <pcl/visualization/cloud_viewer.h>

#include "DataCenter.hpp"
#include "Kinect_Input.h"
#include "PointCloudProcess.h"
#include "Detect.h"

using namespace cv;
using namespace std;

std::vector<FRAME> frameList;
//pcl::PointCloud<pcl::PointXYZRGBA>::Ptr finalPointCloud;

Freenect::Freenect freenect;
PointCloudProcess pointCloudProcess;
Detect detect;

void viewerPsycho (pcl::visualization::PCLVisualizer& viewer)
{
    static unsigned count = 0;
    std::stringstream ss;
    ss << "Once per viewer loop: " << count++;
    viewer.removeShape ("text", 0);
    viewer.addText (ss.str(), 200, 300, "text", 0);
    
    //FIXME: possible race condition here:
    //user_data++;
    //PointCloud<pcl::PointXYZRGB>::Ptr tmp = (PointCloud<pcl::PointXYZRGB>::Ptr)&(frameList[frameList.size()-1].pointCloud);
    //viewer.updatePointCloud(tmp, "Simple Cloud Viewer");
}

int main(int argc, char **argv)
{
	bool die(false);
	
	Mat depthMat(Size(480,640),CV_16UC1);
	Mat rgbMat(Size(480,640),CV_8UC3,Scalar(0));
	
	//Init kinect and cameraMatrix
	KinectInput& device = freenect.createDevice<KinectInput>(0);
	pointCloudProcess.m_cameraMatrix = device.cameraMatrix;
	device.cameraMatrix.copyTo(detect.m_cameraMatrix);
	
	//Start kinect process
	device.startVideo();
	device.startDepth();
	while(!device.getVideo(rgbMat));
	while(!device.getDepth(depthMat));

	//Init pointcloud database
	//finalPointCloud = new pcl::PointCloud<pcl::PointXYZRGBA>;

	FRAME* tmpFrame;

	//First Frame
	tmpFrame = new FRAME;
	device.getVideo(rgbMat);
	device.getDepth(depthMat);
	rgbMat.copyTo(tmpFrame->rgbImage);
	depthMat.copyTo(tmpFrame->depthImage);
	tmpFrame->keyPoints = detect.kp_extract(detect.m_detector, tmpFrame->rgbImage);
	tmpFrame->descriptions = detect.descriptor_compute(detect.m_detector, tmpFrame->rgbImage, tmpFrame->keyPoints);
	pointCloudProcess.pointcloud_generation(*tmpFrame);
	frameList.push_back(*tmpFrame);

	std::cout << "============= Start =============" << std::endl;
	PointCloud<pcl::PointXYZRGB>::Ptr tmp = (PointCloud<pcl::PointXYZRGB>::Ptr)&(frameList[frameList.size()-1].pointCloud);
	//pointCloudProcess.viewer.showCloud(tmp);
	pointCloudProcess.viewer.runOnVisualizationThread (viewerPsycho);
	while (!die)
	{
		tmpFrame = new FRAME;
		//Get  kinect data
		device.getVideo(rgbMat);
		device.getDepth(depthMat);
		rgbMat.copyTo(tmpFrame->rgbImage);
		depthMat.copyTo(tmpFrame->depthImage);

		//Get pointcloud
		pointCloudProcess.pointcloud_generation(*tmpFrame);

		//Features Detect
		if(frameList.size() != 0)
			detect.detect_process(tmpFrame, &frameList[frameList.size()-1]);
		else
			std::cout << "frameList is empty!" << std::endl;

		//Save Frame
		frameList.push_back(*tmpFrame);

		//Show Image
		cv::imshow("rgb", rgbMat);
		cv::imshow("depth",depthMat);
		cv::waitKey(1);
		delete tmpFrame;
	}
	
	device.stopVideo();
	device.stopDepth();
	return 0;
}