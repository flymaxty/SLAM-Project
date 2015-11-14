#include <iostream>
#include "opencv2/opencv.hpp"
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
		//pointCloudProcess.showPointCloud(tmpFrame);

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