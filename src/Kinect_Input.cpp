#include <iostream>
#include <vector>

#include <opencv2/opencv.hpp>
#include <libfreenect/libfreenect.hpp>

#include "Kinect_Input.h"

using namespace cv;
using namespace std;

myMutex::myMutex()
{
	pthread_mutex_init( &m_mutex, NULL );
}

void myMutex::lock()
{
	pthread_mutex_lock( &m_mutex );
}

void myMutex::unlock()
{
	pthread_mutex_unlock( &m_mutex );
}

KinectInput::KinectInput(freenect_context *_ctx, int _index):
			Freenect::FreenectDevice(_ctx, _index),
			m_buffer_depth(freenect_find_video_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_VIDEO_RGB).bytes), 
			m_buffer_rgb(freenect_find_depth_mode(FREENECT_RESOLUTION_MEDIUM, FREENECT_DEPTH_REGISTERED).bytes / 2),
			m_gamma(2048), m_new_rgb_frame(false), 
			m_new_depth_frame(false), depthMat(Size(640,480),CV_16UC1), 
			rgbMat(Size(640,480), CV_8UC3, Scalar(0)), ownMat(Size(640,480),CV_8UC3,Scalar(0))
{	
	for( unsigned int i = 0 ; i < 2048 ; i++)
	{
		float v = i/2048.0;
		v = std::pow(v, 3)* 6;
		m_gamma[i] = v*6*256;
	}

	double cx = 320;
	double cy = 240;
	double fx = 525;
	double fy = 525;

	cameraMatrix = cv::Mat(3, 3, CV_64F);
	cameraMatrix.setTo(0);
	cameraMatrix.at<double>(0,0) = fx;
	cameraMatrix.at<double>(0,2) = cx;
	cameraMatrix.at<double>(1,1) = fy;
	cameraMatrix.at<double>(1,2) = cy;
	cameraMatrix.at<double>(2,2) = 1;

	std::cout << cameraMatrix << std::endl;
}
		
// Do not call directly even in child
void KinectInput::VideoCallback(void* _rgb, uint32_t timestamp)
{
	//std::cout << "RGB callback" << std::endl;
	m_rgb_mutex.lock();
	uint8_t* rgb = static_cast<uint8_t*>(_rgb);
	rgbMat.data = rgb;
	m_new_rgb_frame = true;
	m_rgb_mutex.unlock();
}

// Do not call directly even in child
void KinectInput::DepthCallback(void* _depth, uint32_t timestamp)
{
	//std::cout << "Depth callback" << std::endl;
	m_depth_mutex.lock();
	uint16_t* depth = static_cast<uint16_t*>(_depth);
	depthMat.data = (uchar*) depth;
	m_new_depth_frame = true;
	m_depth_mutex.unlock();
}

bool KinectInput::getVideo(Mat& output)
{
	m_rgb_mutex.lock();
	if(m_new_rgb_frame) {
		cv::cvtColor(rgbMat, output, CV_RGB2BGR);
		m_new_rgb_frame = false;
		m_rgb_mutex.unlock();
		return true;
	} else {
		m_rgb_mutex.unlock();
		return false;
	}
}

bool KinectInput::getDepth(Mat& output)
{
		m_depth_mutex.lock();
		if(m_new_depth_frame) {
			depthMat.copyTo(output);
			m_new_depth_frame = false;
			m_depth_mutex.unlock();
			return true;
		} else {
			m_depth_mutex.unlock();
			return false;
		}
}