#ifndef KINECT_INPUT_H
#define KINECT_INPUT_H

#include <iostream>
#include <vector>
#include <pthread.h>

#include "opencv2/opencv.hpp"
#include "libfreenect/libfreenect.hpp"

using namespace cv;

class myMutex {
	public:
		myMutex();
		void lock();
		void unlock();
	private:
		pthread_mutex_t m_mutex;
};

class KinectInput : public Freenect::FreenectDevice {
public:
			cv::Mat cameraMatrix;

public:
		KinectInput(freenect_context *_ctx, int _index);
		// Do not call directly even in child
		void VideoCallback(void* _rgb, uint32_t timestamp);	
		// Do not call directly even in child
		void DepthCallback(void* _depth, uint32_t timestamp);
		bool getVideo(cv::Mat& output);
		bool getDepth(cv::Mat& output);
		
private:
		std::vector<uint8_t> m_buffer_depth;
		std::vector<uint8_t> m_buffer_rgb;
		std::vector<uint16_t> m_gamma;
		cv::Mat depthMat;
		cv::Mat rgbMat;
		cv::Mat ownMat;
		myMutex m_rgb_mutex;
		myMutex m_depth_mutex;
		bool m_new_rgb_frame;
		bool m_new_depth_frame;
};

#endif