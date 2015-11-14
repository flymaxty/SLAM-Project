#ifndef DETECT_H
#define DETECT_H

#include <iostream>
#include <string>
#include <vector> 

#include "opencv2/opencv.hpp"
#include "opencv2/xfeatures2d.hpp"

#include "Kinect_Input.h"
#include "DataCenter.hpp"

using namespace cv;
using namespace std;

class Detect
{
	public:
		//Data center
		FRAME* m_frame;
		FRAME* m_lastFrame;

		//Features Detect
		BFMatcher m_matcher;
		Ptr<xfeatures2d::SURF> m_detector;
		vector<DMatch> m_matchpoint;

		std::vector<cv::DMatch> m_goodMatches;
		std::vector<cv::KeyPoint> m_goodObjectKeypoints;
		std::vector<cv::KeyPoint> m_goodSceneKeypoints;
		std::vector<cv::Point2f> m_goodScenePoints;
		double m_goodMatchMinValue;
		double m_goodMatchDistanceTimes;

		double m_kfthreshold;

		cv::Mat rvec, tvec, inliers;
		cv::Mat m_cameraMatrix;
	public:
		Detect();
		vector<cv::KeyPoint> kp_extract(Ptr<xfeatures2d::SURF> det, Mat img);
		Mat descriptor_compute(Ptr<xfeatures2d::SURF> det, Mat img, vector<cv::KeyPoint> kp);
		vector<DMatch> img_match(BFMatcher match, Mat last, Mat recent);
		void surf_show(Mat last, Mat recent, vector<KeyPoint> kpl, vector<KeyPoint> kpr, vector<DMatch> points);
		double normofTransform(Mat nmt_rvec, Mat nmt_tvec );
		void detect_process(FRAME* in_frame, FRAME* in_lastFrame);
		void match_process();
		void ransac_detect(vector<DMatch>& in_match, FRAME* in_frame, FRAME* in_lastFrame);
};
#endif
