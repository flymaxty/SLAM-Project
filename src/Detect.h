#ifndef __DETECT_H__
#define __DETECT_H__

#include <vector> 

#include <opencv2/opencv.hpp>
#include <opencv2/xfeatures2d.hpp>

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
		bool getGoodMatches(vector<DMatch>& in_matches, Mat& in_lastKeypoints, Mat& in_keypoints,
									vector<DMatch>& in_goodMatches);
		void match_process();
		void ransac_detect(vector<DMatch>& in_match, FRAME* in_frame, FRAME* in_lastFrame);
};
#endif /*__DETECT_H__*/
