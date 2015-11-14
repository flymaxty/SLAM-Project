#include "Detect.h"

Detect::Detect()
{
	std::cout << "Init Detect class.......";
	
	m_detector = xfeatures2d::SURF::create();

	m_goodMatchMinValue = 0.002;
	m_goodMatchDistanceTimes = 3;
	
	m_kfthreshold = 0.1;

	//namedWindow("KeyPoint_Show");
	//namedWindow("Match_Show");

	std::cout << "Done!" << std::endl;
}

vector<cv::KeyPoint> Detect::kp_extract(Ptr<xfeatures2d::SURF> det, Mat img)
{
	vector<cv::KeyPoint> tmp;
	
	det->detect(img, tmp);
	
	return(tmp);
}

Mat Detect::descriptor_compute(Ptr<xfeatures2d::SURF> det, Mat img, vector<cv::KeyPoint> kp)
{
	Mat tmp;
	
	det->compute(img, kp, tmp);
	
	return(tmp);
}		

vector<DMatch> Detect::img_match(BFMatcher matcher, Mat last, Mat recent)
{
	vector<DMatch> tmp;
	
	matcher.match(last, recent, tmp);
	
	return(tmp);
}

void Detect::surf_show(Mat last, Mat recent, vector<KeyPoint> kpl, vector<KeyPoint> kpr, vector<DMatch> points)
{
	Mat kp_show, match_show;
	
	drawKeypoints(last, kpl, kp_show, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawKeypoints(recent, kpr, kp_show, Scalar::all(-1), DrawMatchesFlags::DEFAULT);
	drawMatches(last, kpl, recent, kpr, points, match_show);
	
	imshow("KeyPoint_Show", kp_show);
	imshow("Match_Show", match_show);
}

void Detect::ransac_detect(vector<DMatch>& in_match, FRAME* in_frame, FRAME* in_lastFrame)
{
	double norm;
	vector<Point2f> pointCloud;
	vector<Point3f> lastPointCloud;

	cv::Point2f tmpPoint2d;
	double pointDepth;
	cv::Point3f tmpLastPoint3d;

    ushort tmpx, tmpy, tmpDepth;

    double cx = 320;
	double cy = 240;
	double fx = 525;
	double fy = 525;
	double scale = 1000;

	for (uint16_t i=0; i<in_match.size(); i++)
	{
		tmpx = in_frame->keyPoints[in_match[i].queryIdx].pt.x;
		tmpy = in_frame->keyPoints[in_match[i].queryIdx].pt.y;
		tmpDepth = in_frame->depthImage.ptr<ushort>(tmpy)[tmpx];

		pointDepth = double(tmpDepth) / scale;
		tmpPoint2d.x = (tmpx - cx) * pointDepth / fx;
		tmpPoint2d.y = (tmpy - cy) * pointDepth / fy;

		pointCloud.push_back(tmpPoint2d);
	}

	for (uint16_t i=0; i<in_match.size(); i++)
	{
		tmpx = in_lastFrame->keyPoints[in_match[i].queryIdx].pt.x;
		tmpy = in_lastFrame->keyPoints[in_match[i].queryIdx].pt.y;
		tmpDepth = in_lastFrame->depthImage.ptr<ushort>(tmpy)[tmpx];

		tmpLastPoint3d.z = double(tmpDepth) / scale;
		tmpLastPoint3d.x = (tmpx - cx) * tmpLastPoint3d.z / fx;
		tmpLastPoint3d.y = (tmpy - cy) * tmpLastPoint3d.z / fy;

		lastPointCloud.push_back(tmpLastPoint3d);
	}

	//求解pnp
	bool result;
	result = solvePnPRansac(lastPointCloud, pointCloud, m_cameraMatrix, Mat(), rvec, tvec, false, 100, 8.0, 0.99, inliers);

	if (inliers.rows < 5)
	{
		cout << "Not Match" << endl;
		return;
	}

	norm = normofTransform(rvec, tvec);

	cout << "norm: " << norm << endl;
	cout << "rvec: " << rvec << endl;
	cout << "tvec: " << tvec << endl;

	if (norm >= 0.3) //0.3
	{
		cout << "Too Far Away" << endl;
		return;
	}

	if(norm >= 5) //5
	{
		cout << "Loop Too Far Away" << endl;
		return;
	}

	if (norm <= m_kfthreshold)
	{
		cout << "Too Close" << endl;
		return;
	}
}

double Detect::normofTransform(Mat nmt_rvec, Mat nmt_tvec )
{
	return fabs(min(cv::norm(nmt_rvec), 2*M_PI-cv::norm(nmt_rvec)))+ fabs(cv::norm(nmt_tvec));
}

void Detect::match_process()
{
	m_frame->keyPoints = kp_extract(m_detector, m_frame->rgbImage);
	m_frame->descriptions = descriptor_compute(m_detector, m_frame->rgbImage, m_frame->keyPoints);
	
	m_matchpoint = img_match(m_matcher, m_lastFrame->descriptions, m_frame->descriptions);
	
	//surf_show(m_lastFrame->rgbImage, m_frame->rgbImage, m_lastFrame->keyPoints, m_frame->keyPoints, m_matchpoint);
	std::cout << m_matchpoint.size() << std::endl;
	if(m_matchpoint.size()>50)
	{
		ransac_detect(m_matchpoint, m_lastFrame, m_frame);
	}
	else
	{
		std::cout << "m_matchpoint size too small" << std::endl;
	}
}

void Detect::detect_process(FRAME* in_frame, FRAME* in_lastFrame)
{
	m_frame = in_frame;
	m_lastFrame = in_lastFrame;

	match_process();
}
