#ifndef __DATACENTER_HPP__
#define __DATACENTER_HPP__

#include <vector>

#include <opencv2/opencv.hpp>
#include <pcl/point_types.h>
#include <pcl/point_cloud.h>

//帧结构
struct FRAME
{
    cv::Mat rgbImage;
    cv::Mat depthImage;
    cv::Mat descriptions;
    std::vector<cv::KeyPoint> keyPoints;

    cv::Mat m_inlier;

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
};

//PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

#endif	/*__DATACENTER_HPP__*/