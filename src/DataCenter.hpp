#ifndef DATACENTER_HPP
#define DATACENTER_HPP

#include <iostream>
#include <vector>

#include "opencv2/opencv.hpp"
#include <pcl-1.8/pcl/io/pcd_io.h>
#include <pcl-1.8/pcl/point_types.h>
#include <pcl-1.8/pcl/visualization/pcl_visualizer.h>
#include <pcl-1.8/pcl/visualization/cloud_viewer.h>
#include <pcl-1.8/pcl/common/transforms.h>
#include <pcl-1.8/pcl/filters/voxel_grid.h>
#include <pcl-1.8/pcl/filters/passthrough.h>

// 帧结构
struct FRAME
{
    cv::Mat rgbImage;
    cv::Mat depthImage;
    cv::Mat descriptions;
    std::vector<cv::KeyPoint> keyPoints;

    cv::Mat m_inlier;

    pcl::PointCloud<pcl::PointXYZRGB> pointCloud;
};

// PnP 结果
struct RESULT_OF_PNP
{
    cv::Mat rvec, tvec;
    int inliers;
};

#endif