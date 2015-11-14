#include "PointCloudProcess.h"

using namespace std;
using namespace cv;
using namespace pcl;

PointCloudProcess::PointCloudProcess()
{
	std::cout << "Init PointCloudProcess......";

	//viewer = new pcl::visualization::CloudViewer("PointCloud");

	std::cout << "Done!" << std::endl;
}

bool PointCloudProcess::pointcloud_generation(FRAME& in_frame)
{
    //pcl::PointCloud<pcl::PointXYZRGBA>::Ptr tmpPointCloud = (pcl::PointCloud<pcl::PointXYZRGBA>::Ptr)(&in_frame.pointCloud);
    pcl::PointXYZRGB tmpPoint3d;
    ushort tmpDepth;

    double cx = 320;
	double cy = 240;
	double fx = 525;
	double fy = 525;
	double scale = 1000;

    for (int m = 0; m < in_frame.depthImage.rows; m++)
        for (int n=0; n < in_frame.depthImage.cols; n++)
        {
            // 获取深度图中(m,n)处的值
            ushort tmpDepth = in_frame.depthImage.ptr<ushort>(m)[n];
            // d 可能没有值，若如此，跳过此点
            if (tmpDepth == 0)
                continue;

            // d 存在值，则向点云增加一个点
            // 计算这个点的空间坐标
            tmpPoint3d.z = double(tmpDepth) / scale;
            tmpPoint3d.x = (n - cx) * tmpPoint3d.z / fx;
            tmpPoint3d.y = (m - cy) * tmpPoint3d.z / fy;
            
            // 从rgb图像中获取它的颜色
            // rgb是三通道的BGR格式图，所以按下面的顺序获取颜色
            tmpPoint3d.b = in_frame.rgbImage.ptr<uchar>(m)[n*3];
            tmpPoint3d.g = in_frame.rgbImage.ptr<uchar>(m)[n*3+1];
            tmpPoint3d.r = in_frame.rgbImage.ptr<uchar>(m)[n*3+2];

            // 把tmpPoint3d加入到点云中
            in_frame.pointCloud.points.push_back( tmpPoint3d );
        }

    // 设置并保存点云
    in_frame.pointCloud.height = 1;
   	in_frame.pointCloud.width = in_frame.pointCloud.points.size();
    in_frame.pointCloud.is_dense = false;

    //in_frame.pointCloud = n_frame.pointCloud;
    return true;
}

bool PointCloudProcess::showPointCloud(FRAME& in_frame)
{
	viewer.setWindowName("3D Viewer");
	PointCloud<pcl::PointXYZRGB>::Ptr g_cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgba(g_cloud);
	viewer.setBackgroundColor (255, 255, 255);
	viewer.addPointCloud<pcl::PointXYZRGB> (g_cloud, rgba, "sample cloud");
	viewer.setPointCloudRenderingProperties (pcl::visualization::PCL_VISUALIZER_POINT_SIZE, 3, "sample cloud");
	viewer.addCoordinateSystem (2.0);

	PointCloud<pcl::PointXYZRGB>::Ptr tmp = (PointCloud<pcl::PointXYZRGB>::Ptr)&in_frame.pointCloud;
	//viewer->showCloud(tmp);
	pcl::visualization::PointCloudColorHandlerRGBField<pcl::PointXYZRGB> rgb(tmp);
	viewer.updatePointCloud(tmp, rgb, "sample cloud");
	viewer.spinOnce (10);
	cv::waitKey(0);
}