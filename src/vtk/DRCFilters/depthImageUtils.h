#ifndef DEPTHIMAGEUTILS_H_
#define DEPTHIMAGEUTILS_H_

#include <iostream>
#include <vector>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>

#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>
#ifndef Q_MOC_RUN
#include <pcl/filters/passthrough.h>
#endif

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <multisense_image_utils/multisense_image_utils.hpp>

class DepthImageUtils
{
public:
  DepthImageUtils();

  void unpackImage(const sensor_msgs::ImageConstPtr& image_a, const sensor_msgs::CameraInfoConstPtr &info_a,
                   const sensor_msgs::ImageConstPtr& image_b, const sensor_msgs::CameraInfoConstPtr &info_b,
                   pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud);

  void unpackMultisense(const uint8_t* depth_data, const uint8_t* color_data, int h, int w, cv::Mat_<double> repro_matrix,
                        pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_rgb = true, std::string depth_encoding = "Unknown");

  void SetDecimate(int decimate);

  void SetRemoveSize(int size_threshold);

  void SetRangeThreshold(float range_threshold);

private:

  int decimate_;

  // Disconnected/Small components filtering:
  int size_threshold_; // in pixels
  float depth_threshold_; // in m, not exposed yet
  float range_threshold_;

  uint8_t* rgb_buf_ ;
  uint8_t* depth_buf_;

  multisense_image_utils miu_;
};

#endif // DEPTHIMAGEUTILS_H_
