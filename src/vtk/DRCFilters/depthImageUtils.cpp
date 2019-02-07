#include "depthImageUtils.h"

#include <sensor_msgs/image_encodings.h>

DepthImageUtils::DepthImageUtils()
{
  decimate_ = 2;
  range_threshold_ = 5.0;
  size_threshold_ = 1000; // in pixels
  depth_threshold_ = 1000.0; // in m
}

void DepthImageUtils::unpackImage(const sensor_msgs::ImageConstPtr& image_a, const sensor_msgs::CameraInfoConstPtr& info_a,
                                  const sensor_msgs::ImageConstPtr &image_b, const sensor_msgs::CameraInfoConstPtr& info_b,
                                  pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud)
{

  sensor_msgs::ImageConstPtr image_rgb, image_depth;
  sensor_msgs::CameraInfoConstPtr info_depth;
  if (sensor_msgs::image_encodings::numChannels(image_a->encoding) == 1
      && sensor_msgs::image_encodings::numChannels(image_b->encoding) > 1)
  {
    image_rgb = image_b;
    image_depth = image_a;
    info_depth = info_a;
  } else if (sensor_msgs::image_encodings::numChannels(image_b->encoding) == 1
             && sensor_msgs::image_encodings::numChannels(image_a->encoding) > 1)
  {
    image_rgb = image_a;
    image_depth = image_b;
    info_depth = info_b;
  } else
  {
    ROS_ERROR("Unknown image types");
    return;
  }

  double baseline = 0.07; // an approximate value for multisense
  double cx = info_depth->K[2];
  double cy = info_depth->K[5];
  double f = info_a->K[0];
  cv::Mat_<double> Q(4, 4, 0.0);
  Q(0,0) =  1.0;
  Q(1,1) = 1.0;
  Q(3,2) = 1.0 / baseline;
  Q(0,3) = -cx;
  Q(1,3) = -cy;
  Q(2,3) = f;
  Q(3,3) = 0;//(stereo_params_.right.cx - stereo_params_.left.cx ) / baseline;

  depth_buf_ = const_cast<uint8_t*>(&image_depth->data[0]);
  rgb_buf_ = const_cast<uint8_t*>(&image_rgb->data[0]);

  unpackMultisense(depth_buf_, rgb_buf_, image_a->height, image_a->width, Q, cloud, true, image_depth->encoding);

  if (range_threshold_ >= 0) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud(cloud);
    pass.setFilterFieldName("z");
    pass.setFilterLimits(0.0, range_threshold_);
    pass.filter(*cloud);
  }
}

//untouched
void DepthImageUtils::unpackMultisense(const uint8_t* depth_data, const uint8_t* color_data, int h, int w, cv::Mat_<double> repro_matrix,
                                       pcl::PointCloud<pcl::PointXYZRGB>::Ptr &cloud, bool is_rgb, std::string depth_encoding)
{

  if (depth_encoding.compare("mono16") == 0){ // mfallon: I think this is what the Multisense disparity format is. Not 100%

    // Convert Carnegie disparity format into floating point disparity. Store in local buffer
    cv::Mat disparity_orig_temp = cv::Mat::zeros(h,w,CV_16UC1); // h,w
    //  const uint8_t* raw_data= stereob_->getDisparity();//= 3;//msg->images[1].data.data();
    disparity_orig_temp.data = (uchar*) depth_data;   // ... is a simple assignment possible?

    // Remove disconnect components. TODO: if needed this can also be used for the depth data
    if (size_threshold_ > 0){
      // Distance threshold conversion:
      float k00 = 1/repro_matrix(2,3);
      float baseline = 1/repro_matrix(3,2);
      float mDisparityFactor = 1/k00/baseline;
      float thresh = 16.0/mDisparityFactor/depth_threshold_;
      miu_.removeSmall(disparity_orig_temp, thresh, size_threshold_);
    }

    //std::copy(msg->images[1].data.data()             , msg->images[1].data.data() + (msg->images[1].size) ,
    //          disparity_orig_temp.data);

    // disparity_orig_temp.data = msg->images[1].data.data();   // ... is a simple assignment possible?
    cv::Mat_<float> disparity_orig(h, w);
    disparity_orig = disparity_orig_temp;

    std::vector<float> disparity_buf;
    disparity_buf.resize(h * w);
    cv::Mat_<float> disparity(h, w, &(disparity_buf[0]));
    disparity = disparity_orig / 16.0;
    // Allocate buffer for reprojection output
    std::vector<cv::Vec3f> points_buf;
    points_buf.resize(h * w);
    cv::Mat_<cv::Vec3f> points(h, w, &(points_buf[0]));
    // Do the reprojection in open space
    static const bool handle_missing_values = true;
    cv::reprojectImageTo3D(disparity, points, repro_matrix, handle_missing_values);

    cloud->width    =(int) std::ceil(w/ (double) decimate_);
    cloud->height   =(int) std::ceil(h/ (double) decimate_);
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<h; v=v+ decimate_) { // t2b
      for(int u=0; u<w; u=u+decimate_ ) {  //l2r
          // cout <<j2 << " " << v << " " << u << " | " <<  points(v,u)[0] << " " <<  points(v,u)[1] << " " <<  points(v,u)[1] << "\n";
          cloud->points[j2].x = points(v,u)[0];
          cloud->points[j2].y = points(v,u)[1];
          cloud->points[j2].z = points(v,u)[2];
          int pixel =v*w + u;
          if (!is_rgb){//color_provided){ // Assumed gray:
            cloud->points[j2].r =color_data[pixel];
            cloud->points[j2].g =color_data[pixel];
            cloud->points[j2].b =color_data[pixel];
          }else{ // RGB:
            cloud->points[j2].r =color_data[pixel*3];
            cloud->points[j2].g =color_data[pixel*3 +1];
            cloud->points[j2].b =color_data[pixel*3 +2];
          }
          j2++;
      }
    }
  }else{
    short* depths_short = (short*) depth_data; // m case
    float* depths_float = (float*) depth_data; // mm case


    // Recover focal lengths from repro_matrix - this assumes fx=fy
    // and the reprojection formula. Yuck!
    double fx = repro_matrix(2,3);
    double fy = repro_matrix(2,3);
    double cx = -repro_matrix(0,3);
    double cy = -repro_matrix(1,3);

    cloud->width    =(int) std::ceil(w/ (double) decimate_) ;
    cloud->height   =(int) std::ceil(h/ (double) decimate_);
    cloud->is_dense = true;
    cloud->points.resize (cloud->width * cloud->height);
    int j2=0;
    for(int v=0; v<h; v=v+ decimate_) { // t2b
      for(int u=0; u<w; u=u+decimate_ ) {  //l2r

          int pixel = v*w +u;
          float z = 0;
          if (depth_encoding.compare("16UC1") == 0) // Depth images, in mm (real realsense data)
            z = (float) depths_short[pixel]/1000.0;
          else if (depth_encoding.compare("32FC1") == 0) // Depth images in m (simulated gazebo realsense data)
            z = (float) depths_float[pixel];
          else // TODO: should this exit here?
            std::cout << "ERROR: Encoding not understood: " << depth_encoding << "\n";

          cloud->points[j2].x =( z * (u  - cx))/ fx ;
          cloud->points[j2].y =( z * (v  - cy))/ fy ;
          cloud->points[j2].z =z;

          if (!is_rgb){//color_provided){ // Assumed gray:
            cloud->points[j2].r =color_data[pixel];
            cloud->points[j2].g =color_data[pixel];
            cloud->points[j2].b =color_data[pixel];
          }else{ // RGB:
            cloud->points[j2].r =color_data[pixel*3];
            cloud->points[j2].g =color_data[pixel*3 +1];
            cloud->points[j2].b =color_data[pixel*3 +2];
          }
          j2++;
      }
    }


  }

//  cout << "points: " << cloud->points.size() << "\n";
//  pcl::io::savePCDFileASCII ("test_pcd.pcd", *cloud);
}

void DepthImageUtils::SetDecimate(int decimate)
{
  decimate_ = decimate;
}

void DepthImageUtils::SetRemoveSize(int size_threshold)
{
  size_threshold_ = size_threshold;
}

void DepthImageUtils::SetRangeThreshold(float range_threshold)
{
  range_threshold_ = range_threshold;
}
