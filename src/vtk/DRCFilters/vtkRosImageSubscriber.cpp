#include "vtkRosImageSubscriber.h"

#include <opencv2/opencv.hpp>
#include <opencv2/highgui/highgui.hpp>

#include <Eigen/Geometry>

#include <transformPolyDataUtils.h>

#include <vtkObjectFactory.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>

vtkStandardNewMacro(vtkRosImageSubscriber);

vtkRosImageSubscriber::vtkRosImageSubscriber()
{
  if (!ros::isInitialized()) {
    ROS_DEBUG("WARNING: ROS not Initialized");
  }
  tf_listener_ = boost::make_shared<tf::TransformListener>();
}

vtkRosImageSubscriber::~vtkRosImageSubscriber()
{
  ros::shutdown();
}

void vtkRosImageSubscriber::Start(const std::string& image_topic, const std::string& image_transport,
                                       const std::string& info_topic)
{
  ros::NodeHandle node;

  image_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  it_ = boost::make_shared<image_transport::ImageTransport>(node);
  image_sub_->subscribe(*it_, ros::names::resolve(image_topic), 100, image_transport::TransportHints( image_transport ));

  info_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_sub_->subscribe(node, ros::names::resolve(info_topic), 100);

  sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> >(10);
  sync_->connectInput(*image_sub_, *info_sub_);
  sync_->registerCallback(boost::bind(&vtkRosImageSubscriber::ImageCallback, this, _1, _2));

}

void vtkRosImageSubscriber::Stop()
{
  image_sub_->unsubscribe();
  info_sub_->unsubscribe();
}

void vtkRosImageSubscriber::ImageCallback(const sensor_msgs::ImageConstPtr& image,
                                          const sensor_msgs::CameraInfoConstPtr& info)
{
   if (!body_to_camera_transform_)
   {
     //The transform between the camera and the base doesn't change so it's only computed once
     std::string frame_id = image->header.frame_id;
     tf::StampedTransform transform;
     ros::Time time = image->header.stamp;
     tf_listener_->waitForTransform(frame_id, "base", time, ros::Duration(2.0));
     try {
       tf_listener_->lookupTransform(frame_id, "base", time, transform);
       body_to_camera_transform_ = transformPolyDataUtils::transformFromPose(transform);
     }
     catch (tf::TransformException& ex){
       ROS_ERROR("%s",ex.what());
       return;
     }
   }

   std::lock_guard<std::mutex> lock(mutex_);
   time_ = image->header.stamp;
   camera_info_ = *info;
   dataset_ = vtkSmartPointer<vtkImageData>::New();
   dataset_->SetDimensions(image->width, image->height, 1);
   dataset_->AllocateScalars(VTK_UNSIGNED_CHAR, 3);

   unsigned char* vtk_data = static_cast<unsigned char*>(dataset_->GetScalarPointer(0, 0, 0));

   memcpy(vtk_data, image->data.data(), image->step*image->height );
}

unsigned long long vtkRosImageSubscriber::GetCurrentImageTime() const
{
  return time_.toNSec();
}

void vtkRosImageSubscriber::GetImage(vtkImageData* image)
{
  std::lock_guard<std::mutex> lock(mutex_);
  image->DeepCopy(dataset_);
}

void vtkRosImageSubscriber::ComputeTextureCoords(const std::string& camera_name, vtkPolyData* poly_data) const
{
  std::lock_guard<std::mutex> lock(mutex_);

  if (dataset_ == 0)
  {
    ROS_ERROR("ComputeTextureCoords : no image received");
    return;
  }

  if (camera_info_.distortion_model != "plumb_bob")
  {
    ROS_ERROR("Unknow distortion model");
    return;
  }

  int w = camera_info_.width;
  int h = camera_info_.height;

  std::string array_name = "tcoords_" + camera_name;
  vtkSmartPointer<vtkFloatArray> tcoords = vtkFloatArray::SafeDownCast(poly_data->GetPointData()->GetArray(array_name.c_str()));
  if (!tcoords)
  {
    tcoords = vtkSmartPointer<vtkFloatArray>::New();
    tcoords->SetName(array_name.c_str());
    tcoords->SetNumberOfComponents(2);
    tcoords->SetNumberOfTuples(poly_data->GetNumberOfPoints());
    poly_data->GetPointData()->AddArray(tcoords);

    tcoords->FillComponent(0, -1);
    tcoords->FillComponent(1, -1);
  }

  const vtkIdType num_points = poly_data->GetNumberOfPoints();
  std::vector<cv::Point3f> object_points;
  for (vtkIdType i = 0; i < num_points; ++i)
  {
    Eigen::Vector3d pt;
    poly_data->GetPoint(i, pt.data());
    object_points.push_back(cv::Point3f(pt[0], pt[1], pt[2]));
  }

  std::vector<cv::Point2f> image_points;
  cv::Mat_<double> camera_matrix(3, 3, 0.0);
  camera_matrix(0, 0) = camera_info_.K[0];
  camera_matrix(0, 2) = camera_info_.K[2];
  camera_matrix(1, 1) = camera_info_.K[4];
  camera_matrix(1, 2) = camera_info_.K[5];
  camera_matrix(2, 2) = 1;

  std::vector<double> dist_coeffs = {camera_info_.D[0], camera_info_.D[1], camera_info_.D[2],
                                    camera_info_.D[3], camera_info_.D[4]};
  cv::projectPoints(object_points, cv::Vec3f(0, 0, 0), cv::Vec3f(0, 0, 0), camera_matrix,
                   dist_coeffs, image_points);

  for (vtkIdType i = 0; i < num_points; ++i)
  {

    float u = image_points[i].x / (w-1);
    float v = image_points[i].y / (h-1);

    tcoords->SetComponent(i, 0, u);
    tcoords->SetComponent(i, 1, v);
  }
}

void vtkRosImageSubscriber::GetBodyToCameraTransform(vtkTransform* transform) const
{
  if (!body_to_camera_transform_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  transform->DeepCopy(body_to_camera_transform_);
}

void vtkRosImageSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
