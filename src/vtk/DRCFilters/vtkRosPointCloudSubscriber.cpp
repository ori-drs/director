#include "vtkRosPointCloudSubscriber.h"

#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkImageData.h>
#include <vtkNew.h>

#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include <vtkRosPointCloudConversions.h>
#include <vtkObjectFactory.h>
#include <vtkAppendPolyData.h>

#include "transformPolyDataUtils.h"

vtkStandardNewMacro(vtkRosPointCloudSubscriber);

vtkRosPointCloudSubscriber::vtkRosPointCloudSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  tfListener_ = boost::make_shared<tf::TransformListener>();
  frame_id_ = "no_frame";
  fixed_frame_ = "map"; // or "odom"
  sec_ = 0;
  nsec_ = 0;
  number_of_point_clouds_ = 10;
}

vtkRosPointCloudSubscriber::~vtkRosPointCloudSubscriber() {
  ros::shutdown();
}

void vtkRosPointCloudSubscriber::Start(std::string topic_name) {

  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1000, &vtkRosPointCloudSubscriber::PointCloudCallback, this));

  if (!spinner_) {
    spinner_ = boost::make_shared<ros::AsyncSpinner>(4);
  }
  spinner_->start();
}

void vtkRosPointCloudSubscriber::Stop() {
  subscriber_->shutdown();
  spinner_.reset();
}

void vtkRosPointCloudSubscriber::PointCloudCallback(const sensor_msgs::PointCloud2Ptr& message) {
  input_ = message;
  frame_id_ = message->header.frame_id;
  sec_ = message->header.stamp.sec;
  nsec_ = message->header.stamp.nsec;

  //
  vtkSmartPointer<vtkTransform> sensorToLocalTransform = vtkSmartPointer<vtkTransform>::New();
  tf::StampedTransform transform;
  ros::Time time = message->header.stamp;
  tfListener_->waitForTransform(fixed_frame_, frame_id_, time, ros::Duration(10.0));
  try {
    tfListener_->lookupTransform(fixed_frame_, frame_id_, time, transform);
    sensorToLocalTransform = transformPolyDataUtils::transformFromPose(transform);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  vtkSmartPointer<vtkPolyData> poly_data = ConvertPointCloud2ToVtk(input_);
  vtkSmartPointer<vtkPolyData> transformed_poly_data = vtkSmartPointer<vtkPolyData>::New();
  transformPolyDataUtils::transformPolyData(poly_data, transformed_poly_data, sensorToLocalTransform);
  addPointCloud(transformed_poly_data);
}



void vtkRosPointCloudSubscriber::GetPointCloud(vtkPolyData* polyData)
{
  if (!polyData || !dataset_.size())
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in PointCloudCallback
  std::lock_guard<std::mutex> lock(mutex_);
  vtkSmartPointer<vtkAppendPolyData> append_poly_data = vtkSmartPointer<vtkAppendPolyData>::New();
  for(auto &data : dataset_)
  {
    append_poly_data->AddInputData(data);
  }
  append_poly_data->Update();
  polyData->DeepCopy(append_poly_data->GetOutput());
}

void vtkRosPointCloudSubscriber::addPointCloud(const vtkSmartPointer<vtkPolyData>& poly_data)
{
  dataset_.push_back(poly_data);

  while(dataset_.size() > number_of_point_clouds_)
  {
    dataset_.pop_front();
  }
}

void vtkRosPointCloudSubscriber::SetNumberOfPointClouds(int number_of_point_clouds)
{
  number_of_point_clouds_ = number_of_point_clouds;
}

void vtkRosPointCloudSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
