#include "vtkRosPointCloudSubscriber.h"

#include "vtkTransform.h"
#include "vtkTransformPolyDataFilter.h"
#include <vtkImageData.h>
#include "vtkNew.h"

#include <vtkIdList.h>
#include <vtkCellArray.h>
#include <vtkPoints.h>
#include <vtkDoubleArray.h>
#include <vtkFloatArray.h>
#include <vtkPointData.h>
#include <vtkPolygon.h>
#include <vtkTriangle.h>
#include "vtkRosPointCloudConversions.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkRosPointCloudSubscriber);

vtkRosPointCloudSubscriber::vtkRosPointCloudSubscriber()
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = 0;
    ros::init(argc, argv, "director", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);
  }
  tfListener_ = boost::make_shared<tf::TransformListener>();
  frame_id_ = "no_frame";
  sec_ = 0;
  nsec_ = 0;
}

vtkRosPointCloudSubscriber::~vtkRosPointCloudSubscriber() {
  ros::shutdown();
}

void vtkRosPointCloudSubscriber::Start(std::string topic_name) {

  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe(topic_name, 1000, &vtkRosPointCloudSubscriber::PointCloudCallback, this));

  if (!spinner_) {
    spinner_ = boost::make_shared<ros::AsyncSpinner>(1);
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

  //we can't modify dataset_ if it's being copied in GetPointCloud
  std::lock_guard<std::mutex> lock(mutex_);
  dataset_ = ConvertPointCloud2ToVtk(input_);
}

void transformPolyData(vtkPolyData* polyDataSrc, vtkPolyData* polyDataDst,
                                               const vtkSmartPointer<vtkTransform>& transform)
{
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputData(polyDataSrc);
  transformFilter->Update();
  polyDataDst->DeepCopy(transformFilter->GetOutput());
}

vtkSmartPointer<vtkTransform> transformFromPose(const tf::StampedTransform& rosTransform)
{
  vtkSmartPointer<vtkTransform> t = vtkSmartPointer<vtkTransform>::New();
  //translation
  tf::Vector3 origin = rosTransform.getOrigin();
  double translation[3] = {origin.getX(), origin.getY(), origin.getZ()};
  //rotation
  double theta = rosTransform.getRotation().getAngle();
  tf::Vector3 tfAxis= rosTransform.getRotation().getAxis();
  double axis[3] = {tfAxis.getX(), tfAxis.getY(), tfAxis.getZ()};

  t->Identity();
  t->Translate(translation);
  t->RotateWXYZ(theta * 180./M_PI, axis);
  return t;
}

void vtkRosPointCloudSubscriber::GetPointCloud(vtkPolyData* polyData)
{
  if (!polyData || !dataset_)
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in PointCloudCallback
  std::lock_guard<std::mutex> lock(mutex_);
  tf::StampedTransform transform;
  vtkSmartPointer<vtkTransform> sensorToLocalTransform = vtkSmartPointer<vtkTransform>::New();
  sensorToLocalTransform->Identity();
  try {
    ros::Time t(GetSec(), GetNsec());
    tfListener_->lookupTransform("/map", GetFrameId(), t, transform);
    sensorToLocalTransform = transformFromPose(transform);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
  }
  transformPolyData(dataset_, polyData, sensorToLocalTransform);
}

void vtkRosPointCloudSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
