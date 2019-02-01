#include "vtkRosDepthImageSubscriber.h"
#include <transformPolyDataUtils.h>

#include <vtkObjectFactory.h>

vtkStandardNewMacro(vtkRosDepthImageSubscriber);

vtkRosDepthImageSubscriber::vtkRosDepthImageSubscriber()
{
  if (!ros::isInitialized()) {
    std::cout << "WARNING: vtkRosGridMapSubscriber: ROS not Initialized\n";
  }
  tf_listener_ = boost::make_shared<tf::TransformListener>();
  dataset_ = vtkSmartPointer<vtkPolyData>::New();

}

vtkRosDepthImageSubscriber::~vtkRosDepthImageSubscriber()
{
  ros::shutdown();
}

void vtkRosDepthImageSubscriber::Start()
{
  ros::NodeHandle node;
  std::string image_topic_a("/realsense_d435/color/image_raw"), image_topic_b("/realsense_d435/aligned_depth_to_color/image_raw");
  std::string info_topic_a, info_topic_b;
  std::string image_a_transport("compressed"), image_b_transport("compressedDepth");

  /*if (!node.getParam("image_a_transport", image_a_transport))
  {
    ROS_ERROR("Could not read `image_a_transport`.");
    exit(-1);
  }
  if (!node.getParam("image_b_transport", image_b_transport))
  {
    ROS_ERROR("Could not read `image_b_transport`.");
    exit(-1);
  }

  if (!node.getParam("image_a_topic", image_topic_a))
  {
    ROS_ERROR("Could not read `image_topic_a`.");
    exit(-1);
  }
  if (node.getParam("image_b_topic", image_topic_b)) {
    ROS_ERROR("Could not read `image_topic_b`.");
    exit(-1);
  }*/
  info_topic_a = "/realsense_d435/color/camera_info";
  info_topic_b = "/realsense_d435/aligned_depth_to_color/camera_info";

  image_a_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  image_b_sub_ = boost::make_shared<image_transport::SubscriberFilter>();
  it_ = boost::make_shared<image_transport::ImageTransport>(node);
  image_a_sub_->subscribe(*it_, ros::names::resolve(image_topic_a), 100, image_transport::TransportHints( image_a_transport ));
  image_b_sub_->subscribe(*it_, ros::names::resolve(image_topic_b), 100, image_transport::TransportHints( image_b_transport ));

  info_a_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_b_sub_ = boost::make_shared<message_filters::Subscriber<sensor_msgs::CameraInfo> >();
  info_a_sub_->subscribe(node, ros::names::resolve(info_topic_a), 100);
  info_b_sub_->subscribe(node, ros::names::resolve(info_topic_b), 100);

  sync_ = boost::make_shared<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> >(10);
  sync_->connectInput(*image_a_sub_, *info_a_sub_, *image_b_sub_, *info_b_sub_);
  sync_->registerCallback(boost::bind(&vtkRosDepthImageSubscriber::DepthImageCallback, this, _1, _2, _3, _4));

  //TODO create a single spinner in Director
  if (!spinner_) {
    spinner_ = boost::make_shared<ros::AsyncSpinner>(1);
  }
  spinner_->start();
}

void vtkRosDepthImageSubscriber::Stop()
{
  image_a_sub_->unsubscribe();
  image_b_sub_->unsubscribe();

  info_a_sub_->unsubscribe();
  info_b_sub_->unsubscribe();
  spinner_.reset();
}

void vtkRosDepthImageSubscriber::DepthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                                                    const sensor_msgs::CameraInfoConstPtr& info_a,
                                                    const sensor_msgs::ImageConstPtr& image_b,
                                                    const sensor_msgs::CameraInfoConstPtr& info_b)
{
  double rangeThreshold = 5.0;
  pcl::PointCloud<pcl::PointXYZRGB>::Ptr cloud (new pcl::PointCloud<pcl::PointXYZRGB>);
  utils_.unpackImage(image_a, info_a, image_b, info_b, cloud);

  if (rangeThreshold >= 0) {
    pcl::PassThrough<pcl::PointXYZRGB> pass;
    pass.setInputCloud (cloud);
    pass.setFilterFieldName ("z");
    pass.setFilterLimits (0.0, rangeThreshold);
    pass.filter(*cloud);
  }

  //std::lock_guard<std::mutex> lock(mutex_);
  //dataset_ = transformPolyDataUtils::PolyDataFromPointCloud(cloud);
  //
  std::string frame_id = image_b->header.frame_id;
  vtkSmartPointer<vtkTransform> sensorToLocalTransform = vtkSmartPointer<vtkTransform>::New();
  tf::StampedTransform transform;
  ros::Time time = image_a->header.stamp;
  tf_listener_->waitForTransform("/map", frame_id, time, ros::Duration(2.0));
  try {
    tf_listener_->lookupTransform("/map", frame_id, time, transform);
    sensorToLocalTransform = transformPolyDataUtils::transformFromPose(transform);
  }
  catch (tf::TransformException& ex){
    ROS_ERROR("%s",ex.what());
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  vtkSmartPointer<vtkPolyData> polyData = transformPolyDataUtils::PolyDataFromPointCloud(cloud);
  transformPolyDataUtils::transformPolyData(polyData, dataset_, sensorToLocalTransform);
}

void vtkRosDepthImageSubscriber::GetMesh(vtkPolyData* poly_data)
{
  if (!poly_data || !dataset_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  poly_data->DeepCopy(dataset_);
}

void vtkRosDepthImageSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void vtkRosDepthImageSubscriber::SetDecimate(int decimate)
{
  utils_.SetDecimate(decimate);
}

void vtkRosDepthImageSubscriber::SetRemoveSize(int size_threshold)
{
  utils_.SetRemoveSize(size_threshold);
}
