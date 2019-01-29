#ifndef VTKROSDEPTHIMAGESUBSCRIBER_H_
#define VTKROSDEPTHIMAGESUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <sensor_msgs/Image.h>
#include <sensor_msgs/CompressedImage.h>
#include <sensor_msgs/CameraInfo.h>
#include <ros/ros.h>
#include <tf/transform_listener.h>
#include <image_transport/image_transport.h>
#include <image_transport/subscriber_filter.h>
#include <message_filters/subscriber.h>
#include <message_filters/time_synchronizer.h>
#include <message_filters/synchronizer.h>
#include <message_filters/sync_policies/approximate_time.h>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>

#include <depthImageUtils.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosDepthImageSubscriber : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkRosDepthImageSubscriber, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosDepthImageSubscriber *New();

  void Start();

  void Stop();

  void GetMesh(vtkPolyData* poly_data);

protected:

  vtkRosDepthImageSubscriber();
  virtual ~vtkRosDepthImageSubscriber();

private:
  vtkRosDepthImageSubscriber(const vtkRosDepthImageSubscriber&);  // Not implemented.
  void operator=(const vtkRosDepthImageSubscriber&);  // Not implemented.


  void DepthImageCallback(const sensor_msgs::ImageConstPtr& image_a,
                          const sensor_msgs::CameraInfoConstPtr& info_a,
                          const sensor_msgs::ImageConstPtr& image_b,
                          const sensor_msgs::CameraInfoConstPtr& info_b);



  vtkSmartPointer<vtkPolyData> dataset_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  DepthImageUtils utils_;

  boost::shared_ptr<image_transport::SubscriberFilter> image_a_sub_, image_b_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > info_a_sub_, info_b_sub_;
  boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo, sensor_msgs::Image,
      sensor_msgs::CameraInfo> > sync_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
};

#endif // VTKROSDEPTHIMAGESUBSCRIBER_H_
