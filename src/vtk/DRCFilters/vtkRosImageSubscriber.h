#ifndef VTKROSIMAGESUBSCRIBER_H_
#define VTKROSIMAGESUBSCRIBER_H_

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
#include <vtkImageData.h>
#include <vtkInformation.h>
#include <vtkInformationVector.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>

class VTKDRCFILTERS_EXPORT vtkRosImageSubscriber : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkRosImageSubscriber, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosImageSubscriber *New();

  void Start(const std::string& image_topic, const std::string& image_transport,
             const std::string& info_topic);

  void Stop();

  unsigned long long GetCurrentImageTime() const;

  void GetImage(vtkImageData* image);

  void ComputeTextureCoords(const std::string& camera_name, vtkPolyData* poly_data) const;

  void GetBodyToCameraTransform(vtkTransform* transform) const;


protected:

  vtkRosImageSubscriber();
  virtual ~vtkRosImageSubscriber();

private:
  vtkRosImageSubscriber(const vtkRosImageSubscriber&);  // Not implemented.
  void operator=(const vtkRosImageSubscriber&);  // Not implemented.


  void ImageCallback(const sensor_msgs::ImageConstPtr& image,
                     const sensor_msgs::CameraInfoConstPtr& info);

  bool first_data_received_;
  vtkSmartPointer<vtkTransform> body_to_camera_transform_;
  vtkSmartPointer<vtkImageData> dataset_;
  sensor_msgs::CameraInfo camera_info_;
  boost::shared_ptr<tf::TransformListener> tf_listener_;
  ros::Time time_;

  boost::shared_ptr<image_transport::SubscriberFilter> image_sub_;
  boost::shared_ptr<message_filters::Subscriber<sensor_msgs::CameraInfo> > info_sub_;
  boost::shared_ptr<message_filters::TimeSynchronizer<sensor_msgs::Image, sensor_msgs::CameraInfo> > sync_;

  boost::shared_ptr<image_transport::ImageTransport> it_;
  mutable std::mutex mutex_;
};

#endif // VTKROSIMAGESUBSCRIBER_H_
