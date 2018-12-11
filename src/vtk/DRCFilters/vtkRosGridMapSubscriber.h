#ifndef VTKROSGRIDMAPSUBSCRIBER_H_
#define VTKROSGRIDMAPSUBSCRIBER_H_

#include <mutex>
#include <deque>

#include <boost/shared_ptr.hpp>

#include <grid_map_ros/grid_map_ros.hpp>
#include <ros/ros.h>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include <vtkSmartPointer.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosGridMapSubscriber : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkRosGridMapSubscriber, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosGridMapSubscriber *New();

  void Start();

  void Stop();

  void GetMeshForMapId(vtkPolyData* polyData);


protected:

  vtkRosGridMapSubscriber();
  virtual ~vtkRosGridMapSubscriber();

private:
  vtkRosGridMapSubscriber(const vtkRosGridMapSubscriber&);  // Not implemented.
  void operator=(const vtkRosGridMapSubscriber&);  // Not implemented.

  void UpdateDequeSize();

  void GridMapCallback(const grid_map_msgs::GridMap& message);

  vtkSmartPointer<vtkPolyData> ConvertMesh(grid_map::GridMap &inputMap);

  /**
  Compute color value in the interval [0,255].
  **/
  void normalizeColor(float& intensity, float min_intensity, float max_intensity) const;

  unsigned char computeColor(grid_map::GridMap& inputMap, const std::string& color_layer,
                     const grid_map::Index& index, float minIntensity, float maxIntensity) const;

  vtkSmartPointer<vtkPolyData> dataset_;
  boost::shared_ptr<ros::Subscriber> subscriber_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
};

#endif // VTKROSGRIDMAPSUBSCRIBER_H_
