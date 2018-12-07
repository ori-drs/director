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

  void GetMeshForMapId(int viewId, vtkIdType mapId, vtkPolyData* polyData);


protected:

  vtkRosGridMapSubscriber();
  virtual ~vtkRosGridMapSubscriber();

private:
  vtkRosGridMapSubscriber(const vtkRosGridMapSubscriber&);  // Not implemented.
  void operator=(const vtkRosGridMapSubscriber&);  // Not implemented.

  void UpdateDequeSize();

  void GridMapCallback(const grid_map_msgs::GridMap& message);

  vtkSmartPointer<vtkPolyData> ConvertMesh(const grid_map::GridMap& inputMap);

  std::deque<vtkSmartPointer<vtkPolyData> > dataset_;
  const int max_number_of_gridmaps_;
  boost::shared_ptr<ros::Subscriber> subscriber_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
};

#endif // VTKROSGRIDMAPSUBSCRIBER_H_
