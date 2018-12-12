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

  void GridMapCallback(const grid_map_msgs::GridMap& message);

  vtkSmartPointer<vtkPolyData> ConvertMesh(grid_map::GridMap &inputMap);

  /**
  * @brief normalizeIntensity computes color value in the interval [0,1].
  */
  void normalizeIntensity(float& intensity, float minIntensity, float maxIntensity) const;

  /**
   * @brief getInterpolatedColor computes the color contained in the colorLayer
   */
  void getInterpolatedColor(grid_map::GridMap& inputMap, const std::string& colorLayer,
                            const grid_map::Index& index, float minIntensity, float maxIntensity,
                            unsigned char (&color)[3]) const;

  static float clamp(float x, float lower, float upper);

  vtkSmartPointer<vtkPolyData> dataset_;
  boost::shared_ptr<ros::Subscriber> subscriber_;
  boost::shared_ptr<ros::AsyncSpinner> spinner_;
  std::mutex mutex_;
};

#endif // VTKROSGRIDMAPSUBSCRIBER_H_
