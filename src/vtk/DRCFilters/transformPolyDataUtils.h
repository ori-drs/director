#ifndef TRANSFORMPOLYDATAUTILS_H_
#define TRANSFORMPOLYDATAUTILS_H_

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkNew.h>

#include <tf/transform_listener.h>
#include <pcl/io/pcd_io.h>
#include <pcl/point_types.h>

class transformPolyDataUtils
{
public:

  static void transformPolyData(vtkPolyData* polyDataSrc, vtkPolyData* polyDataDst,
                                const vtkSmartPointer<vtkTransform>& transform);

  static vtkSmartPointer<vtkTransform> transformFromPose(const tf::StampedTransform& rosTransform);

  static vtkSmartPointer<vtkPolyData> PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud);
};

#endif // TRANSFORMPOLYDATAUTILS_H_
