#include "transformPolyDataUtils.h"
#include "vtkRosPointCloudConversions.h"

#include <vtkPointData.h>
#include <vtkPoints.h>


void transformPolyDataUtils::transformPolyData(vtkPolyData* polyDataSrc, vtkPolyData* polyDataDst,
                                               const vtkSmartPointer<vtkTransform>& transform)
{
  vtkSmartPointer<vtkTransformPolyDataFilter> transformFilter = vtkSmartPointer<vtkTransformPolyDataFilter>::New();
  transformFilter->SetTransform(transform);
  transformFilter->SetInputData(polyDataSrc);
  transformFilter->Update();
  polyDataDst->DeepCopy(transformFilter->GetOutput());

  //add z array
  vtkPoints* points = polyDataDst->GetPoints();
  vtkIdType num_points = points->GetNumberOfPoints();
  vtkSmartPointer<vtkFloatArray> z = vtkSmartPointer<vtkFloatArray>::New();
  z->SetName("z");
  z->SetNumberOfValues(num_points);
  polyDataDst->GetPointData()->AddArray(z);

  for(vtkIdType i = 0; i < num_points; ++i)
  {
    double pt[3];
    points->GetPoint(i, pt);
    z->SetValue(i, pt[2]);
  }
}

vtkSmartPointer<vtkTransform> transformPolyDataUtils::transformFromPose(const tf::StampedTransform& rosTransform)
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

vtkSmartPointer<vtkPolyData> transformPolyDataUtils::PolyDataFromPointCloud(pcl::PointCloud<pcl::PointXYZRGB>::ConstPtr cloud)
{
  vtkIdType nr_points = cloud->points.size();

  vtkNew<vtkPoints> points;
  points->SetDataTypeToFloat();
  points->SetNumberOfPoints(nr_points);

  vtkNew<vtkUnsignedCharArray> rgbArray;
  rgbArray->SetName("rgb_colors");
  rgbArray->SetNumberOfComponents(3);
  rgbArray->SetNumberOfTuples(nr_points);

  if (cloud->is_dense)
  {
    for (vtkIdType i = 0; i < nr_points; ++i) {
      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(i, point);
      rgbArray->SetTupleValue(i, color);
    }
  }
  else
  {
    vtkIdType j = 0;    // true point index
    for (vtkIdType i = 0; i < nr_points; ++i)
    {
      // Check if the point is invalid
      if (!pcl_isfinite (cloud->points[i].x) ||
          !pcl_isfinite (cloud->points[i].y) ||
          !pcl_isfinite (cloud->points[i].z))
        continue;

      float point[3] = {cloud->points[i].x, cloud->points[i].y, cloud->points[i].z};
      unsigned char color[3] = {cloud->points[i].r, cloud->points[i].g, cloud->points[i].b};
      points->SetPoint(j, point);
      rgbArray->SetTupleValue(j, color);
      j++;
    }
    nr_points = j;
    points->SetNumberOfPoints(nr_points);
    rgbArray->SetNumberOfTuples(nr_points);
  }

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  polyData->SetPoints(points.GetPointer());
  polyData->GetPointData()->AddArray(rgbArray.GetPointer());
  polyData->SetVerts(NewVertexCells(nr_points));
  return polyData;
}
