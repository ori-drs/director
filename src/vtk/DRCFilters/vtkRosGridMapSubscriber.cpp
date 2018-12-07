#include "vtkRosGridMapSubscriber.h"

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
#include <vtkMultisenseUtils.h>
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkRosGridMapSubscriber);

vtkRosGridMapSubscriber::vtkRosGridMapSubscriber()
  :max_number_of_gridmaps_(2)
{
  if (!ros::isInitialized()) {
    int argc = 0;
    char** argv = 0;
    ros::init(argc, argv, "director", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);
  }
}

vtkRosGridMapSubscriber::~vtkRosGridMapSubscriber() {
  ros::shutdown();
}


void vtkRosGridMapSubscriber::Start() {
  ros::NodeHandle n;
  subscriber_ = boost::make_shared<ros::Subscriber>(
        n.subscribe("/elevation_mapping/elevation_map", 1000, &vtkRosGridMapSubscriber::GridMapCallback, this));

  if (!spinner_) {
    spinner_ = boost::make_shared<ros::AsyncSpinner>(1);
  }
  spinner_->start();
}

void vtkRosGridMapSubscriber::Stop() {
  subscriber_->shutdown();
  spinner_.reset();
}

void vtkRosGridMapSubscriber::GridMapCallback(const grid_map_msgs::GridMap& message) {
  // Convert message to map.
  grid_map::GridMap inputMap;
  grid_map::GridMapRosConverter::fromMessage(message, inputMap);
  dataset_.push_back(ConvertMesh(inputMap));
  UpdateDequeSize();
}

vtkSmartPointer<vtkPolyData> vtkRosGridMapSubscriber::ConvertMesh(const grid_map::GridMap& inputMap)
{
  const size_t rows = inputMap.getSize()(0);
  const size_t cols = inputMap.getSize()(1);
  long int count_point = 0;

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();

  const std::vector< std::string > &  layers = inputMap.getLayers();
  std::string layer_name = "elevation";
  // check if the gridmap contains the layer called layer_name
  if ( std::find(layers.begin(), layers.end(), layer_name) == layers.end()) {
    return polyData;
  }
  for (size_t i = 0; i < rows - 1; ++i) {
    for (size_t j = 0; j < cols - 1; ++j) {

      grid_map::Position3 position1, position2, position3;

      if (!inputMap.isValid(grid_map::Index(i, j))) continue;
      if (!inputMap.isValid(grid_map::Index(i, j+1))) continue;
      if (!inputMap.isValid(grid_map::Index(i+1, j))) continue;

      inputMap.getPosition3(layer_name, grid_map::Index(i, j), position1);
      inputMap.getPosition3(layer_name, grid_map::Index(i, j+1), position2);
      inputMap.getPosition3(layer_name, grid_map::Index(i+1, j), position3);

      points->InsertNextPoint(position1(0), position1(1), position1(2));
      points->InsertNextPoint(position2(0), position2(1), position2(2));
      points->InsertNextPoint(position3(0), position3(1), position3(2));

      vtkSmartPointer<vtkTriangle> triangle =
                vtkSmartPointer<vtkTriangle>::New();
      triangle->GetPointIds()->SetId(0, count_point);
      triangle->GetPointIds()->SetId(1, count_point + 1);
      triangle->GetPointIds()->SetId(2, count_point + 2);
      cellArray->InsertNextCell(triangle);

      // triangle 2
      if (!inputMap.isValid(grid_map::Index(i+1, j+1))) continue;

      inputMap.getPosition3(layer_name, grid_map::Index(i, j+1), position1);
      inputMap.getPosition3(layer_name, grid_map::Index(i+1, j+1), position2);
      inputMap.getPosition3(layer_name, grid_map::Index(i+1, j), position3);

      points->InsertNextPoint(position1(0), position1(1), position1(2));
      points->InsertNextPoint(position2(0), position2(1), position2(2));
      points->InsertNextPoint(position3(0), position3(1), position3(2));

      triangle->GetPointIds()->SetId(0, count_point + 3);
      triangle->GetPointIds()->SetId(1, count_point + 4);
      triangle->GetPointIds()->SetId(2, count_point + 5);
      cellArray->InsertNextCell(triangle);

      count_point += 6;
    }
  }

  polyData->SetPoints(points);
  polyData->SetPolys(cellArray);
  return polyData;
}

void vtkRosGridMapSubscriber::UpdateDequeSize()
{
  while (dataset_.size() >= max_number_of_gridmaps_)
  {
    dataset_.pop_front();
  }
}

void vtkRosGridMapSubscriber::GetMeshForMapId(vtkPolyData* polyData)
{
  if (!polyData)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  for (size_t i = 0; i < dataset_.size(); ++i)
  {
    polyData->DeepCopy(dataset_[i]);
  }
}

void vtkRosGridMapSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}



