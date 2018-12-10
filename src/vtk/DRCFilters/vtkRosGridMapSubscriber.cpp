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
  dataset_ = ConvertMesh(inputMap);
}

vtkSmartPointer<vtkPolyData> vtkRosGridMapSubscriber::ConvertMesh(grid_map::GridMap& inputMap)
{
  inputMap.convertToDefaultStartIndex();
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

      std::vector<grid_map::Position3> vertices;
      for (size_t k = 0; k < 2; k++) {
        for (size_t l = 0; l < 2; l++) {
          grid_map::Position3 position;
          grid_map::Index index(i + k, j + l);
          if (!inputMap.isValid(index)) {
            continue;
          }

          inputMap.getPosition3(layer_name, index, position);
          vertices.push_back(position);
        }
      }
      if (vertices.size() > 2) {
        for (size_t m = 1; m < vertices.size() - 1; m++) {
          points->InsertNextPoint(vertices[m-1](0), vertices[m-1](1), vertices[m-1](2));
          points->InsertNextPoint(vertices[m](0), vertices[m](1), vertices[m](2));
          points->InsertNextPoint(vertices[m+1](0), vertices[m+1](1), vertices[m+1](2));

          vtkSmartPointer<vtkTriangle> triangle =
              vtkSmartPointer<vtkTriangle>::New();
          triangle->GetPointIds()->SetId(0, count_point);
          triangle->GetPointIds()->SetId(1, count_point + 1);
          triangle->GetPointIds()->SetId(2, count_point + 2);
          cellArray->InsertNextCell(triangle);
          count_point += 3;
        }
      }

    }
  }
  std::cout << "count " << count_point << std::endl;
  polyData->SetPoints(points);
  polyData->SetPolys(cellArray);
  return polyData;
}

void vtkRosGridMapSubscriber::GetMeshForMapId(vtkPolyData* polyData)
{
  if (!polyData || !dataset_)
  {
    return;
  }

  std::lock_guard<std::mutex> lock(mutex_);
  polyData->DeepCopy(dataset_);
}

void vtkRosGridMapSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}



