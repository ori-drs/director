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

  //we can't modify dataset_ if it's being copied in GetMesh
  std::lock_guard<std::mutex> lock(mutex_);
  dataset_ = ConvertMesh(inputMap);
}

#include "boost/date_time/posix_time/posix_time.hpp" //include all types plus i/o

vtkSmartPointer<vtkPolyData> vtkRosGridMapSubscriber::ConvertMesh(grid_map::GridMap& inputMap)
{
  boost::posix_time::ptime mst1 = boost::posix_time::microsec_clock::local_time();

  inputMap.convertToDefaultStartIndex();
  const size_t rows = inputMap.getSize()(0);
  const size_t cols = inputMap.getSize()(1);

  vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
  vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
  vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
  points->SetDataTypeToDouble();

  const std::vector< std::string > &  layers = inputMap.getLayers();
  // initialize colors, one for each layers
  std::vector<vtkSmartPointer<vtkUnsignedCharArray> > colors(layers.size());
  std::vector<float> minIntensity(layers.size());
  std::vector<float> maxIntensity(layers.size());
  for(int i = 0; i < layers.size(); ++i) {
    colors[i] = vtkSmartPointer<vtkUnsignedCharArray>::New();
    colors[i]->SetNumberOfComponents(3);
    colors[i]->SetName(layers[i].c_str());
    minIntensity[i] = inputMap[layers[i]].minCoeffOfFinites();
    maxIntensity[i] = inputMap[layers[i]].maxCoeffOfFinites();
  }

  std::string layer_name = "elevation";
  // check if the gridmap contains the layer called layer_name
  if ( std::find(layers.begin(), layers.end(), layer_name) == layers.end()) {
    std::cout << "No layer called elevation" << std::endl;
    return polyData;
  }

  std::vector<std::vector<grid_map::Position3> > vertices((rows-1)*(cols-1));
  std::vector<std::vector<grid_map::Index> > indexes((rows-1)*(cols-1));
  int numPoints = 0;
  int numCells = 0;
  // first loop to compute the number of vertices and the number of cells
  for (size_t i = 0; i < rows - 1; ++i) {
    for (size_t j = 0; j < cols - 1; ++j) {

      for (size_t k = 0; k < 2; k++) {
        for (size_t l = 0; l < 2; l++) {
          grid_map::Position3 position;
          grid_map::Index index(i + k, j + l);
          if (!inputMap.isValid(index)) {
            continue;
          }

          inputMap.getPosition3(layer_name, index, position);
          vertices[i*(cols-1) + j].push_back(position);
          indexes[i*(cols-1) + j].push_back(index);
        }
      }
      if (vertices[i*(cols-1) + j].size() == 3) {
        numPoints += 3;
        numCells += 1;
      } else if (vertices[i*(cols-1) + j].size() == 4) {
        numPoints += 6;
        numCells += 2;
      }

    }
  }
  points->SetNumberOfPoints(numPoints);

  // populating the cells, the following code is equivalent to
  /*vtkSmartPointer<vtkTriangle> triangle = vtkSmartPointer<vtkTriangle>::New();
  triangle->GetPointIds()->SetId(0, count_point);
  triangle->GetPointIds()->SetId(1, count_point + 1);
  triangle->GetPointIds()->SetId(2, count_point + 2);
  cellArray->InsertNextCell(triangle);*/
  vtkSmartPointer<vtkIdTypeArray> cellsPtr = vtkIdTypeArray::New();
  vtkIdType* vtkCells = (vtkIdType*)malloc(4*sizeof(vtkIdType)*numCells);
  vtkIdType* ptr = vtkCells;
  for(int i = 0; i < numCells; ++i) {
    *ptr = 3;
    *(ptr + 1) = i*3;
    *(ptr + 2) = i*3 + 1;
    *(ptr + 3) = i*3 + 2;
    ptr += 4;
  }
  cellsPtr->SetArray(vtkCells, 4*numCells, 0);
  cellArray->SetCells(numCells, cellsPtr);

  int count_point = 0;
  for (size_t j = 0; j < vertices.size(); ++j) {
    if (vertices[j].size() > 2) {
      for (size_t m = 1; m < vertices[j].size() - 1; m++) {
        points->SetPoint(count_point, vertices[j][m-1](0), vertices[j][m-1](1), vertices[j][m-1](2));
        points->SetPoint(count_point+1, vertices[j][m](0), vertices[j][m](1), vertices[j][m](2));
        points->SetPoint(count_point+2, vertices[j][m+1](0), vertices[j][m+1](1), vertices[j][m+1](2));

        count_point += 3;
        //colors
        // about 50msec for other layers
        for(int i = 0; i< layers.size(); ++i) {
          unsigned char color[3];

          getInterpolatedColor(inputMap, layers[i], indexes[j][m-1],
              minIntensity[i], maxIntensity[i], color);
          colors[i]->InsertNextTupleValue(color);
          getInterpolatedColor(inputMap, layers[i], indexes[j][m],
                               minIntensity[i], maxIntensity[i], color);
          colors[i]->InsertNextTupleValue(color);
          getInterpolatedColor(inputMap, layers[i], indexes[j][m+1],
              minIntensity[i], maxIntensity[i], color);
          colors[i]->InsertNextTupleValue(color);
        }

      }
    }
  }

  polyData->SetPoints(points);
  for(int i = 0; i< layers.size(); ++i) {
    polyData->GetPointData()->AddArray(colors[i]);
  }
  polyData->SetPolys(cellArray);

  boost::posix_time::time_duration msdiff;
  msdiff = boost::posix_time::microsec_clock::local_time() - mst1;
  std::cout << msdiff.total_milliseconds() << " elapsed (elevation map)" << std::endl;
  std::cout << std::endl;

  return polyData;
}

void vtkRosGridMapSubscriber::GetMesh(vtkPolyData* polyData)
{
  if (!polyData || !dataset_)
  {
    return;
  }

  //we can't copy dataset_ if it's being modified in GridMapCallback
  std::lock_guard<std::mutex> lock(mutex_);
  polyData->DeepCopy(dataset_);
}

void vtkRosGridMapSubscriber::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}

void vtkRosGridMapSubscriber::normalizeIntensity(float& intensity, float minIntensity, float maxIntensity) const
{
  if (std::abs(maxIntensity - minIntensity) > 0.01) {
    intensity = std::min(intensity, maxIntensity);
    intensity = std::max(intensity, minIntensity);
    intensity = (intensity - minIntensity) / (maxIntensity - minIntensity);
  } else {
    intensity = 1.f;
  }
}

void vtkRosGridMapSubscriber::getInterpolatedColor(grid_map::GridMap& inputMap, const std::string& colorLayer,
                                                   const grid_map::Index& index, float minIntensity, float maxIntensity,
                                                   unsigned char (&color)[3]) const {
  float intensity = inputMap[colorLayer](index(0), index(1));
  if (colorLayer != "color") {
    // transform the intensity to get a value between [0-1]
    normalizeIntensity(intensity, minIntensity, maxIntensity);
    // transform the intensity into jet color
    color[0] = 255 * clamp(std::min(4*intensity - 1.5, -4*intensity + 4.5) , 0.0, 1.0);
    color[1] = 255 * clamp(std::min(4*intensity - 0.5, -4*intensity + 3.5) , 0.0, 1.0);
    color[2] = 255 * clamp(std::min(4*intensity + 0.5, -4*intensity + 2.5) , 0.0, 1.0);
  } else {
    // extracting the color from the layer
    Eigen::Vector3f colorVectorRGB;
    grid_map::colorValueToVector(intensity, colorVectorRGB);
    color[0] = 255 * colorVectorRGB(0);
    color[1] = 255 * colorVectorRGB(1);
    color[2] = 255 * colorVectorRGB(2);
  }
}

float vtkRosGridMapSubscriber::clamp(float x, float lower, float upper)
{
  return std::min(upper, std::max(x, lower));
}



