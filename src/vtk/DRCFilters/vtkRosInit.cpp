#include "vtkRosInit.h"

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
#include "vtkRosPointCloudConversions.h"
#include "vtkObjectFactory.h"

vtkStandardNewMacro(vtkRosInit);

vtkRosInit::vtkRosInit()
{
}

vtkRosInit::~vtkRosInit() {
  ros::shutdown();
}



void vtkRosInit::Start() {
  if (!ros::isInitialized()) {
    std::cout << "vtkRosInit: roscpp not initialized. Running init.\n";

    //std::cout << "Using these args:\n";
    //for(size_t i = 0; i < args.size(); i++){
    //   std::cout << args[i] << "\n";
    //}

    // guarantee contiguous, null terminated strings
    std::vector<std::vector<char>> vstrings;

    // pointers to rhose strings
    std::vector<char*> cstrings;

    vstrings.reserve(args.size());
    cstrings.reserve(args.size());

    for(size_t i = 0; i < args.size(); ++i)
    {
        vstrings.emplace_back(args[i].begin(), args[i].end());
        vstrings.back().push_back('\0');
        cstrings.push_back(vstrings.back().data());
    }

    int argc = cstrings.size();

    ros::init(argc, cstrings.data(), "director", ros::init_options::NoSigintHandler |
              ros::init_options::AnonymousName);
  }else{
    std::cout << "vtkRosInit: ROS is Initialized. Not running init. This should not happen\n";
  }
}


void vtkRosInit::Stop() {
}

void vtkRosInit::IsInitialized() {
  if (!ros::isInitialized()) {
    std::cout << "vtkRosInit: ROS not Initialized.\n";
  }else{
    std::cout << "vtkRosInit: ROS is Initialized.\n";   
  }
}

void vtkRosInit::PrintSelf(ostream& os, vtkIndent indent)
{
  vtkPolyDataAlgorithm::PrintSelf(os, indent);
}
