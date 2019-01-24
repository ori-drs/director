#ifndef VTKROSINIT_H_
#define VTKROSINIT_H_

#include <mutex>
#include <deque>
#include <vector>

#include <boost/shared_ptr.hpp>

#include <ros/ros.h>

#include <vtkDRCFiltersModule.h>
#include <vtkPolyDataAlgorithm.h>
#include <vtkPolyData.h>
#include "vtkInformation.h"
#include "vtkInformationVector.h"
#include <vtkSmartPointer.h>

class vtkImageData;
class vtkTransform;

class VTKDRCFILTERS_EXPORT vtkRosInit : public vtkPolyDataAlgorithm
{
public:
  vtkTypeMacro(vtkRosInit, vtkPolyDataAlgorithm);
  void PrintSelf(ostream& os, vtkIndent indent);

  static vtkRosInit *New();

  void AddArg(std::string arg){
    // I'd like to be able to pass char** argc to the constructor
    // This is a work around - pass them individually

    std::size_t found = arg.find("__name");
    if (found!=std::string::npos){
      //std::cout << "not adding: " << arg << "\n";
      return;
    }
    //std::cout << "adding: " << arg << "\n";
    args.push_back(arg);
  }

  void Start();

  void Stop();

  void IsInitialized();

protected:

  vtkRosInit();
  virtual ~vtkRosInit();

private:
  vtkRosInit(const vtkRosInit&);  // Not implemented.
  void operator=(const vtkRosInit&);  // Not implemented.

  std::vector< std::string > args;

};

#endif // VTKROSINIT_H_
