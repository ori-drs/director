#ifndef CONVERTCOLLADA_H
#define CONVERTCOLLADA_H

#include <vtkPolyData.h>
#include <vtkSmartPointer.h>

/**
 * @brief The ConvertCollada class provides a method to convert collada ( dae ) files into vtk format
 */

class ConvertCollada
{
public:
  ConvertCollada() {}

  /**
   * @brief Reads a collada file and returns the converted meshes
   * @param fileName
   * @return
   */
  static std::vector<vtkSmartPointer<vtkPolyData> > readCollada(const std::string& fileName);
};

#endif
