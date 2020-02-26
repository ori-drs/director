
#include <assimp/Importer.hpp>
#include <assimp/scene.h>
#include <assimp/postprocess.h>
#include <assimp/IOStream.hpp>
#include <assimp/IOSystem.hpp>


#include <vtkSmartPointer.h>
#include <vtkTransform.h>
#include <vtkAppendPolyData.h>
#include <vtkMultiBlockDataSet.h>
#include <vtkXMLMultiBlockDataWriter.h>
#include <vtkTransformPolyDataFilter.h>
#include <vtkIdTypeArray.h>
#include <vtkCellArray.h>
#include <vtkStringArray.h>
#include <vtkFloatArray.h>
#include <vtkDoubleArray.h>
#include <vtkPointData.h>

#include "convertCollada.h"

std::vector<vtkSmartPointer<vtkPolyData> > buildMesh( const aiScene* scene, const aiNode* node )
{
  if (!node)
  {
    return {};
  }

  std::vector<vtkSmartPointer<vtkPolyData> > polyDatas;
  aiMatrix4x4 transform = node->mTransformation;
  aiNode *pnode = node->mParent;
  while (pnode)
  {
    // Don't convert to y-up orientation, which is what the root node in
    // Assimp does
    if (pnode->mParent != NULL)
      transform = pnode->mTransformation * transform;
    pnode = pnode->mParent;
  }

  aiMatrix3x3 rotation(transform);
  aiMatrix3x3 inverse_transpose_rotation(rotation);
  inverse_transpose_rotation.Inverse();
  inverse_transpose_rotation.Transpose();
  for (uint32_t i = 0; i < node->mNumMeshes; i++)
  {
    aiMesh* input_mesh = scene->mMeshes[node->mMeshes[i]];

    if(3*input_mesh->mNumFaces != input_mesh->mNumVertices)
    {
      continue; //TODO might need to handle this, at the moment only triangular faces are handles
    }

    vtkSmartPointer<vtkPolyData> polyData = vtkSmartPointer<vtkPolyData>::New();
    vtkSmartPointer<vtkPoints> points = vtkSmartPointer<vtkPoints>::New();
    vtkSmartPointer<vtkCellArray> cellArray = vtkSmartPointer<vtkCellArray>::New();
    vtkSmartPointer<vtkDoubleArray> tcoords = vtkSmartPointer<vtkDoubleArray>::New();

    points->SetDataTypeToDouble();
    points->SetNumberOfPoints(input_mesh->mNumVertices);
    int countPoint = 0;

    int numCells = input_mesh->mNumFaces;
    vtkSmartPointer<vtkIdTypeArray> cellsPtr = vtkSmartPointer<vtkIdTypeArray>::New();
    vtkIdType* vtkCells = (vtkIdType*)malloc(4*sizeof(vtkIdType)*numCells);
    vtkIdType* ptr = vtkCells;
    for(int i = 0; i < numCells; ++i) {
      *ptr = 3; // number of points followed by points ids
      *(ptr + 1) = i*3;
      *(ptr + 2) = i*3 + 1;
      *(ptr + 3) = i*3 + 2;
      ptr += 4;
    }
    cellsPtr->SetArray(vtkCells, 4*numCells, 0);
    cellArray->SetCells(numCells, cellsPtr);

    // texture coordinates
    if (input_mesh->HasTextureCoords(0))
    {
      tcoords->SetName("tcoords");
      tcoords->SetNumberOfComponents(2);
      tcoords->SetNumberOfTuples(input_mesh->mNumVertices);

      tcoords->FillComponent(0, -1);
      tcoords->FillComponent(1, -1);

    }

    // Add the vertices
    for (uint32_t j = 0; j < input_mesh->mNumFaces; j++)
    {
      aiFace face = input_mesh->mFaces[j];
      if (face.mNumIndices != 3) //non triangular face are not handled
        continue;

      for(int index = 0; index < 3; ++index)
      {
        aiVector3D p = input_mesh->mVertices[face.mIndices[index]];
        p *= transform;
        if(countPoint >= input_mesh->mNumVertices)
        {
          std::cout << "An error occured while parsing the collada file" << std::endl;
          break;
        }
        points->SetPoint(countPoint, p.x, p.y, p.z);

        //texture
        if (input_mesh->HasTextureCoords(0))
        {
          double u = input_mesh->mTextureCoords[0][countPoint].x;
          double v = input_mesh->mTextureCoords[0][countPoint].y;
          tcoords->SetComponent(countPoint, 0, u);
          tcoords->SetComponent(countPoint, 1, v);
        }
        countPoint++;
      }
    }

    // Store material name and color
    aiMaterial *amat = scene->mMaterials[input_mesh->mMaterialIndex];

    for (uint32_t j=0; j < amat->mNumProperties; j++)
    {
      aiMaterialProperty *prop = amat->mProperties[j];
      std::string propKey = prop->mKey.data;

      if (propKey == "$tex.file")
      {
        aiString texName;
        aiTextureMapping mapping;
        uint32_t uvIndex;
        amat->GetTexture(aiTextureType_DIFFUSE,0, &texName, &mapping, &uvIndex);

        // Assume textures are in paths relative to the mesh
        std::string texturePath = texName.data;
        vtkSmartPointer<vtkStringArray> s = vtkSmartPointer<vtkStringArray>::New();
        s->InsertNextValue(texturePath);
        s->SetName("texture_filename");
        polyData->GetFieldData()->AddArray(s);
        break;
      }
      else if (propKey == "$clr.diffuse")
      {
        aiColor3D clr;
        amat->Get(AI_MATKEY_COLOR_DIFFUSE, clr);

        vtkSmartPointer<vtkDoubleArray> c = vtkSmartPointer<vtkDoubleArray>::New();
        c->SetNumberOfComponents(3);
        c->SetName("color_material");
        c->InsertNextTuple3(clr.r, clr.g, clr.b);
        polyData->GetFieldData()->AddArray(c);
      }
    }

    polyData->SetPoints(points);
    polyData->SetPolys(cellArray);
    if (input_mesh->HasTextureCoords(0))
    {
      polyData->GetPointData()->SetTCoords(tcoords);
    }
    polyDatas.push_back(polyData);

  }


  for (uint32_t i=0; i < node->mNumChildren; ++i)
  {
    std::vector<vtkSmartPointer<vtkPolyData> > v = buildMesh(scene, node->mChildren[i]);
    if(v.size() > 0)
      polyDatas.insert(polyDatas.end(), v.begin(), v.end());
  }
  return polyDatas;
}

std::vector<vtkSmartPointer<vtkPolyData> > ConvertCollada::readCollada(const std::string& fileName)
{
  Assimp::Importer importer;
  const aiScene* scene = importer.ReadFile(fileName, aiProcess_SortByPType|aiProcess_GenNormals|aiProcess_Triangulate|aiProcess_GenUVCoords);
  if (!scene)
  {
    std::cout << "Could not load collada file " << fileName << ", " << importer.GetErrorString() << std::endl;
  }
  std::vector<vtkSmartPointer<vtkPolyData> > data = buildMesh(scene, scene->mRootNode);
  return data;
}

