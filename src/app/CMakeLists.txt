find_package(OpenGL REQUIRED)
if(DD_QT_VERSION EQUAL 4)
  include(${QT_USE_FILE})
else()
  include_directories(${Qt5Core_INCLUDE_DIRS})
endif()

find_package(VTK REQUIRED COMPONENTS
  vtkWrappingPythonCore
  vtkGUISupportQt
  vtkInteractionWidgets
  vtkRenderingAnnotation
  vtkRenderingFreeType
  vtkInteractionStyle
  vtkIOXML
  vtkIOImage
  vtkIOGeometry
  )
include(${VTK_USE_FILE})
check_vtk_qt_version()

include_directories(${CMAKE_CURRENT_SOURCE_DIR})
include_directories(${CMAKE_CURRENT_BINARY_DIR})
include_directories(${CMAKE_DIRECTOR_SOURCE_DIR}/src/common)
include_directories(${PYTHON_INCLUDE_DIRS})

set(decorator_file ${CMAKE_CURRENT_BINARY_DIR}/ddPythonQtDecorators.h)

# Configure whether to use the QVTKWidget or the QVTKOpenGLWidget
set(USE_QVTKWIDGET TRUE)
if((${Qt5Core_VERSION} VERSION_GREATER "5.4.0") AND
   (NOT ${VTK_VERSION} VERSION_LESS "8.0.0") AND
   (${VTK_RENDERING_BACKEND} STREQUAL "OpenGL2"))
  set(USE_QVTKWIDGET FALSE)
endif()
configure_file(${CMAKE_CURRENT_SOURCE_DIR}/ddQVTKOpenGLWidgetConfigure.h.in
               ${CMAKE_CURRENT_BINARY_DIR}/ddQVTKOpenGLWidgetConfigure.h)

if(USE_QVTKWIDGET)
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/wrapped_methods_qvtk.txt
    "QVTKWidget* ddQVTKWidgetView::vtkWidget() const;"
    )
else()
  file(WRITE ${CMAKE_CURRENT_BINARY_DIR}/wrapped_methods_qvtk.txt
    "QVTKOpenGLWidget* ddQVTKWidgetView::vtkWidget() const;"
    )
endif()

set(wrap_files
  wrapped_methods.txt
  ${CMAKE_CURRENT_BINARY_DIR}/wrapped_methods_qvtk.txt
  )

qt_wrap_cpp(moc_srcs
  ${decorator_file}
  ddLumberSelection.h
  ddGLWidgetView.h
  ddGroupBoxHider.h
  ddMacrosManager.h
  ddMainWindow.h
  ddObjectTree.h
  ddPropertiesPanel.h
  ddPythonEventFilter.h
  ddPythonManager.h
  ddPythonUndoCommand.h
  ddQVTKWidgetView.h
  ddSignalMap.h
  ddSpreadsheetView.h
  ddTaskSelection.h
  ddViewBase.h
  ddViewManager.h
  ddViewMenu.h
  ddVTKObjectMap.h
  )

qt_wrap_ui(ui_srcs
  ddMainWindow.ui
  ddLumberSelection.ui
  ddTaskSelection.ui
  )

qt_add_resources(resource_srcs
  uiresources.qrc
  images/resources.qrc
  )

set(srcs
  ${moc_srcs}
  ${ui_srcs}
  ${resource_srcs}

  QVTKOpenGLInit.cpp
  ddGLWidgetView.cpp
  ddLumberSelection.cpp
  ddMacrosManager.cpp
  ddMainWindow.cpp
  ddObjectTree.cpp
  ddPropertiesPanel.cpp
  ddPythonApp.cpp
  ddPythonManager.cpp
  ddPythonQtWrapperFactory.cpp
  ddPythonUndoCommand.cpp
  ddQVTKWidgetView.cpp
  ddSignalMap.cpp
  ddSpreadsheetView.cpp
  ddTaskSelection.cpp
  ddViewBase.cpp
  ddViewManager.cpp
  ddViewMenu.cpp
  ddVTKObjectMap.cpp

  vtkSimpleActorInteractor.cxx
  vtkTDxInteractorStyleCallback.cxx
  )

set(deps
  ${QT_LIBRARIES}
  ${OPENGL_LIBRARIES}
  ${VTK_LIBRARIES}
  )

set(pkg_deps)




#####
project(ddApp)
find_package(catkin  REQUIRED COMPONENTS 
    kdl_parser forward_kinematics ctk_python_console qt_property_browser python_qt topic_tools tf)


##################################
#Compiles the RigidBodyTree model of the robot
find_package(Eigen REQUIRED)

include_directories(
  ${EIGEN_INCLUDE_DIRS}
  ${catkin_INCLUDE_DIRS}
)

set(moc_srcs)
qt_wrap_cpp(moc_srcs
  ddDrakeModel.h
)

list(APPEND srcs
  DirectorRigidBodyTree.cpp
)

list(APPEND srcs
  ${moc_srcs}
)

list(APPEND srcs ddDrakeModel.cpp)

list(APPEND wrap_files
  wrapped_methods_drake.txt
)

list(APPEND deps
  ${catkin_LIBRARIES}
)

if (USE_ROS)

  set(moc_srcs)
  qt_wrap_cpp(moc_srcs
    ddROSSubscriber.h
  )

  list(APPEND srcs
    ${moc_srcs}
  )

  list(APPEND wrap_files
    wrapped_methods_ros.txt
  )

endif()

add_library(ddApp SHARED ${srcs})
target_link_libraries(ddApp ${deps} )

if(pkg_deps)
  use_pkg(ddApp ${pkg_deps})
endif()

set(_install_targets ddApp directorPython drake-visualizer)

add_executable(drake-visualizer drakeVisualizerApp.cpp)
target_link_libraries(drake-visualizer ddApp)

# Build main executable
add_executable(director main.cpp)
target_link_libraries(director ddApp)
list(APPEND _install_targets director)


add_executable(directorPython consoleApp.cpp)
target_link_libraries(directorPython ddApp ${PYTHON_LIBRARIES})

install(TARGETS ${_install_targets}
    EXPORT ${DD_TARGETS_NAME}
    RUNTIME DESTINATION ${DD_INSTALL_BIN_DIR}
    LIBRARY DESTINATION ${DD_INSTALL_LIB_DIR}
    ARCHIVE DESTINATION ${DD_INSTALL_LIB_DIR})

if(APPLE)
  install(DIRECTORY drake-visualizer.app
      DESTINATION ${DD_INSTALL_BIN_DIR}
      USE_SOURCE_PERMISSIONS)
endif()

add_custom_command(OUTPUT ${decorator_file}
  COMMAND ${PYTHON_EXECUTABLE} WrapPythonQt.py
  -i ${wrap_files} -o "${decorator_file}"
  --class-prefixes vtk dd QVTK --qt-class-prefixes dd QVTK
  --export-header ddAppConfigure.h --export-symbol DD_APP_EXPORT
  --auto-class-includes --module-name dd
  DEPENDS ${wrap_files} WrapPythonQt.py
  WORKING_DIRECTORY ${CMAKE_CURRENT_SOURCE_DIR})
