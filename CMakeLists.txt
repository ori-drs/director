cmake_minimum_required(VERSION 2.8.7)


project(director)

find_package(catkin REQUIRED)

catkin_package(
  CATKIN_DEPENDS  
    pcl_ros pcl_plugin
    ctk_python_console
    qt_property_browser python_qt
    anymal_msgs vtk_ros)

# set a default build type if none was specified
if(NOT CMAKE_BUILD_TYPE AND NOT CMAKE_CONFIGURATION_TYPES)
  message(STATUS "Setting build type to 'Release' as none was specified.")
  set(CMAKE_BUILD_TYPE Release CACHE STRING "Choose the type of build." FORCE)
  # set the possible values of build type for cmake-gui
  set_property(CACHE CMAKE_BUILD_TYPE PROPERTY STRINGS
    "Debug" "Release" "MinSizeRel" "RelWithDebInfo")
endif()

option(USE_EXTERNAL_INSTALL "If enabled, install dependencies to CMAKE_INSTALL_PREFIX." OFF)
mark_as_advanced(USE_EXTERNAL_INSTALL)

include(cmake/dd-macros.cmake)
include(cmake/externals.cmake)
include(ExternalProject)

set_property(DIRECTORY PROPERTY EP_PREFIX ${director_BINARY_DIR})

set(source_prefix ${director_BINARY_DIR}/src)
set(install_prefix ${director_BINARY_DIR}/install)
if (USE_EXTERNAL_INSTALL)
  set(install_prefix ${CMAKE_INSTALL_PREFIX})
else()
  mark_as_advanced(CMAKE_INSTALL_PREFIX)
endif()

set(CMAKE_DIRECTOR_SOURCE_DIR ${CMAKE_CURRENT_SOURCE_DIR})

list(APPEND CMAKE_MODULE_PATH "${CMAKE_DIRECTOR_SOURCE_DIR}/cmake/modules")

# set project variables
set(DD_INSTALL_BIN_DIR bin)
set(DD_INSTALL_DOC_DIR share/doc/director)
set(DD_INSTALL_LIB_DIR lib)
set(DD_INSTALL_INCLUDE_DIR include/director)
set(DD_TARGETS_NAME director-targets)

set(EXECUTABLE_OUTPUT_PATH "${CMAKE_BINARY_DIR}/${DD_INSTALL_BIN_DIR}")
set(LIBRARY_OUTPUT_PATH "${CMAKE_BINARY_DIR}/${DD_INSTALL_LIB_DIR}")

set(CMAKE_MACOSX_RPATH ON)
# Sets CMAKE_INSTALL_RPATH if not set manually. This allows packaging on MacOS.
if(NOT CMAKE_INSTALL_RPATH)
  set(CMAKE_INSTALL_RPATH "${CMAKE_INSTALL_PREFIX}/lib")
  set(CMAKE_INSTALL_RPATH_USE_LINK_PATH TRUE)
endif()

# declare options
option(BUILD_SHARED_LIBS "Build director with shared libraries." ON)
option(BUILD_TESTING "Build director with tests enabled." OFF)

# include cmake scripts
include(cmake/dd-macros.cmake)
include(cmake/dd-testing.cmake)
include(cmake/dd-version.cmake)

# dependency options
option(USE_PORTMIDI "Build director with portmidi dependency." OFF)
option(USE_ROS "Build director with ros dependency." OFF)
option(USE_OCTOMAP "Build director with octomap dependency." OFF)

use_cpp11()
setup_qt()
setup_python()
setup_pkg_config_path()
set(DD_INSTALL_PYTHON_DIR ${DD_INSTALL_LIB_DIR}/python${PYTHON_VERSION_MAJOR}.${PYTHON_VERSION_MINOR}/site-packages)

# build project
add_subdirectory(src)

# documentation
add_subdirectory(docs)

include(cmake/dd-post-build.cmake)
