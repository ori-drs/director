cmake_minimum_required(VERSION 2.8.5)
project(director)



# I don't know if this function is used. Another call to find catkin
# is used elsewhere
find_package(catkin  REQUIRED COMPONENTS bot_core_lcmtypes drc_lcmtypes
    multisense_utils image_utils bot_frames
    vs_lcmtypes multi_gait_executor_msgs quadruped_msgs)

catkin_package(
CATKIN_DEPENDS bot_core_lcmtypes vs_lcmtypes bot_core bot_param bot_frames maps_lcmtypes drc_lcmtypes multisense_utils
multisense_image_utils image_utils pcl_ros pcl_plugin forward_kinematics ctk_python_console  qt_property_browser python_qt
multi_gait_executor_msgs quadruped_msgs)


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

include(${CMAKE_SOURCE_DIR}/../../cmake/dd-macros.cmake)
include(ExternalProject)
set_property(DIRECTORY PROPERTY EP_PREFIX ${director_BINARY_DIR})

set(source_prefix ${director_BINARY_DIR}/src)
set(install_prefix ${director_BINARY_DIR}/install)
if (USE_EXTERNAL_INSTALL)
  set(install_prefix ${CMAKE_INSTALL_PREFIX})
else()
  mark_as_advanced(CMAKE_INSTALL_PREFIX)
endif()
include(cmake/externals.cmake)
add_subdirectory(../../ ./directory) #director





