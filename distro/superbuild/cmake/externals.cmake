option(USE_ROS "Build director with ros dependency." ON)
option(USE_OCTOMAP "Build with octomap." OFF)


option(USE_SYSTEM_VTK "Use system version of VTK.  If off, VTK will be built." ON)
if(NOT USE_SYSTEM_VTK AND NOT APPLE)
  option(USE_PRECOMPILED_VTK "Download and use precompiled VTK.  If off, VTK will be compiled from source." ON)
endif()

option(BUILD_SHARED_LIBS "Build director and externals with shared libraries." ON)

if(USE_DRAKE)
  set(DRAKE_SOURCE_DIR CACHE PATH "")
  set(DRAKE_SUPERBUILD_PREFIX_PATH "")
  if(DRAKE_SOURCE_DIR)
    set(DRAKE_SUPERBUILD_PREFIX_PATH "${DRAKE_SOURCE_DIR}/build/install")
    if(NOT EXISTS "${DRAKE_SUPERBUILD_PREFIX_PATH}")
        message(SEND_ERROR "Cannot find build directory in DRAKE_SOURCE_DIR: ${DRAKE_SOURCE_DIR}")
    endif()
  endif()
endif()

set(default_cmake_args
  "-DCMAKE_PREFIX_PATH:PATH=${install_prefix};${DRAKE_SUPERBUILD_PREFIX_PATH};${CMAKE_PREFIX_PATH}"
  "-DCMAKE_INSTALL_PREFIX:PATH=${install_prefix}"
  "-DCMAKE_BUILD_TYPE:STRING=${CMAKE_BUILD_TYPE}"
  "-DCMAKE_POSITION_INDEPENDENT_CODE:BOOL=ON"
  "-DBUILD_SHARED_LIBS:BOOL=${BUILD_SHARED_LIBS}"
  "-DBUILD_DOCUMENTATION:BOOL=OFF"
  "-DENABLE_TESTING:BOOL=OFF"
  "-DCMAKE_C_COMPILER_LAUNCHER:FILEPATH=${CMAKE_C_COMPILER_LAUNCHER}"
  "-DCMAKE_C_COMPILER:FILEPATH=${CMAKE_C_COMPILER}"
  "-DCMAKE_C_FLAGS:STRING=${CMAKE_C_FLAGS}"
  "-DCMAKE_CXX_COMPILER_LAUNCHER:FILEPATH=${CMAKE_CXX_COMPILER_LAUNCHER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_COMPILER:FILEPATH=${CMAKE_CXX_COMPILER}"
  "-DCMAKE_CXX_FLAGS:STRING=${CMAKE_CXX_FLAGS}"
  "-DCMAKE_EXE_LINKER_FLAGS:STRING=${CMAKE_EXE_LINKER_FLAGS}"
  "-DCMAKE_SHARED_LINKER_FLAGS:STRING=${CMAKE_SHARED_LINKER_FLAGS}"
  "-DCMAKE_VERBOSE_MAKEFILE:BOOL=${CMAKE_VERBOSE_MAKEFILE}"
  )

if(APPLE)
  list(APPEND default_cmake_args -DCMAKE_MACOSX_RPATH:BOOL=ON)
  if(CMAKE_INSTALL_RPATH)
    list(APPEND default_cmake_args "-DCMAKE_INSTALL_RPATH:STRING=${CMAKE_INSTALL_RPATH}")
  endif()
endif()

# Find required external dependencies
setup_qt()
if (DD_QT_VERSION EQUAL 4)
  set(qt_args
    -DQT_QMAKE_EXECUTABLE:FILEPATH=${QT_QMAKE_EXECUTABLE}
    )
else()
  set(qt_args
    -DQt5_DIR:PATH=${Qt5_DIR}
    -DQt5Core_DIR:PATH=${Qt5Core_DIR}
    -DQt5Gui_DIR:PATH=${Qt5Gui_DIR}
    -DQt5Widgets_DIR:PATH=${Qt5Widgets_DIR}
    )
endif()


if(APPLE)
  find_program(PYTHON_CONFIG_EXECUTABLE python-config)
  if (NOT PYTHON_CONFIG_EXECUTABLE)
    message(SEND_ERROR "python-config executable not found, but python is required.")
  endif()
  # using "python-config --prefix" so that cmake always uses the python that is
  # in the users path, this is a fix for homebrew on Mac:
  # https://github.com/Homebrew/homebrew/issues/25118
  execute_process(COMMAND ${PYTHON_CONFIG_EXECUTABLE} --prefix OUTPUT_VARIABLE python_prefix OUTPUT_STRIP_TRAILING_WHITESPACE)
  set(PYTHON_INCLUDE_DIR ${python_prefix}/include/python2.7)
  set(PYTHON_LIBRARY ${python_prefix}/lib/libpython2.7${CMAKE_SHARED_LIBRARY_SUFFIX})
else()
  find_package(PythonLibs 2.7 REQUIRED)
endif()
find_package(PythonInterp 2.7 REQUIRED)

set(python_args
  -DPYTHON_EXECUTABLE:PATH=${PYTHON_EXECUTABLE}
  -DPYTHON_INCLUDE_DIR:PATH=${PYTHON_INCLUDE_DIR}
  -DPYTHON_INCLUDE_DIR2:PATH=${PYTHON_INCLUDE_DIR} # required for cmake 2.8 on ubuntu 14.04
  -DPYTHON_LIBRARY:PATH=${PYTHON_LIBRARY}
  )


###############################################################################
# vtk

if(USE_SYSTEM_VTK)

  if(APPLE)
    set(vtk_homebrew_dir /usr/local/opt/vtk7/lib/cmake/vtk-7.1)
  endif()

  find_package(VTK REQUIRED PATHS ${vtk_homebrew_dir})
  if (VTK_VERSION VERSION_LESS 6.2)
    message(FATAL_ERROR "Director requires VTK version 6.2 or greater."
      " System has VTK version ${VTK_VERSION}")
  endif()
  check_vtk_qt_version()

  set(vtk_args -DVTK_DIR:PATH=${VTK_DIR})

elseif(USE_PRECOMPILED_VTK)

  set(url_base "http://patmarion.com/bottles")

  find_program(LSB_RELEASE lsb_release)
  message("++++++++++++++++++++++++XX")
  message(${LSB_RELEASE})
  message("||||||||||||||||||||||||XX")
  mark_as_advanced(LSB_RELEASE)
  message("++++++++++++++++++++++++0")
  message(${LSB_RELEASE})
  message("||||||||||||||||||||||||0")
  set(ubuntu_version)
  message("++++++++++++++++++++++++1")
  message(${LSB_RELEASE})
  message("||||||||||||||||||||||||1")
  message(STATUS ${ubuntu_version})
  message("++++++++++++++++++++++++1")
  if(LSB_RELEASE)
    message("a")
    execute_process(COMMAND ${LSB_RELEASE} -is
        OUTPUT_VARIABLE osname
        OUTPUT_STRIP_TRAILING_WHITESPACE)
    message("b")
    message(STATUS ${osname})
    if(osname STREQUAL Ubuntu)
      execute_process(COMMAND ${LSB_RELEASE} -rs
          OUTPUT_VARIABLE ubuntu_version
          OUTPUT_STRIP_TRAILING_WHITESPACE)
      message(${ubuntu_version})
    endif()
  endif()

  message("++++++++++++++++++++++++2")
  message(${LSB_RELEASE})
  message("||||||||||||||||||||||||2")
  message(STATUS ${ubuntu_version})
  message("++++++++++++++++++++++++2")

  if (LSB_RELEASE EQUAL 14.04)
    if(DD_QT_VERSION EQUAL 4)
      set(vtk_package_url ${url_base}/vtk7.1-qt4.8-python2.7-ubuntu14.04.tar.gz)
      set(vtk_package_md5 fe5c16f427a497b5713c52a68ecf564d)
    else()
      message(FATAL_ERROR "Compiling director with Qt5 is not supported on Ubuntu 14.04. "
               "Please set DD_QT_VERSION to 4.")
    endif()
  elseif(LSB_RELEASE EQUAL 16.04)
    if(DD_QT_VERSION EQUAL 4)
      set(vtk_package_url ${url_base}/vtk7.1-qt4.8-python2.7-ubuntu16.04.tar.gz)
      set(vtk_package_md5 1291e072405a3982b559ec011c3cf2a1)
    else()
      set(vtk_package_url ${url_base}/vtk7.1-qt5.5-python2.7-ubuntu16.04.tar.gz)
      set(vtk_package_md5 5ac930a7b1c083f975115d5970fb1a34)
    endif()
  else()
    message(FATAL_ERROR "USE_PRECOMPILED_VTK requires Ubuntu 14.04 or 16.04 "
            "but the detected system version does not match. "
            "Please disable USE_PRECOMPILED_VTK.")
  endif()

  ExternalProject_Add(vtk-precompiled
    URL ${vtk_package_url}
    URL_MD5 ${vtk_package_md5}
    CONFIGURE_COMMAND ""
    BUILD_COMMAND ""
    INSTALL_COMMAND ${CMAKE_COMMAND} -E copy_directory
      ${source_prefix}/vtk-precompiled ${install_prefix}
  )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/cmake/vtk-7.1)
  set(vtk_depends vtk-precompiled)

else()

  ExternalProject_Add(vtk
    GIT_REPOSITORY git://vtk.org/VTK.git
    GIT_TAG v8.0.0
    CMAKE_CACHE_ARGS
      ${default_cmake_args}
      ${python_args}
      ${qt_args}
      -DBUILD_TESTING:BOOL=OFF
      -DBUILD_EXAMPLES:BOOL=OFF
      -DVTK_RENDERING_BACKEND:STRING=OpenGL2
      -DVTK_QT_VERSION:STRING=${DD_QT_VERSION}
      -DVTK_PYTHON_VERSION:STRING=2
      -DModule_vtkGUISupportQt:BOOL=ON
      -DVTK_WRAP_PYTHON:BOOL=ON
    )

  set(vtk_args -DVTK_DIR:PATH=${install_prefix}/lib/cmake/vtk-7.1)
  set(vtk_depends vtk)

endif()
