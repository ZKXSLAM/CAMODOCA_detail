# Install script for directory: /home/zoukaixiang/code/camodocal/src

# Set the install prefix
if(NOT DEFINED CMAKE_INSTALL_PREFIX)
  set(CMAKE_INSTALL_PREFIX "/usr/local")
endif()
string(REGEX REPLACE "/$" "" CMAKE_INSTALL_PREFIX "${CMAKE_INSTALL_PREFIX}")

# Set the install configuration name.
if(NOT DEFINED CMAKE_INSTALL_CONFIG_NAME)
  if(BUILD_TYPE)
    string(REGEX REPLACE "^[^A-Za-z0-9_]+" ""
           CMAKE_INSTALL_CONFIG_NAME "${BUILD_TYPE}")
  else()
    set(CMAKE_INSTALL_CONFIG_NAME "Debug")
  endif()
  message(STATUS "Install configuration: \"${CMAKE_INSTALL_CONFIG_NAME}\"")
endif()

# Set the component getting installed.
if(NOT CMAKE_INSTALL_COMPONENT)
  if(COMPONENT)
    message(STATUS "Install component: \"${COMPONENT}\"")
    set(CMAKE_INSTALL_COMPONENT "${COMPONENT}")
  else()
    set(CMAKE_INSTALL_COMPONENT)
  endif()
endif()

# Install shared libraries without execute permission?
if(NOT DEFINED CMAKE_INSTALL_SO_NO_EXE)
  set(CMAKE_INSTALL_SO_NO_EXE "1")
endif()

# Is this installation the result of a crosscompile?
if(NOT DEFINED CMAKE_CROSSCOMPILING)
  set(CMAKE_CROSSCOMPILING "FALSE")
endif()

# Set default install directory permissions.
if(NOT DEFINED CMAKE_OBJDUMP)
  set(CMAKE_OBJDUMP "/usr/bin/objdump")
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/brisk/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/calib/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_models/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/camera_systems/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/chessboard/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/dbow2/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/examples/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/features2d/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/gpl/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/infrastr_calib/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/location_recognition/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/npoint/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/pose_estimation/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/pose_graph/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/pugixml/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/sparse_graph/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/visual_odometry/cmake_install.cmake")

endif()

