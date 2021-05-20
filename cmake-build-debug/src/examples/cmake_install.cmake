# Install script for directory: /home/zoukaixiang/code/camodocal/src/examples

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib"
         RPATH "/usr/local/lib/camodocal:/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/intrinsic_calib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/bin/intrinsic_calib")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib"
         OLD_RPATH "/home/zoukaixiang/code/camodocal/cmake-build-debug/lib:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib/camodocal:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/intrinsic_calib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/stereo_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/stereo_calib")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/stereo_calib"
         RPATH "/usr/local/lib/camodocal:/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/stereo_calib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/bin/stereo_calib")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/stereo_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/stereo_calib")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/stereo_calib"
         OLD_RPATH "/home/zoukaixiang/code/camodocal/cmake-build-debug/lib:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib/camodocal:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/stereo_calib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib"
         RPATH "/usr/local/lib/camodocal:/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/extrinsic_calib")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/bin/extrinsic_calib")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib"
         OLD_RPATH "/home/zoukaixiang/code/camodocal/cmake-build-debug/lib:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib/camodocal:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/extrinsic_calib")
    endif()
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/train_voctree" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/train_voctree")
    file(RPATH_CHECK
         FILE "$ENV{DESTDIR}/usr/local/bin/train_voctree"
         RPATH "/usr/local/lib/camodocal:/usr/local/lib")
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/bin/train_voctree")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/bin" TYPE EXECUTABLE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/bin/train_voctree")
  if(EXISTS "$ENV{DESTDIR}/usr/local/bin/train_voctree" AND
     NOT IS_SYMLINK "$ENV{DESTDIR}/usr/local/bin/train_voctree")
    file(RPATH_CHANGE
         FILE "$ENV{DESTDIR}/usr/local/bin/train_voctree"
         OLD_RPATH "/home/zoukaixiang/code/camodocal/cmake-build-debug/lib:/usr/local/lib:"
         NEW_RPATH "/usr/local/lib/camodocal:/usr/local/lib")
    if(CMAKE_INSTALL_DO_STRIP)
      execute_process(COMMAND "/usr/bin/strip" "$ENV{DESTDIR}/usr/local/bin/train_voctree")
    endif()
  endif()
endif()

