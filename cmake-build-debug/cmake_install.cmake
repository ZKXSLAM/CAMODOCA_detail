# Install script for directory: /home/zoukaixiang/code/camodocal

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

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/CAMODOCAL/FindBLAS.cmake;/usr/local/lib/cmake/CAMODOCAL/FindEigen3.cmake;/usr/local/lib/cmake/CAMODOCAL/FindGflags.cmake;/usr/local/lib/cmake/CAMODOCAL/FindGlog.cmake;/usr/local/lib/cmake/CAMODOCAL/FindGTest.cmake;/usr/local/lib/cmake/CAMODOCAL/FindLAPACK.cmake;/usr/local/lib/cmake/CAMODOCAL/FindLibDL.cmake;/usr/local/lib/cmake/CAMODOCAL/FindLibm.cmake;/usr/local/lib/cmake/CAMODOCAL/FindLibnsl.cmake;/usr/local/lib/cmake/CAMODOCAL/FindLibrt.cmake;/usr/local/lib/cmake/CAMODOCAL/FindOpenMP.cmake;/usr/local/lib/cmake/CAMODOCAL/FindSuiteSparse.cmake;/usr/local/lib/cmake/CAMODOCAL/FindTBB.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/CAMODOCAL" TYPE FILE FILES
    "/home/zoukaixiang/code/camodocal/cmake/FindBLAS.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindEigen3.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindGflags.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindGlog.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindGTest.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindLAPACK.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindLibDL.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindLibm.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindLibnsl.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindLibrt.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindOpenMP.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindSuiteSparse.cmake"
    "/home/zoukaixiang/code/camodocal/cmake/FindTBB.cmake"
    )
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  file(INSTALL DESTINATION "${CMAKE_INSTALL_PREFIX}/include" TYPE DIRECTORY FILES "/home/zoukaixiang/code/camodocal/include/camodocal")
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xUnspecifiedx" OR NOT CMAKE_INSTALL_COMPONENT)
  if(EXISTS "$ENV{DESTDIR}/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake")
    file(DIFFERENT EXPORT_FILE_CHANGED FILES
         "$ENV{DESTDIR}/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake"
         "/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles/Export/_usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake")
    if(EXPORT_FILE_CHANGED)
      file(GLOB OLD_CONFIG_FILES "$ENV{DESTDIR}/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets-*.cmake")
      if(OLD_CONFIG_FILES)
        message(STATUS "Old export file \"$ENV{DESTDIR}/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake\" will be replaced.  Removing files [${OLD_CONFIG_FILES}].")
        file(REMOVE ${OLD_CONFIG_FILES})
      endif()
    endif()
  endif()
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/CAMODOCAL" TYPE FILE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles/Export/_usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets.cmake")
  if("${CMAKE_INSTALL_CONFIG_NAME}" MATCHES "^([Dd][Ee][Bb][Uu][Gg])$")
    list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
     "/usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets-debug.cmake")
    if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
    if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
        message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
    endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/CAMODOCAL" TYPE FILE FILES "/home/zoukaixiang/code/camodocal/cmake-build-debug/CMakeFiles/Export/_usr/local/lib/cmake/CAMODOCAL/CAMODOCALTargets-debug.cmake")
  endif()
endif()

if("x${CMAKE_INSTALL_COMPONENT}x" STREQUAL "xDevelx" OR NOT CMAKE_INSTALL_COMPONENT)
  list(APPEND CMAKE_ABSOLUTE_DESTINATION_FILES
   "/usr/local/lib/cmake/CAMODOCAL/CAMODOCALConfig.cmake;/usr/local/lib/cmake/CAMODOCAL/CAMODOCALConfigVersion.cmake")
  if(CMAKE_WARN_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(WARNING "ABSOLUTE path INSTALL DESTINATION : ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
  if(CMAKE_ERROR_ON_ABSOLUTE_INSTALL_DESTINATION)
    message(FATAL_ERROR "ABSOLUTE path INSTALL DESTINATION forbidden (by caller): ${CMAKE_ABSOLUTE_DESTINATION_FILES}")
  endif()
file(INSTALL DESTINATION "/usr/local/lib/cmake/CAMODOCAL" TYPE FILE FILES
    "/home/zoukaixiang/code/camodocal/cmake-build-debug/CAMODOCALConfig.cmake"
    "/home/zoukaixiang/code/camodocal/cmake-build-debug/CAMODOCALConfigVersion.cmake"
    )
endif()

if(NOT CMAKE_INSTALL_LOCAL_ONLY)
  # Include the install script for each subdirectory.
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/cmake/doc/cmake_install.cmake")
  include("/home/zoukaixiang/code/camodocal/cmake-build-debug/src/cmake_install.cmake")

endif()

if(CMAKE_INSTALL_COMPONENT)
  set(CMAKE_INSTALL_MANIFEST "install_manifest_${CMAKE_INSTALL_COMPONENT}.txt")
else()
  set(CMAKE_INSTALL_MANIFEST "install_manifest.txt")
endif()

string(REPLACE ";" "\n" CMAKE_INSTALL_MANIFEST_CONTENT
       "${CMAKE_INSTALL_MANIFEST_FILES}")
file(WRITE "/home/zoukaixiang/code/camodocal/cmake-build-debug/${CMAKE_INSTALL_MANIFEST}"
     "${CMAKE_INSTALL_MANIFEST_CONTENT}")
