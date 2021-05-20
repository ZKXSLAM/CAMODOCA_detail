# Config file for the CAMODOCAL package
# This adds CAMODOCAL IMPORTED targets
#
# Usage example:
# 
#   find_package(CAMODOCAL)
#   add_executable(foo foo.cpp)
#   target_link_libraries(foo CAMODOCALTargets)
#
# for a list of possible targets see the file
# CAMODOCALTargets.cmake in the same folder as this file

# todo: implement a mechanism for find_dependency scripts to work
#include(CMakeFindDependencyMacro)
#find_dependency(Gflags)
#find_dependency(Glog)
#find_dependency(OpenCV)
#find_dependency(Ceres)

# Our library dependencies (contains definitions for IMPORTED targets)
if(NOT TARGET CAMODOCAL)
  include("${CMAKE_CURRENT_LIST_DIR}/CAMODOCALTargets.cmake")
endif()

# 
####### Expanded from @PACKAGE_INIT@ by configure_package_config_file() #######
####### Any changes to this file will be overwritten by the next CMake run ####
####### The input file was CAMODOCALConfig.cmake.in                            ########

get_filename_component(PACKAGE_PREFIX_DIR "${CMAKE_CURRENT_LIST_DIR}/../../../" ABSOLUTE)

macro(set_and_check _var _file)
  set(${_var} "${_file}")
  if(NOT EXISTS "${_file}")
    message(FATAL_ERROR "File or directory ${_file} referenced by variable ${_var} does not exist !")
  endif()
endmacro()

macro(check_required_components _NAME)
  foreach(comp ${${_NAME}_FIND_COMPONENTS})
    if(NOT ${_NAME}_${comp}_FOUND)
      if(${_NAME}_FIND_REQUIRED_${comp})
        set(${_NAME}_FOUND FALSE)
      endif()
    endif()
  endforeach()
endmacro()

####################################################################################

check_required_components(CAMODOCAL)

