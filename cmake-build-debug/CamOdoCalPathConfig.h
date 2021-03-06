/**
 @file
 @brief
    Various constants and options that only affect very little code.
 @note
    DO NOT EDIT THIS FILE!
    It has been automatically generated by CMake from CamOdoCalConfig.h.in
 */

#ifndef _VChargePathConfig_H__
#define _VChargePathConfig_H__

#include "VChargeConfig.h"

/* VCharge either gets installed to the system or just into a folder.
   The latter uses relative paths. */
/* #undef INSTALL_COPYABLE */

/* Using MSVC or XCode IDE */
/* #undef CMAKE_CONFIGURATION_TYPES */

/* Handle default ConfigValues */
namespace vcharge
{
    const char* const VCHARGE_RUNTIME_INSTALL_PATH ("");
}

#endif /* _VChargePathConfig_H__ */
