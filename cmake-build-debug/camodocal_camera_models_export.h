
#ifndef CAMODOCAL_CAMERA_MODELS_EXPORT_H
#define CAMODOCAL_CAMERA_MODELS_EXPORT_H

#ifdef CAMODOCAL_CAMERA_MODELS_STATIC_DEFINE
#  define CAMODOCAL_CAMERA_MODELS_EXPORT
#  define CAMODOCAL_CAMERA_MODELS_NO_EXPORT
#else
#  ifndef CAMODOCAL_CAMERA_MODELS_EXPORT
#    ifdef camodocal_camera_models_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_CAMERA_MODELS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_CAMERA_MODELS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_CAMERA_MODELS_NO_EXPORT
#    define CAMODOCAL_CAMERA_MODELS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_CAMERA_MODELS_DEPRECATED
#  define CAMODOCAL_CAMERA_MODELS_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_CAMERA_MODELS_DEPRECATED_EXPORT
#  define CAMODOCAL_CAMERA_MODELS_DEPRECATED_EXPORT CAMODOCAL_CAMERA_MODELS_EXPORT CAMODOCAL_CAMERA_MODELS_DEPRECATED
#endif

#ifndef CAMODOCAL_CAMERA_MODELS_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_CAMERA_MODELS_DEPRECATED_NO_EXPORT CAMODOCAL_CAMERA_MODELS_NO_EXPORT CAMODOCAL_CAMERA_MODELS_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_CAMERA_MODELS_NO_DEPRECATED
#    define CAMODOCAL_CAMERA_MODELS_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_CAMERA_MODELS_EXPORT_H */