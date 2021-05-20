
#ifndef CAMODOCAL_CALIB_EXPORT_H
#define CAMODOCAL_CALIB_EXPORT_H

#ifdef CAMODOCAL_CALIB_STATIC_DEFINE
#  define CAMODOCAL_CALIB_EXPORT
#  define CAMODOCAL_CALIB_NO_EXPORT
#else
#  ifndef CAMODOCAL_CALIB_EXPORT
#    ifdef camodocal_calib_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_CALIB_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_CALIB_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_CALIB_NO_EXPORT
#    define CAMODOCAL_CALIB_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_CALIB_DEPRECATED
#  define CAMODOCAL_CALIB_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_CALIB_DEPRECATED_EXPORT
#  define CAMODOCAL_CALIB_DEPRECATED_EXPORT CAMODOCAL_CALIB_EXPORT CAMODOCAL_CALIB_DEPRECATED
#endif

#ifndef CAMODOCAL_CALIB_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_CALIB_DEPRECATED_NO_EXPORT CAMODOCAL_CALIB_NO_EXPORT CAMODOCAL_CALIB_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_CALIB_NO_DEPRECATED
#    define CAMODOCAL_CALIB_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_CALIB_EXPORT_H */
