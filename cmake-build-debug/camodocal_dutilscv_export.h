
#ifndef CAMODOCAL_DUTILSCV_EXPORT_H
#define CAMODOCAL_DUTILSCV_EXPORT_H

#ifdef CAMODOCAL_DUTILSCV_STATIC_DEFINE
#  define CAMODOCAL_DUTILSCV_EXPORT
#  define CAMODOCAL_DUTILSCV_NO_EXPORT
#else
#  ifndef CAMODOCAL_DUTILSCV_EXPORT
#    ifdef camodocal_dutilscv_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_DUTILSCV_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_DUTILSCV_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_DUTILSCV_NO_EXPORT
#    define CAMODOCAL_DUTILSCV_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_DUTILSCV_DEPRECATED
#  define CAMODOCAL_DUTILSCV_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_DUTILSCV_DEPRECATED_EXPORT
#  define CAMODOCAL_DUTILSCV_DEPRECATED_EXPORT CAMODOCAL_DUTILSCV_EXPORT CAMODOCAL_DUTILSCV_DEPRECATED
#endif

#ifndef CAMODOCAL_DUTILSCV_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_DUTILSCV_DEPRECATED_NO_EXPORT CAMODOCAL_DUTILSCV_NO_EXPORT CAMODOCAL_DUTILSCV_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_DUTILSCV_NO_DEPRECATED
#    define CAMODOCAL_DUTILSCV_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_DUTILSCV_EXPORT_H */
