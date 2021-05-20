
#ifndef CAMODOCAL_DVISION_EXPORT_H
#define CAMODOCAL_DVISION_EXPORT_H

#ifdef CAMODOCAL_DVISION_STATIC_DEFINE
#  define CAMODOCAL_DVISION_EXPORT
#  define CAMODOCAL_DVISION_NO_EXPORT
#else
#  ifndef CAMODOCAL_DVISION_EXPORT
#    ifdef camodocal_dvision_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_DVISION_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_DVISION_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_DVISION_NO_EXPORT
#    define CAMODOCAL_DVISION_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_DVISION_DEPRECATED
#  define CAMODOCAL_DVISION_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_DVISION_DEPRECATED_EXPORT
#  define CAMODOCAL_DVISION_DEPRECATED_EXPORT CAMODOCAL_DVISION_EXPORT CAMODOCAL_DVISION_DEPRECATED
#endif

#ifndef CAMODOCAL_DVISION_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_DVISION_DEPRECATED_NO_EXPORT CAMODOCAL_DVISION_NO_EXPORT CAMODOCAL_DVISION_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_DVISION_NO_DEPRECATED
#    define CAMODOCAL_DVISION_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_DVISION_EXPORT_H */
