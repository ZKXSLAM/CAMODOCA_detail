
#ifndef CAMODOCAL_DUTILS_EXPORT_H
#define CAMODOCAL_DUTILS_EXPORT_H

#ifdef CAMODOCAL_DUTILS_STATIC_DEFINE
#  define CAMODOCAL_DUTILS_EXPORT
#  define CAMODOCAL_DUTILS_NO_EXPORT
#else
#  ifndef CAMODOCAL_DUTILS_EXPORT
#    ifdef camodocal_dutils_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_DUTILS_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_DUTILS_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_DUTILS_NO_EXPORT
#    define CAMODOCAL_DUTILS_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_DUTILS_DEPRECATED
#  define CAMODOCAL_DUTILS_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_DUTILS_DEPRECATED_EXPORT
#  define CAMODOCAL_DUTILS_DEPRECATED_EXPORT CAMODOCAL_DUTILS_EXPORT CAMODOCAL_DUTILS_DEPRECATED
#endif

#ifndef CAMODOCAL_DUTILS_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_DUTILS_DEPRECATED_NO_EXPORT CAMODOCAL_DUTILS_NO_EXPORT CAMODOCAL_DUTILS_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_DUTILS_NO_DEPRECATED
#    define CAMODOCAL_DUTILS_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_DUTILS_EXPORT_H */
