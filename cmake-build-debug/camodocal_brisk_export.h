
#ifndef CAMODOCAL_BRISK_EXPORT_H
#define CAMODOCAL_BRISK_EXPORT_H

#ifdef CAMODOCAL_BRISK_STATIC_DEFINE
#  define CAMODOCAL_BRISK_EXPORT
#  define CAMODOCAL_BRISK_NO_EXPORT
#else
#  ifndef CAMODOCAL_BRISK_EXPORT
#    ifdef camodocal_brisk_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_BRISK_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_BRISK_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_BRISK_NO_EXPORT
#    define CAMODOCAL_BRISK_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_BRISK_DEPRECATED
#  define CAMODOCAL_BRISK_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_BRISK_DEPRECATED_EXPORT
#  define CAMODOCAL_BRISK_DEPRECATED_EXPORT CAMODOCAL_BRISK_EXPORT CAMODOCAL_BRISK_DEPRECATED
#endif

#ifndef CAMODOCAL_BRISK_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_BRISK_DEPRECATED_NO_EXPORT CAMODOCAL_BRISK_NO_EXPORT CAMODOCAL_BRISK_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_BRISK_NO_DEPRECATED
#    define CAMODOCAL_BRISK_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_BRISK_EXPORT_H */
