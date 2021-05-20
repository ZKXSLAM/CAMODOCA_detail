
#ifndef CAMODOCAL_DBOW2_EXPORT_H
#define CAMODOCAL_DBOW2_EXPORT_H

#ifdef CAMODOCAL_DBOW2_STATIC_DEFINE
#  define CAMODOCAL_DBOW2_EXPORT
#  define CAMODOCAL_DBOW2_NO_EXPORT
#else
#  ifndef CAMODOCAL_DBOW2_EXPORT
#    ifdef camodocal_dbow2_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_DBOW2_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_DBOW2_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_DBOW2_NO_EXPORT
#    define CAMODOCAL_DBOW2_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_DBOW2_DEPRECATED
#  define CAMODOCAL_DBOW2_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_DBOW2_DEPRECATED_EXPORT
#  define CAMODOCAL_DBOW2_DEPRECATED_EXPORT CAMODOCAL_DBOW2_EXPORT CAMODOCAL_DBOW2_DEPRECATED
#endif

#ifndef CAMODOCAL_DBOW2_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_DBOW2_DEPRECATED_NO_EXPORT CAMODOCAL_DBOW2_NO_EXPORT CAMODOCAL_DBOW2_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_DBOW2_NO_DEPRECATED
#    define CAMODOCAL_DBOW2_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_DBOW2_EXPORT_H */
