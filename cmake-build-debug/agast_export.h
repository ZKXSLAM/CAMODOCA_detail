
#ifndef AGAST_EXPORT_H
#define AGAST_EXPORT_H

#ifdef AGAST_STATIC_DEFINE
#  define AGAST_EXPORT
#  define AGAST_NO_EXPORT
#else
#  ifndef AGAST_EXPORT
#    ifdef agast_EXPORTS
        /* We are building this library */
#      define AGAST_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define AGAST_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef AGAST_NO_EXPORT
#    define AGAST_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef AGAST_DEPRECATED
#  define AGAST_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef AGAST_DEPRECATED_EXPORT
#  define AGAST_DEPRECATED_EXPORT AGAST_EXPORT AGAST_DEPRECATED
#endif

#ifndef AGAST_DEPRECATED_NO_EXPORT
#  define AGAST_DEPRECATED_NO_EXPORT AGAST_NO_EXPORT AGAST_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef AGAST_NO_DEPRECATED
#    define AGAST_NO_DEPRECATED
#  endif
#endif

#endif /* AGAST_EXPORT_H */
