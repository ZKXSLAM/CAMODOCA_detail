
#ifndef CAMODOCAL_GPL_EXPORT_H
#define CAMODOCAL_GPL_EXPORT_H

#ifdef CAMODOCAL_GPL_STATIC_DEFINE
#  define CAMODOCAL_GPL_EXPORT
#  define CAMODOCAL_GPL_NO_EXPORT
#else
#  ifndef CAMODOCAL_GPL_EXPORT
#    ifdef camodocal_gpl_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_GPL_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_GPL_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_GPL_NO_EXPORT
#    define CAMODOCAL_GPL_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_GPL_DEPRECATED
#  define CAMODOCAL_GPL_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_GPL_DEPRECATED_EXPORT
#  define CAMODOCAL_GPL_DEPRECATED_EXPORT CAMODOCAL_GPL_EXPORT CAMODOCAL_GPL_DEPRECATED
#endif

#ifndef CAMODOCAL_GPL_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_GPL_DEPRECATED_NO_EXPORT CAMODOCAL_GPL_NO_EXPORT CAMODOCAL_GPL_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_GPL_NO_DEPRECATED
#    define CAMODOCAL_GPL_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_GPL_EXPORT_H */
