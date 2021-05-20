
#ifndef CAMODOCAL_PUGIXML_EXPORT_H
#define CAMODOCAL_PUGIXML_EXPORT_H

#ifdef CAMODOCAL_PUGIXML_STATIC_DEFINE
#  define CAMODOCAL_PUGIXML_EXPORT
#  define CAMODOCAL_PUGIXML_NO_EXPORT
#else
#  ifndef CAMODOCAL_PUGIXML_EXPORT
#    ifdef camodocal_pugixml_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_PUGIXML_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_PUGIXML_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_PUGIXML_NO_EXPORT
#    define CAMODOCAL_PUGIXML_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_PUGIXML_DEPRECATED
#  define CAMODOCAL_PUGIXML_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_PUGIXML_DEPRECATED_EXPORT
#  define CAMODOCAL_PUGIXML_DEPRECATED_EXPORT CAMODOCAL_PUGIXML_EXPORT CAMODOCAL_PUGIXML_DEPRECATED
#endif

#ifndef CAMODOCAL_PUGIXML_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_PUGIXML_DEPRECATED_NO_EXPORT CAMODOCAL_PUGIXML_NO_EXPORT CAMODOCAL_PUGIXML_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_PUGIXML_NO_DEPRECATED
#    define CAMODOCAL_PUGIXML_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_PUGIXML_EXPORT_H */
