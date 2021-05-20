
#ifndef CAMODOCAL_POSE_GRAPH_EXPORT_H
#define CAMODOCAL_POSE_GRAPH_EXPORT_H

#ifdef CAMODOCAL_POSE_GRAPH_STATIC_DEFINE
#  define CAMODOCAL_POSE_GRAPH_EXPORT
#  define CAMODOCAL_POSE_GRAPH_NO_EXPORT
#else
#  ifndef CAMODOCAL_POSE_GRAPH_EXPORT
#    ifdef camodocal_pose_graph_EXPORTS
        /* We are building this library */
#      define CAMODOCAL_POSE_GRAPH_EXPORT __attribute__((visibility("default")))
#    else
        /* We are using this library */
#      define CAMODOCAL_POSE_GRAPH_EXPORT __attribute__((visibility("default")))
#    endif
#  endif

#  ifndef CAMODOCAL_POSE_GRAPH_NO_EXPORT
#    define CAMODOCAL_POSE_GRAPH_NO_EXPORT __attribute__((visibility("hidden")))
#  endif
#endif

#ifndef CAMODOCAL_POSE_GRAPH_DEPRECATED
#  define CAMODOCAL_POSE_GRAPH_DEPRECATED __attribute__ ((__deprecated__))
#endif

#ifndef CAMODOCAL_POSE_GRAPH_DEPRECATED_EXPORT
#  define CAMODOCAL_POSE_GRAPH_DEPRECATED_EXPORT CAMODOCAL_POSE_GRAPH_EXPORT CAMODOCAL_POSE_GRAPH_DEPRECATED
#endif

#ifndef CAMODOCAL_POSE_GRAPH_DEPRECATED_NO_EXPORT
#  define CAMODOCAL_POSE_GRAPH_DEPRECATED_NO_EXPORT CAMODOCAL_POSE_GRAPH_NO_EXPORT CAMODOCAL_POSE_GRAPH_DEPRECATED
#endif

#if 0 /* DEFINE_NO_DEPRECATED */
#  ifndef CAMODOCAL_POSE_GRAPH_NO_DEPRECATED
#    define CAMODOCAL_POSE_GRAPH_NO_DEPRECATED
#  endif
#endif

#endif /* CAMODOCAL_POSE_GRAPH_EXPORT_H */
