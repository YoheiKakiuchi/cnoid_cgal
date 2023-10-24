#ifndef CNOID_CNOID_CGAL_LIB_EXPORTDECL_H
#define CNOID_CNOID_CGAL_LIB_EXPORTDECL_H

# if defined _WIN32 || defined __CYGWIN__
#  define CNOID_CNOID_CGAL_LIB_DLLIMPORT __declspec(dllimport)
#  define CNOID_CNOID_CGAL_LIB_DLLEXPORT __declspec(dllexport)
#  define CNOID_CNOID_CGAL_LIB_DLLLOCAL
# else
#  if __GNUC__ >= 4
#   define CNOID_CNOID_CGAL_LIB_DLLIMPORT __attribute__ ((visibility("default")))
#   define CNOID_CNOID_CGAL_LIB_DLLEXPORT __attribute__ ((visibility("default")))
#   define CNOID_CNOID_CGAL_LIB_DLLLOCAL  __attribute__ ((visibility("hidden")))
#  else
#   define CNOID_CNOID_CGAL_LIB_DLLIMPORT
#   define CNOID_CNOID_CGAL_LIB_DLLEXPORT
#   define CNOID_CNOID_CGAL_LIB_DLLLOCAL
#  endif
# endif
# ifdef CNOID_CNOID_CGAL_LIB_STATIC
#  define CNOID_CNOID_CGAL_LIB_DLLAPI
#  define CNOID_CNOID_CGAL_LIB_LOCAL
# else
#  ifdef CnoidRobotAssemblerPlugin_EXPORTS
#   define CNOID_CNOID_CGAL_LIB_DLLAPI CNOID_CNOID_CGAL_LIB_DLLEXPORT
#  else
#   define CNOID_CNOID_CGAL_LIB_DLLAPI CNOID_CNOID_CGAL_LIB_DLLIMPORT
#  endif
#  define CNOID_CNOID_CGAL_LIB_LOCAL CNOID_CNOID_CGAL_LIB_DLLLOCAL
# endif
#endif // CNOID_CNOID_CGAL_LIB_EXPORTDECL_H

#ifdef CNOID_EXPORT
# undef CNOID_EXPORT
#endif
#define CNOID_EXPORT CNOID_CNOID_CGAL_LIB_DLLAPI
