#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define GravityController_DLLIMPORT __declspec(dllimport)
#  define GravityController_DLLEXPORT __declspec(dllexport)
#  define GravityController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define GravityController_DLLIMPORT __attribute__((visibility("default")))
#    define GravityController_DLLEXPORT __attribute__((visibility("default")))
#    define GravityController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define GravityController_DLLIMPORT
#    define GravityController_DLLEXPORT
#    define GravityController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef GravityController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define GravityController_DLLAPI
#  define GravityController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef GravityController_EXPORTS
#    define GravityController_DLLAPI GravityController_DLLEXPORT
#  else
#    define GravityController_DLLAPI GravityController_DLLIMPORT
#  endif // GravityController_EXPORTS
#  define GravityController_LOCAL GravityController_DLLLOCAL
#endif // GravityController_STATIC