#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define RandomGoalController_DLLIMPORT __declspec(dllimport)
#  define RandomGoalController_DLLEXPORT __declspec(dllexport)
#  define RandomGoalController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define RandomGoalController_DLLIMPORT __attribute__((visibility("default")))
#    define RandomGoalController_DLLEXPORT __attribute__((visibility("default")))
#    define RandomGoalController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define RandomGoalController_DLLIMPORT
#    define RandomGoalController_DLLEXPORT
#    define RandomGoalController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef RandomGoalController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define RandomGoalController_DLLAPI
#  define RandomGoalController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef RandomGoalController_EXPORTS
#    define RandomGoalController_DLLAPI RandomGoalController_DLLEXPORT
#  else
#    define RandomGoalController_DLLAPI RandomGoalController_DLLIMPORT
#  endif // RandomGoalController_EXPORTS
#  define RandomGoalController_LOCAL RandomGoalController_DLLLOCAL
#endif // RandomGoalController_STATIC