#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define ResidualLearningController_DLLIMPORT __declspec(dllimport)
#  define ResidualLearningController_DLLEXPORT __declspec(dllexport)
#  define ResidualLearningController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define ResidualLearningController_DLLIMPORT __attribute__((visibility("default")))
#    define ResidualLearningController_DLLEXPORT __attribute__((visibility("default")))
#    define ResidualLearningController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define ResidualLearningController_DLLIMPORT
#    define ResidualLearningController_DLLEXPORT
#    define ResidualLearningController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ResidualLearningController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ResidualLearningController_DLLAPI
#  define ResidualLearningController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef ResidualLearningController_EXPORTS
#    define ResidualLearningController_DLLAPI ResidualLearningController_DLLEXPORT
#  else
#    define ResidualLearningController_DLLAPI ResidualLearningController_DLLIMPORT
#  endif // ResidualLearningController_EXPORTS
#  define ResidualLearningController_LOCAL ResidualLearningController_DLLLOCAL
#endif // ResidualLearningController_STATIC