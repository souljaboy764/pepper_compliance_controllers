#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define BolotnikovaController_DLLIMPORT __declspec(dllimport)
#  define BolotnikovaController_DLLEXPORT __declspec(dllexport)
#  define BolotnikovaController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define BolotnikovaController_DLLIMPORT __attribute__((visibility("default")))
#    define BolotnikovaController_DLLEXPORT __attribute__((visibility("default")))
#    define BolotnikovaController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define BolotnikovaController_DLLIMPORT
#    define BolotnikovaController_DLLEXPORT
#    define BolotnikovaController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef BolotnikovaController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define BolotnikovaController_DLLAPI
#  define BolotnikovaController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef BolotnikovaController_EXPORTS
#    define BolotnikovaController_DLLAPI BolotnikovaController_DLLEXPORT
#  else
#    define BolotnikovaController_DLLAPI BolotnikovaController_DLLIMPORT
#  endif // BolotnikovaController_EXPORTS
#  define BolotnikovaController_LOCAL BolotnikovaController_DLLLOCAL
#endif // BolotnikovaController_STATIC