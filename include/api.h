#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define H1RLQPController_DLLIMPORT __declspec(dllimport)
#  define H1RLQPController_DLLEXPORT __declspec(dllexport)
#  define H1RLQPController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define H1RLQPController_DLLIMPORT __attribute__((visibility("default")))
#    define H1RLQPController_DLLEXPORT __attribute__((visibility("default")))
#    define H1RLQPController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define H1RLQPController_DLLIMPORT
#    define H1RLQPController_DLLEXPORT
#    define H1RLQPController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef H1RLQPController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define H1RLQPController_DLLAPI
#  define H1RLQPController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef H1RLQPController_EXPORTS
#    define H1RLQPController_DLLAPI H1RLQPController_DLLEXPORT
#  else
#    define H1RLQPController_DLLAPI H1RLQPController_DLLIMPORT
#  endif // H1RLQPController_EXPORTS
#  define H1RLQPController_LOCAL H1RLQPController_DLLLOCAL
#endif // H1RLQPController_STATIC
