#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define NewRLQPController_DLLIMPORT __declspec(dllimport)
#  define NewRLQPController_DLLEXPORT __declspec(dllexport)
#  define NewRLQPController_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define NewRLQPController_DLLIMPORT __attribute__((visibility("default")))
#    define NewRLQPController_DLLEXPORT __attribute__((visibility("default")))
#    define NewRLQPController_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define NewRLQPController_DLLIMPORT
#    define NewRLQPController_DLLEXPORT
#    define NewRLQPController_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef NewRLQPController_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define NewRLQPController_DLLAPI
#  define NewRLQPController_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef NewRLQPController_EXPORTS
#    define NewRLQPController_DLLAPI NewRLQPController_DLLEXPORT
#  else
#    define NewRLQPController_DLLAPI NewRLQPController_DLLIMPORT
#  endif // NewRLQPController_EXPORTS
#  define NewRLQPController_LOCAL NewRLQPController_DLLLOCAL
#endif // NewRLQPController_STATIC
