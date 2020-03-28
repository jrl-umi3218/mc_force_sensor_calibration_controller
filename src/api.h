#pragma once

#if defined _WIN32 || defined __CYGWIN__
#  define ForceSensorCalibration_DLLIMPORT __declspec(dllimport)
#  define ForceSensorCalibration_DLLEXPORT __declspec(dllexport)
#  define ForceSensorCalibration_DLLLOCAL
#else
// On Linux, for GCC >= 4, tag symbols using GCC extension.
#  if __GNUC__ >= 4
#    define ForceSensorCalibration_DLLIMPORT __attribute__((visibility("default")))
#    define ForceSensorCalibration_DLLEXPORT __attribute__((visibility("default")))
#    define ForceSensorCalibration_DLLLOCAL __attribute__((visibility("hidden")))
#  else
// Otherwise (GCC < 4 or another compiler is used), export everything.
#    define ForceSensorCalibration_DLLIMPORT
#    define ForceSensorCalibration_DLLEXPORT
#    define ForceSensorCalibration_DLLLOCAL
#  endif // __GNUC__ >= 4
#endif // defined _WIN32 || defined __CYGWIN__

#ifdef ForceSensorCalibration_STATIC
// If one is using the library statically, get rid of
// extra information.
#  define ForceSensorCalibration_DLLAPI
#  define ForceSensorCalibration_LOCAL
#else
// Depending on whether one is building or using the
// library define DLLAPI to import or export.
#  ifdef ForceSensorCalibration_EXPORTS
#    define ForceSensorCalibration_DLLAPI ForceSensorCalibration_DLLEXPORT
#  else
#    define ForceSensorCalibration_DLLAPI ForceSensorCalibration_DLLIMPORT
#  endif // ForceSensorCalibration_EXPORTS
#  define ForceSensorCalibration_LOCAL ForceSensorCalibration_DLLLOCAL
#endif // ForceSensorCalibration_STATIC