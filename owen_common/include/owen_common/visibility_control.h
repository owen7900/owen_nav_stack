#ifndef OWEN_COMMON__VISIBILITY_CONTROL_H_
#define OWEN_COMMON__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
#ifdef __GNUC__
#define OWEN_COMMON_EXPORT __attribute__((dllexport))
#define OWEN_COMMON_IMPORT __attribute__((dllimport))
#else
#define OWEN_COMMON_EXPORT __declspec(dllexport)
#define OWEN_COMMON_IMPORT __declspec(dllimport)
#endif
#ifdef OWEN_COMMON_BUILDING_LIBRARY
#define OWEN_COMMON_PUBLIC OWEN_COMMON_EXPORT
#else
#define OWEN_COMMON_PUBLIC OWEN_COMMON_IMPORT
#endif
#define OWEN_COMMON_PUBLIC_TYPE OWEN_COMMON_PUBLIC
#define OWEN_COMMON_LOCAL
#else
#define OWEN_COMMON_EXPORT __attribute__((visibility("default")))
#define OWEN_COMMON_IMPORT
#if __GNUC__ >= 4
#define OWEN_COMMON_PUBLIC __attribute__((visibility("default")))
#define OWEN_COMMON_LOCAL __attribute__((visibility("hidden")))
#else
#define OWEN_COMMON_PUBLIC
#define OWEN_COMMON_LOCAL
#endif
#define OWEN_COMMON_PUBLIC_TYPE
#endif

#endif  // OWEN_COMMON__VISIBILITY_CONTROL_H_
