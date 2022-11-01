#ifndef SEMANTIC_MAP_PLUGIN__VISIBILITY_CONTROL_H_
#define SEMANTIC_MAP_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define SEMANTIC_MAP_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define SEMANTIC_MAP_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define SEMANTIC_MAP_PLUGIN_EXPORT __declspec(dllexport)
    #define SEMANTIC_MAP_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef SEMANTIC_MAP_PLUGIN_BUILDING_LIBRARY
    #define SEMANTIC_MAP_PLUGIN_PUBLIC SEMANTIC_MAP_PLUGIN_EXPORT
  #else
    #define SEMANTIC_MAP_PLUGIN_PUBLIC SEMANTIC_MAP_PLUGIN_IMPORT
  #endif
  #define SEMANTIC_MAP_PLUGIN_PUBLIC_TYPE SEMANTIC_MAP_PLUGIN_PUBLIC
  #define SEMANTIC_MAP_PLUGIN_LOCAL
#else
  #define SEMANTIC_MAP_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define SEMANTIC_MAP_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define SEMANTIC_MAP_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define SEMANTIC_MAP_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define SEMANTIC_MAP_PLUGIN_PUBLIC
    #define SEMANTIC_MAP_PLUGIN_LOCAL
  #endif
  #define SEMANTIC_MAP_PLUGIN_PUBLIC_TYPE
#endif

#endif  // SEMANTIC_MAP_PLUGIN__VISIBILITY_CONTROL_H_
