#ifndef EXPAND_TO_GOAL_COSTMAP_PLUGIN__VISIBILITY_CONTROL_H_
#define EXPAND_TO_GOAL_COSTMAP_PLUGIN__VISIBILITY_CONTROL_H_

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_EXPORT __attribute__ ((dllexport))
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_IMPORT __attribute__ ((dllimport))
  #else
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_EXPORT __declspec(dllexport)
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_IMPORT __declspec(dllimport)
  #endif
  #ifdef EXPAND_TO_GOAL_COSTMAP_PLUGIN_BUILDING_LIBRARY
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC EXPAND_TO_GOAL_COSTMAP_PLUGIN_EXPORT
  #else
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC EXPAND_TO_GOAL_COSTMAP_PLUGIN_IMPORT
  #endif
  #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC_TYPE EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC
  #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_LOCAL
#else
  #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_EXPORT __attribute__ ((visibility("default")))
  #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_IMPORT
  #if __GNUC__ >= 4
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC __attribute__ ((visibility("default")))
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC
    #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_LOCAL
  #endif
  #define EXPAND_TO_GOAL_COSTMAP_PLUGIN_PUBLIC_TYPE
#endif

#endif  // EXPAND_TO_GOAL_COSTMAP_PLUGIN__VISIBILITY_CONTROL_H_
