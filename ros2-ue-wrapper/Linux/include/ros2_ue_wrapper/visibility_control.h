

#ifndef ROS2_UE_WRAPPER__VISIBILITY_CONTROL_H_
#define ROS2_UE_WRAPPER__VISIBILITY_CONTROL_H_

#ifdef __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROS2_UE_WRAPPER_EXPORT __attribute__ ((dllexport))
    #define ROS2_UE_WRAPPER_IMPORT __attribute__ ((dllimport))
  #else
    #define ROS2_UE_WRAPPER_EXPORT __declspec(dllexport)
    #define ROS2_UE_WRAPPER_IMPORT __declspec(dllimport)
  #endif
  #ifdef ROS2_UE_WRAPPER_BUILDING_DLL
    #define ROS2_UE_WRAPPER_PUBLIC ROS2_UE_WRAPPER_EXPORT
  #else
    #define ROS2_UE_WRAPPER_PUBLIC ROS2_UE_WRAPPER_IMPORT
  #endif
  #define ROS2_UE_WRAPPER_PUBLIC_TYPE ROS2_UE_WRAPPER_PUBLIC
  #define ROS2_UE_WRAPPER_LOCAL
#else
  #define ROS2_UE_WRAPPER_EXPORT __attribute__ ((visibility("default")))
  #define ROS2_UE_WRAPPER_IMPORT
  #if __GNUC__ >= 4
    #define ROS2_UE_WRAPPER_PUBLIC __attribute__ ((visibility("default")))
    #define ROS2_UE_WRAPPER_LOCAL  __attribute__ ((visibility("hidden")))
  #else
    #define ROS2_UE_WRAPPER_PUBLIC
    #define ROS2_UE_WRAPPER_LOCAL
  #endif
  #define ROS2_UE_WRAPPER_PUBLIC_TYPE
#endif

#ifdef __cplusplus
}
#endif

#endif  // ROS2_UE_WRAPPER__VISIBILITY_CONTROL_H_
