// generated from
// rosidl_typesupport_fastrtps_cpp/resource/rosidl_typesupport_fastrtps_cpp__visibility_control.h.in
// generated code does not contain a copyright notice

#ifndef ROARM_ANYGRASP_INTEGRATION__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
#define ROARM_ANYGRASP_INTEGRATION__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_

#if __cplusplus
extern "C"
{
#endif

// This logic was borrowed (then namespaced) from the examples on the gcc wiki:
//     https://gcc.gnu.org/wiki/Visibility

#if defined _WIN32 || defined __CYGWIN__
  #ifdef __GNUC__
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration __attribute__ ((dllexport))
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_roarm_anygrasp_integration __attribute__ ((dllimport))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration __declspec(dllexport)
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_roarm_anygrasp_integration __declspec(dllimport)
  #endif
  #ifdef ROSIDL_TYPESUPPORT_FASTRTPS_CPP_BUILDING_DLL_roarm_anygrasp_integration
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_roarm_anygrasp_integration
  #endif
#else
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_EXPORT_roarm_anygrasp_integration __attribute__ ((visibility("default")))
  #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_IMPORT_roarm_anygrasp_integration
  #if __GNUC__ >= 4
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration __attribute__ ((visibility("default")))
  #else
    #define ROSIDL_TYPESUPPORT_FASTRTPS_CPP_PUBLIC_roarm_anygrasp_integration
  #endif
#endif

#if __cplusplus
}
#endif

#endif  // ROARM_ANYGRASP_INTEGRATION__MSG__ROSIDL_TYPESUPPORT_FASTRTPS_CPP__VISIBILITY_CONTROL_H_
