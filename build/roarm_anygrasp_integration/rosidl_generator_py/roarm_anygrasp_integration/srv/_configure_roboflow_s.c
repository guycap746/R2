// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
// generated code does not contain a copyright notice
#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
#include <Python.h>
#include <stdbool.h>
#ifndef _WIN32
# pragma GCC diagnostic push
# pragma GCC diagnostic ignored "-Wunused-function"
#endif
#include "numpy/ndarrayobject.h"
#ifndef _WIN32
# pragma GCC diagnostic pop
#endif
#include "rosidl_runtime_c/visibility_control.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"
#include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool roarm_anygrasp_integration__srv__configure_roboflow__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[77];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("roarm_anygrasp_integration.srv._configure_roboflow.ConfigureRoboflow_Request", full_classname_dest, 76) == 0);
  }
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * ros_message = _ros_message;
  {  // api_key
    PyObject * field = PyObject_GetAttrString(_pymsg, "api_key");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->api_key, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // workspace_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "workspace_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->workspace_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // project_name
    PyObject * field = PyObject_GetAttrString(_pymsg, "project_name");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->project_name, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // dataset_version
    PyObject * field = PyObject_GetAttrString(_pymsg, "dataset_version");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->dataset_version, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // auto_upload
    PyObject * field = PyObject_GetAttrString(_pymsg, "auto_upload");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->auto_upload = (Py_True == field);
    Py_DECREF(field);
  }
  {  // include_failed_grasps
    PyObject * field = PyObject_GetAttrString(_pymsg, "include_failed_grasps");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->include_failed_grasps = (Py_True == field);
    Py_DECREF(field);
  }
  {  // annotation_format
    PyObject * field = PyObject_GetAttrString(_pymsg, "annotation_format");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->annotation_format, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // min_confidence_for_upload
    PyObject * field = PyObject_GetAttrString(_pymsg, "min_confidence_for_upload");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->min_confidence_for_upload = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * roarm_anygrasp_integration__srv__configure_roboflow__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ConfigureRoboflow_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("roarm_anygrasp_integration.srv._configure_roboflow");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ConfigureRoboflow_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Request * ros_message = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Request *)raw_ros_message;
  {  // api_key
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->api_key.data,
      strlen(ros_message->api_key.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "api_key", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // workspace_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->workspace_name.data,
      strlen(ros_message->workspace_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "workspace_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // project_name
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->project_name.data,
      strlen(ros_message->project_name.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "project_name", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // dataset_version
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->dataset_version.data,
      strlen(ros_message->dataset_version.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "dataset_version", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // auto_upload
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->auto_upload ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "auto_upload", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // include_failed_grasps
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->include_failed_grasps ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "include_failed_grasps", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // annotation_format
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->annotation_format.data,
      strlen(ros_message->annotation_format.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "annotation_format", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // min_confidence_for_upload
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->min_confidence_for_upload);
    {
      int rc = PyObject_SetAttrString(_pymessage, "min_confidence_for_upload", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}

#define NPY_NO_DEPRECATED_API NPY_1_7_API_VERSION
// already included above
// #include <Python.h>
// already included above
// #include <stdbool.h>
// already included above
// #include "numpy/ndarrayobject.h"
// already included above
// #include "rosidl_runtime_c/visibility_control.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__struct.h"
// already included above
// #include "roarm_anygrasp_integration/srv/detail/configure_roboflow__functions.h"

// already included above
// #include "rosidl_runtime_c/string.h"
// already included above
// #include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool roarm_anygrasp_integration__srv__configure_roboflow__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[78];
    {
      char * class_name = NULL;
      char * module_name = NULL;
      {
        PyObject * class_attr = PyObject_GetAttrString(_pymsg, "__class__");
        if (class_attr) {
          PyObject * name_attr = PyObject_GetAttrString(class_attr, "__name__");
          if (name_attr) {
            class_name = (char *)PyUnicode_1BYTE_DATA(name_attr);
            Py_DECREF(name_attr);
          }
          PyObject * module_attr = PyObject_GetAttrString(class_attr, "__module__");
          if (module_attr) {
            module_name = (char *)PyUnicode_1BYTE_DATA(module_attr);
            Py_DECREF(module_attr);
          }
          Py_DECREF(class_attr);
        }
      }
      if (!class_name || !module_name) {
        return false;
      }
      snprintf(full_classname_dest, sizeof(full_classname_dest), "%s.%s", module_name, class_name);
    }
    assert(strncmp("roarm_anygrasp_integration.srv._configure_roboflow.ConfigureRoboflow_Response", full_classname_dest, 77) == 0);
  }
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * ros_message = _ros_message;
  {  // success
    PyObject * field = PyObject_GetAttrString(_pymsg, "success");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->success = (Py_True == field);
    Py_DECREF(field);
  }
  {  // message
    PyObject * field = PyObject_GetAttrString(_pymsg, "message");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->message, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // workspace_url
    PyObject * field = PyObject_GetAttrString(_pymsg, "workspace_url");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->workspace_url, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // project_url
    PyObject * field = PyObject_GetAttrString(_pymsg, "project_url");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->project_url, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }
  {  // total_images_in_dataset
    PyObject * field = PyObject_GetAttrString(_pymsg, "total_images_in_dataset");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->total_images_in_dataset = (int32_t)PyLong_AsLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * roarm_anygrasp_integration__srv__configure_roboflow__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of ConfigureRoboflow_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("roarm_anygrasp_integration.srv._configure_roboflow");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "ConfigureRoboflow_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  roarm_anygrasp_integration__srv__ConfigureRoboflow_Response * ros_message = (roarm_anygrasp_integration__srv__ConfigureRoboflow_Response *)raw_ros_message;
  {  // success
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->success ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "success", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // message
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->message.data,
      strlen(ros_message->message.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "message", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // workspace_url
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->workspace_url.data,
      strlen(ros_message->workspace_url.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "workspace_url", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // project_url
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->project_url.data,
      strlen(ros_message->project_url.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "project_url", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // total_images_in_dataset
    PyObject * field = NULL;
    field = PyLong_FromLong(ros_message->total_images_in_dataset);
    {
      int rc = PyObject_SetAttrString(_pymessage, "total_images_in_dataset", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
