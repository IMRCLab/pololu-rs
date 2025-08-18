// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from crazyflie_interfaces:srv/Takeoff.idl
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
#include "crazyflie_interfaces/srv/detail/takeoff__struct.h"
#include "crazyflie_interfaces/srv/detail/takeoff__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool builtin_interfaces__msg__duration__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * builtin_interfaces__msg__duration__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__takeoff__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[50];
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
    assert(strncmp("crazyflie_interfaces.srv._takeoff.Takeoff_Request", full_classname_dest, 49) == 0);
  }
  crazyflie_interfaces__srv__Takeoff_Request * ros_message = _ros_message;
  {  // group_mask
    PyObject * field = PyObject_GetAttrString(_pymsg, "group_mask");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->group_mask = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // height
    PyObject * field = PyObject_GetAttrString(_pymsg, "height");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->height = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // duration
    PyObject * field = PyObject_GetAttrString(_pymsg, "duration");
    if (!field) {
      return false;
    }
    if (!builtin_interfaces__msg__duration__convert_from_py(field, &ros_message->duration)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__takeoff__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Takeoff_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._takeoff");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Takeoff_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crazyflie_interfaces__srv__Takeoff_Request * ros_message = (crazyflie_interfaces__srv__Takeoff_Request *)raw_ros_message;
  {  // group_mask
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->group_mask);
    {
      int rc = PyObject_SetAttrString(_pymessage, "group_mask", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // height
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->height);
    {
      int rc = PyObject_SetAttrString(_pymessage, "height", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // duration
    PyObject * field = NULL;
    field = builtin_interfaces__msg__duration__convert_to_py(&ros_message->duration);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "duration", field);
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
// #include "crazyflie_interfaces/srv/detail/takeoff__struct.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/takeoff__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__takeoff__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[51];
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
    assert(strncmp("crazyflie_interfaces.srv._takeoff.Takeoff_Response", full_classname_dest, 50) == 0);
  }
  crazyflie_interfaces__srv__Takeoff_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__takeoff__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Takeoff_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._takeoff");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Takeoff_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  (void)raw_ros_message;

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
