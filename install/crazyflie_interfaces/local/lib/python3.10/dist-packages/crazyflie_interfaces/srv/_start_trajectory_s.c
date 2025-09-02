// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from crazyflie_interfaces:srv/StartTrajectory.idl
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
#include "crazyflie_interfaces/srv/detail/start_trajectory__struct.h"
#include "crazyflie_interfaces/srv/detail/start_trajectory__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__start_trajectory__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[67];
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
    assert(strncmp("crazyflie_interfaces.srv._start_trajectory.StartTrajectory_Request", full_classname_dest, 66) == 0);
  }
  crazyflie_interfaces__srv__StartTrajectory_Request * ros_message = _ros_message;
  {  // group_mask
    PyObject * field = PyObject_GetAttrString(_pymsg, "group_mask");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->group_mask = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // trajectory_id
    PyObject * field = PyObject_GetAttrString(_pymsg, "trajectory_id");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->trajectory_id = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // timescale
    PyObject * field = PyObject_GetAttrString(_pymsg, "timescale");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->timescale = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // reversed
    PyObject * field = PyObject_GetAttrString(_pymsg, "reversed");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->reversed = (Py_True == field);
    Py_DECREF(field);
  }
  {  // relative
    PyObject * field = PyObject_GetAttrString(_pymsg, "relative");
    if (!field) {
      return false;
    }
    assert(PyBool_Check(field));
    ros_message->relative = (Py_True == field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__start_trajectory__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartTrajectory_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._start_trajectory");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartTrajectory_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crazyflie_interfaces__srv__StartTrajectory_Request * ros_message = (crazyflie_interfaces__srv__StartTrajectory_Request *)raw_ros_message;
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
  {  // trajectory_id
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->trajectory_id);
    {
      int rc = PyObject_SetAttrString(_pymessage, "trajectory_id", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // timescale
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->timescale);
    {
      int rc = PyObject_SetAttrString(_pymessage, "timescale", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // reversed
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->reversed ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "reversed", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // relative
    PyObject * field = NULL;
    field = PyBool_FromLong(ros_message->relative ? 1 : 0);
    {
      int rc = PyObject_SetAttrString(_pymessage, "relative", field);
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
// #include "crazyflie_interfaces/srv/detail/start_trajectory__struct.h"
// already included above
// #include "crazyflie_interfaces/srv/detail/start_trajectory__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__srv__start_trajectory__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[68];
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
    assert(strncmp("crazyflie_interfaces.srv._start_trajectory.StartTrajectory_Response", full_classname_dest, 67) == 0);
  }
  crazyflie_interfaces__srv__StartTrajectory_Response * ros_message = _ros_message;
  ros_message->structure_needs_at_least_one_member = 0;

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__srv__start_trajectory__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of StartTrajectory_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.srv._start_trajectory");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "StartTrajectory_Response");
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
