// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from crazyflie_interfaces:msg/Status.idl
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
#include "crazyflie_interfaces/msg/detail/status__struct.h"
#include "crazyflie_interfaces/msg/detail/status__functions.h"

ROSIDL_GENERATOR_C_IMPORT
bool std_msgs__msg__header__convert_from_py(PyObject * _pymsg, void * _ros_message);
ROSIDL_GENERATOR_C_IMPORT
PyObject * std_msgs__msg__header__convert_to_py(void * raw_ros_message);

ROSIDL_GENERATOR_C_EXPORT
bool crazyflie_interfaces__msg__status__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[40];
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
    assert(strncmp("crazyflie_interfaces.msg._status.Status", full_classname_dest, 39) == 0);
  }
  crazyflie_interfaces__msg__Status * ros_message = _ros_message;
  {  // header
    PyObject * field = PyObject_GetAttrString(_pymsg, "header");
    if (!field) {
      return false;
    }
    if (!std_msgs__msg__header__convert_from_py(field, &ros_message->header)) {
      Py_DECREF(field);
      return false;
    }
    Py_DECREF(field);
  }
  {  // supervisor_info
    PyObject * field = PyObject_GetAttrString(_pymsg, "supervisor_info");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->supervisor_info = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // battery_voltage
    PyObject * field = PyObject_GetAttrString(_pymsg, "battery_voltage");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->battery_voltage = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // pm_state
    PyObject * field = PyObject_GetAttrString(_pymsg, "pm_state");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->pm_state = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rssi
    PyObject * field = PyObject_GetAttrString(_pymsg, "rssi");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rssi = (uint8_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_rx_broadcast
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_rx_broadcast");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_rx_broadcast = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_tx_broadcast
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_tx_broadcast");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_tx_broadcast = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_rx_unicast
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_rx_unicast");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_rx_unicast = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // num_tx_unicast
    PyObject * field = PyObject_GetAttrString(_pymsg, "num_tx_unicast");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->num_tx_unicast = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // latency_unicast
    PyObject * field = PyObject_GetAttrString(_pymsg, "latency_unicast");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->latency_unicast = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * crazyflie_interfaces__msg__status__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Status */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("crazyflie_interfaces.msg._status");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Status");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  crazyflie_interfaces__msg__Status * ros_message = (crazyflie_interfaces__msg__Status *)raw_ros_message;
  {  // header
    PyObject * field = NULL;
    field = std_msgs__msg__header__convert_to_py(&ros_message->header);
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "header", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // supervisor_info
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->supervisor_info);
    {
      int rc = PyObject_SetAttrString(_pymessage, "supervisor_info", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // battery_voltage
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->battery_voltage);
    {
      int rc = PyObject_SetAttrString(_pymessage, "battery_voltage", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // pm_state
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->pm_state);
    {
      int rc = PyObject_SetAttrString(_pymessage, "pm_state", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rssi
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rssi);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rssi", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_rx_broadcast
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_rx_broadcast);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_rx_broadcast", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_tx_broadcast
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_tx_broadcast);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_tx_broadcast", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_rx_unicast
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_rx_unicast);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_rx_unicast", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // num_tx_unicast
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->num_tx_unicast);
    {
      int rc = PyObject_SetAttrString(_pymessage, "num_tx_unicast", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // latency_unicast
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->latency_unicast);
    {
      int rc = PyObject_SetAttrString(_pymessage, "latency_unicast", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
