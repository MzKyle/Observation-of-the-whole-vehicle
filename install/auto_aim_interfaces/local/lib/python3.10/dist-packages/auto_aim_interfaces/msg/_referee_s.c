// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from auto_aim_interfaces:msg/Referee.idl
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
#include "auto_aim_interfaces/msg/detail/referee__struct.h"
#include "auto_aim_interfaces/msg/detail/referee__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool auto_aim_interfaces__msg__referee__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[41];
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
    assert(strncmp("auto_aim_interfaces.msg._referee.Referee", full_classname_dest, 40) == 0);
  }
  auto_aim_interfaces__msg__Referee * ros_message = _ros_message;
  {  // event_data
    PyObject * field = PyObject_GetAttrString(_pymsg, "event_data");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->event_data = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // time
    PyObject * field = PyObject_GetAttrString(_pymsg, "time");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->time = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // rfid
    PyObject * field = PyObject_GetAttrString(_pymsg, "rfid");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->rfid = PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // base_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "base_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->base_hp = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // sentry_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "sentry_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->sentry_hp = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // outpost_hp
    PyObject * field = PyObject_GetAttrString(_pymsg, "outpost_hp");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->outpost_hp = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }
  {  // projectile_allowance_17mm
    PyObject * field = PyObject_GetAttrString(_pymsg, "projectile_allowance_17mm");
    if (!field) {
      return false;
    }
    assert(PyLong_Check(field));
    ros_message->projectile_allowance_17mm = (uint16_t)PyLong_AsUnsignedLong(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * auto_aim_interfaces__msg__referee__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of Referee */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("auto_aim_interfaces.msg._referee");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "Referee");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  auto_aim_interfaces__msg__Referee * ros_message = (auto_aim_interfaces__msg__Referee *)raw_ros_message;
  {  // event_data
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->event_data);
    {
      int rc = PyObject_SetAttrString(_pymessage, "event_data", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // time
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->time);
    {
      int rc = PyObject_SetAttrString(_pymessage, "time", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // rfid
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->rfid);
    {
      int rc = PyObject_SetAttrString(_pymessage, "rfid", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // base_hp
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->base_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "base_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // sentry_hp
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->sentry_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "sentry_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // outpost_hp
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->outpost_hp);
    {
      int rc = PyObject_SetAttrString(_pymessage, "outpost_hp", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // projectile_allowance_17mm
    PyObject * field = NULL;
    field = PyLong_FromUnsignedLong(ros_message->projectile_allowance_17mm);
    {
      int rc = PyObject_SetAttrString(_pymessage, "projectile_allowance_17mm", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
