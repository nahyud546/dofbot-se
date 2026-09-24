// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dofbot_interface:msg/WidthInfo.idl
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
#include "dofbot_interface/msg/detail/width_info__struct.h"
#include "dofbot_interface/msg/detail/width_info__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool dofbot_interface__msg__width_info__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[43];
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
    assert(strncmp("dofbot_interface.msg._width_info.WidthInfo", full_classname_dest, 42) == 0);
  }
  dofbot_interface__msg__WidthInfo * ros_message = _ros_message;
  {  // l_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "l_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // l_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "l_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->l_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r_x
    PyObject * field = PyObject_GetAttrString(_pymsg, "r_x");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r_x = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // r_y
    PyObject * field = PyObject_GetAttrString(_pymsg, "r_y");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->r_y = (float)PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dofbot_interface__msg__width_info__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of WidthInfo */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dofbot_interface.msg._width_info");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "WidthInfo");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dofbot_interface__msg__WidthInfo * ros_message = (dofbot_interface__msg__WidthInfo *)raw_ros_message;
  {  // l_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // l_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->l_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "l_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r_x
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r_x);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r_x", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // r_y
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->r_y);
    {
      int rc = PyObject_SetAttrString(_pymessage, "r_y", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
