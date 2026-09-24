// generated from rosidl_generator_py/resource/_idl_support.c.em
// with input from dofbot_interface:srv/CurJoint.idl
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
#include "dofbot_interface/srv/detail/cur_joint__struct.h"
#include "dofbot_interface/srv/detail/cur_joint__functions.h"

#include "rosidl_runtime_c/string.h"
#include "rosidl_runtime_c/string_functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool dofbot_interface__srv__cur_joint__request__convert_from_py(PyObject * _pymsg, void * _ros_message)
{
  // check that the passed message is of the expected Python class
  {
    char full_classname_dest[49];
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
    assert(strncmp("dofbot_interface.srv._cur_joint.CurJoint_Request", full_classname_dest, 48) == 0);
  }
  dofbot_interface__srv__CurJoint_Request * ros_message = _ros_message;
  {  // srv_joints
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joints");
    if (!field) {
      return false;
    }
    assert(PyUnicode_Check(field));
    PyObject * encoded_field = PyUnicode_AsUTF8String(field);
    if (!encoded_field) {
      Py_DECREF(field);
      return false;
    }
    rosidl_runtime_c__String__assign(&ros_message->srv_joints, PyBytes_AS_STRING(encoded_field));
    Py_DECREF(encoded_field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dofbot_interface__srv__cur_joint__request__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CurJoint_Request */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dofbot_interface.srv._cur_joint");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CurJoint_Request");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dofbot_interface__srv__CurJoint_Request * ros_message = (dofbot_interface__srv__CurJoint_Request *)raw_ros_message;
  {  // srv_joints
    PyObject * field = NULL;
    field = PyUnicode_DecodeUTF8(
      ros_message->srv_joints.data,
      strlen(ros_message->srv_joints.data),
      "replace");
    if (!field) {
      return NULL;
    }
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joints", field);
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
// #include "dofbot_interface/srv/detail/cur_joint__struct.h"
// already included above
// #include "dofbot_interface/srv/detail/cur_joint__functions.h"


ROSIDL_GENERATOR_C_EXPORT
bool dofbot_interface__srv__cur_joint__response__convert_from_py(PyObject * _pymsg, void * _ros_message)
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
    assert(strncmp("dofbot_interface.srv._cur_joint.CurJoint_Response", full_classname_dest, 49) == 0);
  }
  dofbot_interface__srv__CurJoint_Response * ros_message = _ros_message;
  {  // srv_joint1
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint1");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint1 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srv_joint2
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint2");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint2 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srv_joint3
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint3");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint3 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srv_joint4
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint4");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint4 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srv_joint5
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint5");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint5 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }
  {  // srv_joint6
    PyObject * field = PyObject_GetAttrString(_pymsg, "srv_joint6");
    if (!field) {
      return false;
    }
    assert(PyFloat_Check(field));
    ros_message->srv_joint6 = PyFloat_AS_DOUBLE(field);
    Py_DECREF(field);
  }

  return true;
}

ROSIDL_GENERATOR_C_EXPORT
PyObject * dofbot_interface__srv__cur_joint__response__convert_to_py(void * raw_ros_message)
{
  /* NOTE(esteve): Call constructor of CurJoint_Response */
  PyObject * _pymessage = NULL;
  {
    PyObject * pymessage_module = PyImport_ImportModule("dofbot_interface.srv._cur_joint");
    assert(pymessage_module);
    PyObject * pymessage_class = PyObject_GetAttrString(pymessage_module, "CurJoint_Response");
    assert(pymessage_class);
    Py_DECREF(pymessage_module);
    _pymessage = PyObject_CallObject(pymessage_class, NULL);
    Py_DECREF(pymessage_class);
    if (!_pymessage) {
      return NULL;
    }
  }
  dofbot_interface__srv__CurJoint_Response * ros_message = (dofbot_interface__srv__CurJoint_Response *)raw_ros_message;
  {  // srv_joint1
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint1);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint1", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srv_joint2
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint2);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint2", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srv_joint3
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint3);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint3", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srv_joint4
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint4);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint4", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srv_joint5
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint5);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint5", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }
  {  // srv_joint6
    PyObject * field = NULL;
    field = PyFloat_FromDouble(ros_message->srv_joint6);
    {
      int rc = PyObject_SetAttrString(_pymessage, "srv_joint6", field);
      Py_DECREF(field);
      if (rc) {
        return NULL;
      }
    }
  }

  // ownership of _pymessage is transferred to the caller
  return _pymessage;
}
