// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dofbot_interface:msg/JointInfo.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/msg/detail/joint_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
dofbot_interface__msg__JointInfo__init(dofbot_interface__msg__JointInfo * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    dofbot_interface__msg__JointInfo__fini(msg);
    return false;
  }
  // joint1
  // joint2
  // joint3
  // joint4
  // joint5
  // joint6
  return true;
}

void
dofbot_interface__msg__JointInfo__fini(dofbot_interface__msg__JointInfo * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // joint1
  // joint2
  // joint3
  // joint4
  // joint5
  // joint6
}

bool
dofbot_interface__msg__JointInfo__are_equal(const dofbot_interface__msg__JointInfo * lhs, const dofbot_interface__msg__JointInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->name), &(rhs->name)))
  {
    return false;
  }
  // joint1
  if (lhs->joint1 != rhs->joint1) {
    return false;
  }
  // joint2
  if (lhs->joint2 != rhs->joint2) {
    return false;
  }
  // joint3
  if (lhs->joint3 != rhs->joint3) {
    return false;
  }
  // joint4
  if (lhs->joint4 != rhs->joint4) {
    return false;
  }
  // joint5
  if (lhs->joint5 != rhs->joint5) {
    return false;
  }
  // joint6
  if (lhs->joint6 != rhs->joint6) {
    return false;
  }
  return true;
}

bool
dofbot_interface__msg__JointInfo__copy(
  const dofbot_interface__msg__JointInfo * input,
  dofbot_interface__msg__JointInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__copy(
      &(input->name), &(output->name)))
  {
    return false;
  }
  // joint1
  output->joint1 = input->joint1;
  // joint2
  output->joint2 = input->joint2;
  // joint3
  output->joint3 = input->joint3;
  // joint4
  output->joint4 = input->joint4;
  // joint5
  output->joint5 = input->joint5;
  // joint6
  output->joint6 = input->joint6;
  return true;
}

dofbot_interface__msg__JointInfo *
dofbot_interface__msg__JointInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__JointInfo * msg = (dofbot_interface__msg__JointInfo *)allocator.allocate(sizeof(dofbot_interface__msg__JointInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__msg__JointInfo));
  bool success = dofbot_interface__msg__JointInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__msg__JointInfo__destroy(dofbot_interface__msg__JointInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__msg__JointInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__msg__JointInfo__Sequence__init(dofbot_interface__msg__JointInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__JointInfo * data = NULL;

  if (size) {
    data = (dofbot_interface__msg__JointInfo *)allocator.zero_allocate(size, sizeof(dofbot_interface__msg__JointInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__msg__JointInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__msg__JointInfo__fini(&data[i - 1]);
      }
      allocator.deallocate(data, allocator.state);
      return false;
    }
  }
  array->data = data;
  array->size = size;
  array->capacity = size;
  return true;
}

void
dofbot_interface__msg__JointInfo__Sequence__fini(dofbot_interface__msg__JointInfo__Sequence * array)
{
  if (!array) {
    return;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();

  if (array->data) {
    // ensure that data and capacity values are consistent
    assert(array->capacity > 0);
    // finalize all array elements
    for (size_t i = 0; i < array->capacity; ++i) {
      dofbot_interface__msg__JointInfo__fini(&array->data[i]);
    }
    allocator.deallocate(array->data, allocator.state);
    array->data = NULL;
    array->size = 0;
    array->capacity = 0;
  } else {
    // ensure that data, size, and capacity values are consistent
    assert(0 == array->size);
    assert(0 == array->capacity);
  }
}

dofbot_interface__msg__JointInfo__Sequence *
dofbot_interface__msg__JointInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__JointInfo__Sequence * array = (dofbot_interface__msg__JointInfo__Sequence *)allocator.allocate(sizeof(dofbot_interface__msg__JointInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__msg__JointInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__msg__JointInfo__Sequence__destroy(dofbot_interface__msg__JointInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__msg__JointInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__msg__JointInfo__Sequence__are_equal(const dofbot_interface__msg__JointInfo__Sequence * lhs, const dofbot_interface__msg__JointInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__msg__JointInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__msg__JointInfo__Sequence__copy(
  const dofbot_interface__msg__JointInfo__Sequence * input,
  dofbot_interface__msg__JointInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__msg__JointInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__msg__JointInfo * data =
      (dofbot_interface__msg__JointInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__msg__JointInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__msg__JointInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__msg__JointInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
