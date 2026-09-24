// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dofbot_interface:srv/CurJoint.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/srv/detail/cur_joint__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"

// Include directives for member types
// Member `srv_joints`
#include "rosidl_runtime_c/string_functions.h"

bool
dofbot_interface__srv__CurJoint_Request__init(dofbot_interface__srv__CurJoint_Request * msg)
{
  if (!msg) {
    return false;
  }
  // srv_joints
  if (!rosidl_runtime_c__String__init(&msg->srv_joints)) {
    dofbot_interface__srv__CurJoint_Request__fini(msg);
    return false;
  }
  return true;
}

void
dofbot_interface__srv__CurJoint_Request__fini(dofbot_interface__srv__CurJoint_Request * msg)
{
  if (!msg) {
    return;
  }
  // srv_joints
  rosidl_runtime_c__String__fini(&msg->srv_joints);
}

bool
dofbot_interface__srv__CurJoint_Request__are_equal(const dofbot_interface__srv__CurJoint_Request * lhs, const dofbot_interface__srv__CurJoint_Request * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // srv_joints
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->srv_joints), &(rhs->srv_joints)))
  {
    return false;
  }
  return true;
}

bool
dofbot_interface__srv__CurJoint_Request__copy(
  const dofbot_interface__srv__CurJoint_Request * input,
  dofbot_interface__srv__CurJoint_Request * output)
{
  if (!input || !output) {
    return false;
  }
  // srv_joints
  if (!rosidl_runtime_c__String__copy(
      &(input->srv_joints), &(output->srv_joints)))
  {
    return false;
  }
  return true;
}

dofbot_interface__srv__CurJoint_Request *
dofbot_interface__srv__CurJoint_Request__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Request * msg = (dofbot_interface__srv__CurJoint_Request *)allocator.allocate(sizeof(dofbot_interface__srv__CurJoint_Request), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__srv__CurJoint_Request));
  bool success = dofbot_interface__srv__CurJoint_Request__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__srv__CurJoint_Request__destroy(dofbot_interface__srv__CurJoint_Request * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__srv__CurJoint_Request__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__srv__CurJoint_Request__Sequence__init(dofbot_interface__srv__CurJoint_Request__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Request * data = NULL;

  if (size) {
    data = (dofbot_interface__srv__CurJoint_Request *)allocator.zero_allocate(size, sizeof(dofbot_interface__srv__CurJoint_Request), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__srv__CurJoint_Request__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__srv__CurJoint_Request__fini(&data[i - 1]);
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
dofbot_interface__srv__CurJoint_Request__Sequence__fini(dofbot_interface__srv__CurJoint_Request__Sequence * array)
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
      dofbot_interface__srv__CurJoint_Request__fini(&array->data[i]);
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

dofbot_interface__srv__CurJoint_Request__Sequence *
dofbot_interface__srv__CurJoint_Request__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Request__Sequence * array = (dofbot_interface__srv__CurJoint_Request__Sequence *)allocator.allocate(sizeof(dofbot_interface__srv__CurJoint_Request__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__srv__CurJoint_Request__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__srv__CurJoint_Request__Sequence__destroy(dofbot_interface__srv__CurJoint_Request__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__srv__CurJoint_Request__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__srv__CurJoint_Request__Sequence__are_equal(const dofbot_interface__srv__CurJoint_Request__Sequence * lhs, const dofbot_interface__srv__CurJoint_Request__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__srv__CurJoint_Request__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__srv__CurJoint_Request__Sequence__copy(
  const dofbot_interface__srv__CurJoint_Request__Sequence * input,
  dofbot_interface__srv__CurJoint_Request__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__srv__CurJoint_Request);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__srv__CurJoint_Request * data =
      (dofbot_interface__srv__CurJoint_Request *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__srv__CurJoint_Request__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__srv__CurJoint_Request__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__srv__CurJoint_Request__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}


bool
dofbot_interface__srv__CurJoint_Response__init(dofbot_interface__srv__CurJoint_Response * msg)
{
  if (!msg) {
    return false;
  }
  // srv_joint1
  // srv_joint2
  // srv_joint3
  // srv_joint4
  // srv_joint5
  // srv_joint6
  return true;
}

void
dofbot_interface__srv__CurJoint_Response__fini(dofbot_interface__srv__CurJoint_Response * msg)
{
  if (!msg) {
    return;
  }
  // srv_joint1
  // srv_joint2
  // srv_joint3
  // srv_joint4
  // srv_joint5
  // srv_joint6
}

bool
dofbot_interface__srv__CurJoint_Response__are_equal(const dofbot_interface__srv__CurJoint_Response * lhs, const dofbot_interface__srv__CurJoint_Response * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // srv_joint1
  if (lhs->srv_joint1 != rhs->srv_joint1) {
    return false;
  }
  // srv_joint2
  if (lhs->srv_joint2 != rhs->srv_joint2) {
    return false;
  }
  // srv_joint3
  if (lhs->srv_joint3 != rhs->srv_joint3) {
    return false;
  }
  // srv_joint4
  if (lhs->srv_joint4 != rhs->srv_joint4) {
    return false;
  }
  // srv_joint5
  if (lhs->srv_joint5 != rhs->srv_joint5) {
    return false;
  }
  // srv_joint6
  if (lhs->srv_joint6 != rhs->srv_joint6) {
    return false;
  }
  return true;
}

bool
dofbot_interface__srv__CurJoint_Response__copy(
  const dofbot_interface__srv__CurJoint_Response * input,
  dofbot_interface__srv__CurJoint_Response * output)
{
  if (!input || !output) {
    return false;
  }
  // srv_joint1
  output->srv_joint1 = input->srv_joint1;
  // srv_joint2
  output->srv_joint2 = input->srv_joint2;
  // srv_joint3
  output->srv_joint3 = input->srv_joint3;
  // srv_joint4
  output->srv_joint4 = input->srv_joint4;
  // srv_joint5
  output->srv_joint5 = input->srv_joint5;
  // srv_joint6
  output->srv_joint6 = input->srv_joint6;
  return true;
}

dofbot_interface__srv__CurJoint_Response *
dofbot_interface__srv__CurJoint_Response__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Response * msg = (dofbot_interface__srv__CurJoint_Response *)allocator.allocate(sizeof(dofbot_interface__srv__CurJoint_Response), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__srv__CurJoint_Response));
  bool success = dofbot_interface__srv__CurJoint_Response__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__srv__CurJoint_Response__destroy(dofbot_interface__srv__CurJoint_Response * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__srv__CurJoint_Response__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__srv__CurJoint_Response__Sequence__init(dofbot_interface__srv__CurJoint_Response__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Response * data = NULL;

  if (size) {
    data = (dofbot_interface__srv__CurJoint_Response *)allocator.zero_allocate(size, sizeof(dofbot_interface__srv__CurJoint_Response), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__srv__CurJoint_Response__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__srv__CurJoint_Response__fini(&data[i - 1]);
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
dofbot_interface__srv__CurJoint_Response__Sequence__fini(dofbot_interface__srv__CurJoint_Response__Sequence * array)
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
      dofbot_interface__srv__CurJoint_Response__fini(&array->data[i]);
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

dofbot_interface__srv__CurJoint_Response__Sequence *
dofbot_interface__srv__CurJoint_Response__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__srv__CurJoint_Response__Sequence * array = (dofbot_interface__srv__CurJoint_Response__Sequence *)allocator.allocate(sizeof(dofbot_interface__srv__CurJoint_Response__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__srv__CurJoint_Response__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__srv__CurJoint_Response__Sequence__destroy(dofbot_interface__srv__CurJoint_Response__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__srv__CurJoint_Response__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__srv__CurJoint_Response__Sequence__are_equal(const dofbot_interface__srv__CurJoint_Response__Sequence * lhs, const dofbot_interface__srv__CurJoint_Response__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__srv__CurJoint_Response__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__srv__CurJoint_Response__Sequence__copy(
  const dofbot_interface__srv__CurJoint_Response__Sequence * input,
  dofbot_interface__srv__CurJoint_Response__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__srv__CurJoint_Response);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__srv__CurJoint_Response * data =
      (dofbot_interface__srv__CurJoint_Response *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__srv__CurJoint_Response__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__srv__CurJoint_Response__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__srv__CurJoint_Response__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
