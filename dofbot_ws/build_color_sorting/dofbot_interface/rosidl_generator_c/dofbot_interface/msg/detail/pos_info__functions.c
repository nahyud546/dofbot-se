// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dofbot_interface:msg/PosInfo.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/msg/detail/pos_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `name`
#include "rosidl_runtime_c/string_functions.h"

bool
dofbot_interface__msg__PosInfo__init(dofbot_interface__msg__PosInfo * msg)
{
  if (!msg) {
    return false;
  }
  // name
  if (!rosidl_runtime_c__String__init(&msg->name)) {
    dofbot_interface__msg__PosInfo__fini(msg);
    return false;
  }
  // pos1
  // pos2
  // pos3
  // roll
  // pitch
  // yaw
  return true;
}

void
dofbot_interface__msg__PosInfo__fini(dofbot_interface__msg__PosInfo * msg)
{
  if (!msg) {
    return;
  }
  // name
  rosidl_runtime_c__String__fini(&msg->name);
  // pos1
  // pos2
  // pos3
  // roll
  // pitch
  // yaw
}

bool
dofbot_interface__msg__PosInfo__are_equal(const dofbot_interface__msg__PosInfo * lhs, const dofbot_interface__msg__PosInfo * rhs)
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
  // pos1
  if (lhs->pos1 != rhs->pos1) {
    return false;
  }
  // pos2
  if (lhs->pos2 != rhs->pos2) {
    return false;
  }
  // pos3
  if (lhs->pos3 != rhs->pos3) {
    return false;
  }
  // roll
  if (lhs->roll != rhs->roll) {
    return false;
  }
  // pitch
  if (lhs->pitch != rhs->pitch) {
    return false;
  }
  // yaw
  if (lhs->yaw != rhs->yaw) {
    return false;
  }
  return true;
}

bool
dofbot_interface__msg__PosInfo__copy(
  const dofbot_interface__msg__PosInfo * input,
  dofbot_interface__msg__PosInfo * output)
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
  // pos1
  output->pos1 = input->pos1;
  // pos2
  output->pos2 = input->pos2;
  // pos3
  output->pos3 = input->pos3;
  // roll
  output->roll = input->roll;
  // pitch
  output->pitch = input->pitch;
  // yaw
  output->yaw = input->yaw;
  return true;
}

dofbot_interface__msg__PosInfo *
dofbot_interface__msg__PosInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__PosInfo * msg = (dofbot_interface__msg__PosInfo *)allocator.allocate(sizeof(dofbot_interface__msg__PosInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__msg__PosInfo));
  bool success = dofbot_interface__msg__PosInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__msg__PosInfo__destroy(dofbot_interface__msg__PosInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__msg__PosInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__msg__PosInfo__Sequence__init(dofbot_interface__msg__PosInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__PosInfo * data = NULL;

  if (size) {
    data = (dofbot_interface__msg__PosInfo *)allocator.zero_allocate(size, sizeof(dofbot_interface__msg__PosInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__msg__PosInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__msg__PosInfo__fini(&data[i - 1]);
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
dofbot_interface__msg__PosInfo__Sequence__fini(dofbot_interface__msg__PosInfo__Sequence * array)
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
      dofbot_interface__msg__PosInfo__fini(&array->data[i]);
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

dofbot_interface__msg__PosInfo__Sequence *
dofbot_interface__msg__PosInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__PosInfo__Sequence * array = (dofbot_interface__msg__PosInfo__Sequence *)allocator.allocate(sizeof(dofbot_interface__msg__PosInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__msg__PosInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__msg__PosInfo__Sequence__destroy(dofbot_interface__msg__PosInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__msg__PosInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__msg__PosInfo__Sequence__are_equal(const dofbot_interface__msg__PosInfo__Sequence * lhs, const dofbot_interface__msg__PosInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__msg__PosInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__msg__PosInfo__Sequence__copy(
  const dofbot_interface__msg__PosInfo__Sequence * input,
  dofbot_interface__msg__PosInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__msg__PosInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__msg__PosInfo * data =
      (dofbot_interface__msg__PosInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__msg__PosInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__msg__PosInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__msg__PosInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
