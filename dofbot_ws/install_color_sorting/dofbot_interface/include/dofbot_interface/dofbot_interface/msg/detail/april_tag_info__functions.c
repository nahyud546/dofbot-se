// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dofbot_interface:msg/AprilTagInfo.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/msg/detail/april_tag_info__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


bool
dofbot_interface__msg__AprilTagInfo__init(dofbot_interface__msg__AprilTagInfo * msg)
{
  if (!msg) {
    return false;
  }
  // id
  // x
  // y
  // z
  return true;
}

void
dofbot_interface__msg__AprilTagInfo__fini(dofbot_interface__msg__AprilTagInfo * msg)
{
  if (!msg) {
    return;
  }
  // id
  // x
  // y
  // z
}

bool
dofbot_interface__msg__AprilTagInfo__are_equal(const dofbot_interface__msg__AprilTagInfo * lhs, const dofbot_interface__msg__AprilTagInfo * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // id
  if (lhs->id != rhs->id) {
    return false;
  }
  // x
  if (lhs->x != rhs->x) {
    return false;
  }
  // y
  if (lhs->y != rhs->y) {
    return false;
  }
  // z
  if (lhs->z != rhs->z) {
    return false;
  }
  return true;
}

bool
dofbot_interface__msg__AprilTagInfo__copy(
  const dofbot_interface__msg__AprilTagInfo * input,
  dofbot_interface__msg__AprilTagInfo * output)
{
  if (!input || !output) {
    return false;
  }
  // id
  output->id = input->id;
  // x
  output->x = input->x;
  // y
  output->y = input->y;
  // z
  output->z = input->z;
  return true;
}

dofbot_interface__msg__AprilTagInfo *
dofbot_interface__msg__AprilTagInfo__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__AprilTagInfo * msg = (dofbot_interface__msg__AprilTagInfo *)allocator.allocate(sizeof(dofbot_interface__msg__AprilTagInfo), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__msg__AprilTagInfo));
  bool success = dofbot_interface__msg__AprilTagInfo__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__msg__AprilTagInfo__destroy(dofbot_interface__msg__AprilTagInfo * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__msg__AprilTagInfo__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__msg__AprilTagInfo__Sequence__init(dofbot_interface__msg__AprilTagInfo__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__AprilTagInfo * data = NULL;

  if (size) {
    data = (dofbot_interface__msg__AprilTagInfo *)allocator.zero_allocate(size, sizeof(dofbot_interface__msg__AprilTagInfo), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__msg__AprilTagInfo__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__msg__AprilTagInfo__fini(&data[i - 1]);
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
dofbot_interface__msg__AprilTagInfo__Sequence__fini(dofbot_interface__msg__AprilTagInfo__Sequence * array)
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
      dofbot_interface__msg__AprilTagInfo__fini(&array->data[i]);
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

dofbot_interface__msg__AprilTagInfo__Sequence *
dofbot_interface__msg__AprilTagInfo__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__AprilTagInfo__Sequence * array = (dofbot_interface__msg__AprilTagInfo__Sequence *)allocator.allocate(sizeof(dofbot_interface__msg__AprilTagInfo__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__msg__AprilTagInfo__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__msg__AprilTagInfo__Sequence__destroy(dofbot_interface__msg__AprilTagInfo__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__msg__AprilTagInfo__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__msg__AprilTagInfo__Sequence__are_equal(const dofbot_interface__msg__AprilTagInfo__Sequence * lhs, const dofbot_interface__msg__AprilTagInfo__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__msg__AprilTagInfo__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__msg__AprilTagInfo__Sequence__copy(
  const dofbot_interface__msg__AprilTagInfo__Sequence * input,
  dofbot_interface__msg__AprilTagInfo__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__msg__AprilTagInfo);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__msg__AprilTagInfo * data =
      (dofbot_interface__msg__AprilTagInfo *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__msg__AprilTagInfo__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__msg__AprilTagInfo__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__msg__AprilTagInfo__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
