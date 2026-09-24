// generated from rosidl_generator_c/resource/idl__functions.c.em
// with input from dofbot_interface:msg/Yolov11Detect.idl
// generated code does not contain a copyright notice
#include "dofbot_interface/msg/detail/yolov11_detect__functions.h"

#include <assert.h>
#include <stdbool.h>
#include <stdlib.h>
#include <string.h>

#include "rcutils/allocator.h"


// Include directives for member types
// Member `result`
#include "rosidl_runtime_c/string_functions.h"

bool
dofbot_interface__msg__Yolov11Detect__init(dofbot_interface__msg__Yolov11Detect * msg)
{
  if (!msg) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__init(&msg->result)) {
    dofbot_interface__msg__Yolov11Detect__fini(msg);
    return false;
  }
  // centerx
  // centery
  return true;
}

void
dofbot_interface__msg__Yolov11Detect__fini(dofbot_interface__msg__Yolov11Detect * msg)
{
  if (!msg) {
    return;
  }
  // result
  rosidl_runtime_c__String__fini(&msg->result);
  // centerx
  // centery
}

bool
dofbot_interface__msg__Yolov11Detect__are_equal(const dofbot_interface__msg__Yolov11Detect * lhs, const dofbot_interface__msg__Yolov11Detect * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__are_equal(
      &(lhs->result), &(rhs->result)))
  {
    return false;
  }
  // centerx
  if (lhs->centerx != rhs->centerx) {
    return false;
  }
  // centery
  if (lhs->centery != rhs->centery) {
    return false;
  }
  return true;
}

bool
dofbot_interface__msg__Yolov11Detect__copy(
  const dofbot_interface__msg__Yolov11Detect * input,
  dofbot_interface__msg__Yolov11Detect * output)
{
  if (!input || !output) {
    return false;
  }
  // result
  if (!rosidl_runtime_c__String__copy(
      &(input->result), &(output->result)))
  {
    return false;
  }
  // centerx
  output->centerx = input->centerx;
  // centery
  output->centery = input->centery;
  return true;
}

dofbot_interface__msg__Yolov11Detect *
dofbot_interface__msg__Yolov11Detect__create()
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__Yolov11Detect * msg = (dofbot_interface__msg__Yolov11Detect *)allocator.allocate(sizeof(dofbot_interface__msg__Yolov11Detect), allocator.state);
  if (!msg) {
    return NULL;
  }
  memset(msg, 0, sizeof(dofbot_interface__msg__Yolov11Detect));
  bool success = dofbot_interface__msg__Yolov11Detect__init(msg);
  if (!success) {
    allocator.deallocate(msg, allocator.state);
    return NULL;
  }
  return msg;
}

void
dofbot_interface__msg__Yolov11Detect__destroy(dofbot_interface__msg__Yolov11Detect * msg)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (msg) {
    dofbot_interface__msg__Yolov11Detect__fini(msg);
  }
  allocator.deallocate(msg, allocator.state);
}


bool
dofbot_interface__msg__Yolov11Detect__Sequence__init(dofbot_interface__msg__Yolov11Detect__Sequence * array, size_t size)
{
  if (!array) {
    return false;
  }
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__Yolov11Detect * data = NULL;

  if (size) {
    data = (dofbot_interface__msg__Yolov11Detect *)allocator.zero_allocate(size, sizeof(dofbot_interface__msg__Yolov11Detect), allocator.state);
    if (!data) {
      return false;
    }
    // initialize all array elements
    size_t i;
    for (i = 0; i < size; ++i) {
      bool success = dofbot_interface__msg__Yolov11Detect__init(&data[i]);
      if (!success) {
        break;
      }
    }
    if (i < size) {
      // if initialization failed finalize the already initialized array elements
      for (; i > 0; --i) {
        dofbot_interface__msg__Yolov11Detect__fini(&data[i - 1]);
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
dofbot_interface__msg__Yolov11Detect__Sequence__fini(dofbot_interface__msg__Yolov11Detect__Sequence * array)
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
      dofbot_interface__msg__Yolov11Detect__fini(&array->data[i]);
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

dofbot_interface__msg__Yolov11Detect__Sequence *
dofbot_interface__msg__Yolov11Detect__Sequence__create(size_t size)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  dofbot_interface__msg__Yolov11Detect__Sequence * array = (dofbot_interface__msg__Yolov11Detect__Sequence *)allocator.allocate(sizeof(dofbot_interface__msg__Yolov11Detect__Sequence), allocator.state);
  if (!array) {
    return NULL;
  }
  bool success = dofbot_interface__msg__Yolov11Detect__Sequence__init(array, size);
  if (!success) {
    allocator.deallocate(array, allocator.state);
    return NULL;
  }
  return array;
}

void
dofbot_interface__msg__Yolov11Detect__Sequence__destroy(dofbot_interface__msg__Yolov11Detect__Sequence * array)
{
  rcutils_allocator_t allocator = rcutils_get_default_allocator();
  if (array) {
    dofbot_interface__msg__Yolov11Detect__Sequence__fini(array);
  }
  allocator.deallocate(array, allocator.state);
}

bool
dofbot_interface__msg__Yolov11Detect__Sequence__are_equal(const dofbot_interface__msg__Yolov11Detect__Sequence * lhs, const dofbot_interface__msg__Yolov11Detect__Sequence * rhs)
{
  if (!lhs || !rhs) {
    return false;
  }
  if (lhs->size != rhs->size) {
    return false;
  }
  for (size_t i = 0; i < lhs->size; ++i) {
    if (!dofbot_interface__msg__Yolov11Detect__are_equal(&(lhs->data[i]), &(rhs->data[i]))) {
      return false;
    }
  }
  return true;
}

bool
dofbot_interface__msg__Yolov11Detect__Sequence__copy(
  const dofbot_interface__msg__Yolov11Detect__Sequence * input,
  dofbot_interface__msg__Yolov11Detect__Sequence * output)
{
  if (!input || !output) {
    return false;
  }
  if (output->capacity < input->size) {
    const size_t allocation_size =
      input->size * sizeof(dofbot_interface__msg__Yolov11Detect);
    rcutils_allocator_t allocator = rcutils_get_default_allocator();
    dofbot_interface__msg__Yolov11Detect * data =
      (dofbot_interface__msg__Yolov11Detect *)allocator.reallocate(
      output->data, allocation_size, allocator.state);
    if (!data) {
      return false;
    }
    // If reallocation succeeded, memory may or may not have been moved
    // to fulfill the allocation request, invalidating output->data.
    output->data = data;
    for (size_t i = output->capacity; i < input->size; ++i) {
      if (!dofbot_interface__msg__Yolov11Detect__init(&output->data[i])) {
        // If initialization of any new item fails, roll back
        // all previously initialized items. Existing items
        // in output are to be left unmodified.
        for (; i-- > output->capacity; ) {
          dofbot_interface__msg__Yolov11Detect__fini(&output->data[i]);
        }
        return false;
      }
    }
    output->capacity = input->size;
  }
  output->size = input->size;
  for (size_t i = 0; i < input->size; ++i) {
    if (!dofbot_interface__msg__Yolov11Detect__copy(
        &(input->data[i]), &(output->data[i])))
    {
      return false;
    }
  }
  return true;
}
