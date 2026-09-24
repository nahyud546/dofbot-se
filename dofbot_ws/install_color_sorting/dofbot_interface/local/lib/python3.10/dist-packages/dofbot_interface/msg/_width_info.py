# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dofbot_interface:msg/WidthInfo.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_WidthInfo(type):
    """Metaclass of message 'WidthInfo'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
    }

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dofbot_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dofbot_interface.msg.WidthInfo')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__width_info
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__width_info
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__width_info
            cls._TYPE_SUPPORT = module.type_support_msg__msg__width_info
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__width_info

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class WidthInfo(metaclass=Metaclass_WidthInfo):
    """Message class 'WidthInfo'."""

    __slots__ = [
        '_l_x',
        '_l_y',
        '_r_x',
        '_r_y',
    ]

    _fields_and_field_types = {
        'l_x': 'float',
        'l_y': 'float',
        'r_x': 'float',
        'r_y': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.l_x = kwargs.get('l_x', float())
        self.l_y = kwargs.get('l_y', float())
        self.r_x = kwargs.get('r_x', float())
        self.r_y = kwargs.get('r_y', float())

    def __repr__(self):
        typename = self.__class__.__module__.split('.')
        typename.pop()
        typename.append(self.__class__.__name__)
        args = []
        for s, t in zip(self.__slots__, self.SLOT_TYPES):
            field = getattr(self, s)
            fieldstr = repr(field)
            # We use Python array type for fields that can be directly stored
            # in them, and "normal" sequences for everything else.  If it is
            # a type that we store in an array, strip off the 'array' portion.
            if (
                isinstance(t, rosidl_parser.definition.AbstractSequence) and
                isinstance(t.value_type, rosidl_parser.definition.BasicType) and
                t.value_type.typename in ['float', 'double', 'int8', 'uint8', 'int16', 'uint16', 'int32', 'uint32', 'int64', 'uint64']
            ):
                if len(field) == 0:
                    fieldstr = '[]'
                else:
                    assert fieldstr.startswith('array(')
                    prefix = "array('X', "
                    suffix = ')'
                    fieldstr = fieldstr[len(prefix):-len(suffix)]
            args.append(s[1:] + '=' + fieldstr)
        return '%s(%s)' % ('.'.join(typename), ', '.join(args))

    def __eq__(self, other):
        if not isinstance(other, self.__class__):
            return False
        if self.l_x != other.l_x:
            return False
        if self.l_y != other.l_y:
            return False
        if self.r_x != other.r_x:
            return False
        if self.r_y != other.r_y:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def l_x(self):
        """Message field 'l_x'."""
        return self._l_x

    @l_x.setter
    def l_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'l_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._l_x = value

    @builtins.property
    def l_y(self):
        """Message field 'l_y'."""
        return self._l_y

    @l_y.setter
    def l_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'l_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'l_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._l_y = value

    @builtins.property
    def r_x(self):
        """Message field 'r_x'."""
        return self._r_x

    @r_x.setter
    def r_x(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r_x' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'r_x' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._r_x = value

    @builtins.property
    def r_y(self):
        """Message field 'r_y'."""
        return self._r_y

    @r_y.setter
    def r_y(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'r_y' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'r_y' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._r_y = value
