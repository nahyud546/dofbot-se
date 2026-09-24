# generated from rosidl_generator_py/resource/_idl.py.em
# with input from dofbot_interface:srv/CurJoint.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CurJoint_Request(type):
    """Metaclass of message 'CurJoint_Request'."""

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
                'dofbot_interface.srv.CurJoint_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cur_joint__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cur_joint__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cur_joint__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cur_joint__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cur_joint__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CurJoint_Request(metaclass=Metaclass_CurJoint_Request):
    """Message class 'CurJoint_Request'."""

    __slots__ = [
        '_srv_joints',
    ]

    _fields_and_field_types = {
        'srv_joints': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.srv_joints = kwargs.get('srv_joints', str())

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
        if self.srv_joints != other.srv_joints:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def srv_joints(self):
        """Message field 'srv_joints'."""
        return self._srv_joints

    @srv_joints.setter
    def srv_joints(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'srv_joints' field must be of type 'str'"
        self._srv_joints = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_CurJoint_Response(type):
    """Metaclass of message 'CurJoint_Response'."""

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
                'dofbot_interface.srv.CurJoint_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__cur_joint__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__cur_joint__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__cur_joint__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__cur_joint__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__cur_joint__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CurJoint_Response(metaclass=Metaclass_CurJoint_Response):
    """Message class 'CurJoint_Response'."""

    __slots__ = [
        '_srv_joint1',
        '_srv_joint2',
        '_srv_joint3',
        '_srv_joint4',
        '_srv_joint5',
        '_srv_joint6',
    ]

    _fields_and_field_types = {
        'srv_joint1': 'double',
        'srv_joint2': 'double',
        'srv_joint3': 'double',
        'srv_joint4': 'double',
        'srv_joint5': 'double',
        'srv_joint6': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.srv_joint1 = kwargs.get('srv_joint1', float())
        self.srv_joint2 = kwargs.get('srv_joint2', float())
        self.srv_joint3 = kwargs.get('srv_joint3', float())
        self.srv_joint4 = kwargs.get('srv_joint4', float())
        self.srv_joint5 = kwargs.get('srv_joint5', float())
        self.srv_joint6 = kwargs.get('srv_joint6', float())

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
        if self.srv_joint1 != other.srv_joint1:
            return False
        if self.srv_joint2 != other.srv_joint2:
            return False
        if self.srv_joint3 != other.srv_joint3:
            return False
        if self.srv_joint4 != other.srv_joint4:
            return False
        if self.srv_joint5 != other.srv_joint5:
            return False
        if self.srv_joint6 != other.srv_joint6:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def srv_joint1(self):
        """Message field 'srv_joint1'."""
        return self._srv_joint1

    @srv_joint1.setter
    def srv_joint1(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint1' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint1' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint1 = value

    @builtins.property
    def srv_joint2(self):
        """Message field 'srv_joint2'."""
        return self._srv_joint2

    @srv_joint2.setter
    def srv_joint2(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint2' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint2' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint2 = value

    @builtins.property
    def srv_joint3(self):
        """Message field 'srv_joint3'."""
        return self._srv_joint3

    @srv_joint3.setter
    def srv_joint3(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint3' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint3' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint3 = value

    @builtins.property
    def srv_joint4(self):
        """Message field 'srv_joint4'."""
        return self._srv_joint4

    @srv_joint4.setter
    def srv_joint4(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint4' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint4' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint4 = value

    @builtins.property
    def srv_joint5(self):
        """Message field 'srv_joint5'."""
        return self._srv_joint5

    @srv_joint5.setter
    def srv_joint5(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint5' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint5' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint5 = value

    @builtins.property
    def srv_joint6(self):
        """Message field 'srv_joint6'."""
        return self._srv_joint6

    @srv_joint6.setter
    def srv_joint6(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'srv_joint6' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'srv_joint6' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._srv_joint6 = value


class Metaclass_CurJoint(type):
    """Metaclass of service 'CurJoint'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('dofbot_interface')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'dofbot_interface.srv.CurJoint')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__cur_joint

            from dofbot_interface.srv import _cur_joint
            if _cur_joint.Metaclass_CurJoint_Request._TYPE_SUPPORT is None:
                _cur_joint.Metaclass_CurJoint_Request.__import_type_support__()
            if _cur_joint.Metaclass_CurJoint_Response._TYPE_SUPPORT is None:
                _cur_joint.Metaclass_CurJoint_Response.__import_type_support__()


class CurJoint(metaclass=Metaclass_CurJoint):
    from dofbot_interface.srv._cur_joint import CurJoint_Request as Request
    from dofbot_interface.srv._cur_joint import CurJoint_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
