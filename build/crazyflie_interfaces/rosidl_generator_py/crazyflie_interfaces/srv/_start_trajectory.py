# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:srv/StartTrajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_StartTrajectory_Request(type):
    """Metaclass of message 'StartTrajectory_Request'."""

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
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.StartTrajectory_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_trajectory__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_trajectory__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_trajectory__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_trajectory__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_trajectory__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartTrajectory_Request(metaclass=Metaclass_StartTrajectory_Request):
    """Message class 'StartTrajectory_Request'."""

    __slots__ = [
        '_group_mask',
        '_trajectory_id',
        '_timescale',
        '_reversed',
        '_relative',
    ]

    _fields_and_field_types = {
        'group_mask': 'uint8',
        'trajectory_id': 'uint8',
        'timescale': 'float',
        'reversed': 'boolean',
        'relative': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.group_mask = kwargs.get('group_mask', int())
        self.trajectory_id = kwargs.get('trajectory_id', int())
        self.timescale = kwargs.get('timescale', float())
        self.reversed = kwargs.get('reversed', bool())
        self.relative = kwargs.get('relative', bool())

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
        if self.group_mask != other.group_mask:
            return False
        if self.trajectory_id != other.trajectory_id:
            return False
        if self.timescale != other.timescale:
            return False
        if self.reversed != other.reversed:
            return False
        if self.relative != other.relative:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def group_mask(self):
        """Message field 'group_mask'."""
        return self._group_mask

    @group_mask.setter
    def group_mask(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'group_mask' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'group_mask' field must be an unsigned integer in [0, 255]"
        self._group_mask = value

    @builtins.property
    def trajectory_id(self):
        """Message field 'trajectory_id'."""
        return self._trajectory_id

    @trajectory_id.setter
    def trajectory_id(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'trajectory_id' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'trajectory_id' field must be an unsigned integer in [0, 255]"
        self._trajectory_id = value

    @builtins.property
    def timescale(self):
        """Message field 'timescale'."""
        return self._timescale

    @timescale.setter
    def timescale(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'timescale' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'timescale' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._timescale = value

    @builtins.property  # noqa: A003
    def reversed(self):  # noqa: A003
        """Message field 'reversed'."""
        return self._reversed

    @reversed.setter  # noqa: A003
    def reversed(self, value):  # noqa: A003
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'reversed' field must be of type 'bool'"
        self._reversed = value

    @builtins.property
    def relative(self):
        """Message field 'relative'."""
        return self._relative

    @relative.setter
    def relative(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'relative' field must be of type 'bool'"
        self._relative = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_StartTrajectory_Response(type):
    """Metaclass of message 'StartTrajectory_Response'."""

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
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.StartTrajectory_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__start_trajectory__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__start_trajectory__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__start_trajectory__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__start_trajectory__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__start_trajectory__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class StartTrajectory_Response(metaclass=Metaclass_StartTrajectory_Response):
    """Message class 'StartTrajectory_Response'."""

    __slots__ = [
    ]

    _fields_and_field_types = {
    }

    SLOT_TYPES = (
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))

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
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)


class Metaclass_StartTrajectory(type):
    """Metaclass of service 'StartTrajectory'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('crazyflie_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'crazyflie_interfaces.srv.StartTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__start_trajectory

            from crazyflie_interfaces.srv import _start_trajectory
            if _start_trajectory.Metaclass_StartTrajectory_Request._TYPE_SUPPORT is None:
                _start_trajectory.Metaclass_StartTrajectory_Request.__import_type_support__()
            if _start_trajectory.Metaclass_StartTrajectory_Response._TYPE_SUPPORT is None:
                _start_trajectory.Metaclass_StartTrajectory_Response.__import_type_support__()


class StartTrajectory(metaclass=Metaclass_StartTrajectory):
    from crazyflie_interfaces.srv._start_trajectory import StartTrajectory_Request as Request
    from crazyflie_interfaces.srv._start_trajectory import StartTrajectory_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
