# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:msg/TrajectoryPolynomialPiece.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'poly_x'
# Member 'poly_y'
# Member 'poly_z'
# Member 'poly_yaw'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_TrajectoryPolynomialPiece(type):
    """Metaclass of message 'TrajectoryPolynomialPiece'."""

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
                'crazyflie_interfaces.msg.TrajectoryPolynomialPiece')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__trajectory_polynomial_piece
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__trajectory_polynomial_piece
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__trajectory_polynomial_piece
            cls._TYPE_SUPPORT = module.type_support_msg__msg__trajectory_polynomial_piece
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__trajectory_polynomial_piece

            from builtin_interfaces.msg import Duration
            if Duration.__class__._TYPE_SUPPORT is None:
                Duration.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class TrajectoryPolynomialPiece(metaclass=Metaclass_TrajectoryPolynomialPiece):
    """Message class 'TrajectoryPolynomialPiece'."""

    __slots__ = [
        '_poly_x',
        '_poly_y',
        '_poly_z',
        '_poly_yaw',
        '_duration',
    ]

    _fields_and_field_types = {
        'poly_x': 'sequence<float>',
        'poly_y': 'sequence<float>',
        'poly_z': 'sequence<float>',
        'poly_yaw': 'sequence<float>',
        'duration': 'builtin_interfaces/Duration',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Duration'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.poly_x = array.array('f', kwargs.get('poly_x', []))
        self.poly_y = array.array('f', kwargs.get('poly_y', []))
        self.poly_z = array.array('f', kwargs.get('poly_z', []))
        self.poly_yaw = array.array('f', kwargs.get('poly_yaw', []))
        from builtin_interfaces.msg import Duration
        self.duration = kwargs.get('duration', Duration())

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
        if self.poly_x != other.poly_x:
            return False
        if self.poly_y != other.poly_y:
            return False
        if self.poly_z != other.poly_z:
            return False
        if self.poly_yaw != other.poly_yaw:
            return False
        if self.duration != other.duration:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def poly_x(self):
        """Message field 'poly_x'."""
        return self._poly_x

    @poly_x.setter
    def poly_x(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'poly_x' array.array() must have the type code of 'f'"
            self._poly_x = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'poly_x' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._poly_x = array.array('f', value)

    @builtins.property
    def poly_y(self):
        """Message field 'poly_y'."""
        return self._poly_y

    @poly_y.setter
    def poly_y(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'poly_y' array.array() must have the type code of 'f'"
            self._poly_y = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'poly_y' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._poly_y = array.array('f', value)

    @builtins.property
    def poly_z(self):
        """Message field 'poly_z'."""
        return self._poly_z

    @poly_z.setter
    def poly_z(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'poly_z' array.array() must have the type code of 'f'"
            self._poly_z = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'poly_z' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._poly_z = array.array('f', value)

    @builtins.property
    def poly_yaw(self):
        """Message field 'poly_yaw'."""
        return self._poly_yaw

    @poly_yaw.setter
    def poly_yaw(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'poly_yaw' array.array() must have the type code of 'f'"
            self._poly_yaw = value
            return
        if __debug__:
            from collections.abc import Sequence
            from collections.abc import Set
            from collections import UserList
            from collections import UserString
            assert \
                ((isinstance(value, Sequence) or
                  isinstance(value, Set) or
                  isinstance(value, UserList)) and
                 not isinstance(value, str) and
                 not isinstance(value, UserString) and
                 all(isinstance(v, float) for v in value) and
                 all(not (val < -3.402823466e+38 or val > 3.402823466e+38) or math.isinf(val) for val in value)), \
                "The 'poly_yaw' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._poly_yaw = array.array('f', value)

    @builtins.property
    def duration(self):
        """Message field 'duration'."""
        return self._duration

    @duration.setter
    def duration(self, value):
        if __debug__:
            from builtin_interfaces.msg import Duration
            assert \
                isinstance(value, Duration), \
                "The 'duration' field must be a sub message of type 'Duration'"
        self._duration = value
