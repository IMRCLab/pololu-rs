# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:srv/UploadTrajectory.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UploadTrajectory_Request(type):
    """Metaclass of message 'UploadTrajectory_Request'."""

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
                'crazyflie_interfaces.srv.UploadTrajectory_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__upload_trajectory__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__upload_trajectory__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__upload_trajectory__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__upload_trajectory__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__upload_trajectory__request

            from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
            if TrajectoryPolynomialPiece.__class__._TYPE_SUPPORT is None:
                TrajectoryPolynomialPiece.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UploadTrajectory_Request(metaclass=Metaclass_UploadTrajectory_Request):
    """Message class 'UploadTrajectory_Request'."""

    __slots__ = [
        '_trajectory_id',
        '_piece_offset',
        '_pieces',
    ]

    _fields_and_field_types = {
        'trajectory_id': 'uint8',
        'piece_offset': 'uint32',
        'pieces': 'sequence<crazyflie_interfaces/TrajectoryPolynomialPiece>',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['crazyflie_interfaces', 'msg'], 'TrajectoryPolynomialPiece')),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.trajectory_id = kwargs.get('trajectory_id', int())
        self.piece_offset = kwargs.get('piece_offset', int())
        self.pieces = kwargs.get('pieces', [])

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
        if self.trajectory_id != other.trajectory_id:
            return False
        if self.piece_offset != other.piece_offset:
            return False
        if self.pieces != other.pieces:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

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
    def piece_offset(self):
        """Message field 'piece_offset'."""
        return self._piece_offset

    @piece_offset.setter
    def piece_offset(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'piece_offset' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'piece_offset' field must be an unsigned integer in [0, 4294967295]"
        self._piece_offset = value

    @builtins.property
    def pieces(self):
        """Message field 'pieces'."""
        return self._pieces

    @pieces.setter
    def pieces(self, value):
        if __debug__:
            from crazyflie_interfaces.msg import TrajectoryPolynomialPiece
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
                 all(isinstance(v, TrajectoryPolynomialPiece) for v in value) and
                 True), \
                "The 'pieces' field must be a set or sequence and each value of type 'TrajectoryPolynomialPiece'"
        self._pieces = value


# Import statements for member types

# already imported above
# import rosidl_parser.definition


class Metaclass_UploadTrajectory_Response(type):
    """Metaclass of message 'UploadTrajectory_Response'."""

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
                'crazyflie_interfaces.srv.UploadTrajectory_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__upload_trajectory__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__upload_trajectory__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__upload_trajectory__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__upload_trajectory__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__upload_trajectory__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UploadTrajectory_Response(metaclass=Metaclass_UploadTrajectory_Response):
    """Message class 'UploadTrajectory_Response'."""

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


class Metaclass_UploadTrajectory(type):
    """Metaclass of service 'UploadTrajectory'."""

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
                'crazyflie_interfaces.srv.UploadTrajectory')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__upload_trajectory

            from crazyflie_interfaces.srv import _upload_trajectory
            if _upload_trajectory.Metaclass_UploadTrajectory_Request._TYPE_SUPPORT is None:
                _upload_trajectory.Metaclass_UploadTrajectory_Request.__import_type_support__()
            if _upload_trajectory.Metaclass_UploadTrajectory_Response._TYPE_SUPPORT is None:
                _upload_trajectory.Metaclass_UploadTrajectory_Response.__import_type_support__()


class UploadTrajectory(metaclass=Metaclass_UploadTrajectory):
    from crazyflie_interfaces.srv._upload_trajectory import UploadTrajectory_Request as Request
    from crazyflie_interfaces.srv._upload_trajectory import UploadTrajectory_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
