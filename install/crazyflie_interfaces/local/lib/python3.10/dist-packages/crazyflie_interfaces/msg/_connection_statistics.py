# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:msg/ConnectionStatistics.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ConnectionStatistics(type):
    """Metaclass of message 'ConnectionStatistics'."""

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
                'crazyflie_interfaces.msg.ConnectionStatistics')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__connection_statistics
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__connection_statistics
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__connection_statistics
            cls._TYPE_SUPPORT = module.type_support_msg__msg__connection_statistics
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__connection_statistics

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ConnectionStatistics(metaclass=Metaclass_ConnectionStatistics):
    """Message class 'ConnectionStatistics'."""

    __slots__ = [
        '_uri',
        '_sent_count',
        '_sent_ping_count',
        '_receive_count',
        '_enqueued_count',
        '_ack_count',
    ]

    _fields_and_field_types = {
        'uri': 'string',
        'sent_count': 'uint64',
        'sent_ping_count': 'uint64',
        'receive_count': 'uint64',
        'enqueued_count': 'uint64',
        'ack_count': 'uint64',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint64'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.uri = kwargs.get('uri', str())
        self.sent_count = kwargs.get('sent_count', int())
        self.sent_ping_count = kwargs.get('sent_ping_count', int())
        self.receive_count = kwargs.get('receive_count', int())
        self.enqueued_count = kwargs.get('enqueued_count', int())
        self.ack_count = kwargs.get('ack_count', int())

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
        if self.uri != other.uri:
            return False
        if self.sent_count != other.sent_count:
            return False
        if self.sent_ping_count != other.sent_ping_count:
            return False
        if self.receive_count != other.receive_count:
            return False
        if self.enqueued_count != other.enqueued_count:
            return False
        if self.ack_count != other.ack_count:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def uri(self):
        """Message field 'uri'."""
        return self._uri

    @uri.setter
    def uri(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'uri' field must be of type 'str'"
        self._uri = value

    @builtins.property
    def sent_count(self):
        """Message field 'sent_count'."""
        return self._sent_count

    @sent_count.setter
    def sent_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sent_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'sent_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._sent_count = value

    @builtins.property
    def sent_ping_count(self):
        """Message field 'sent_ping_count'."""
        return self._sent_ping_count

    @sent_ping_count.setter
    def sent_ping_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sent_ping_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'sent_ping_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._sent_ping_count = value

    @builtins.property
    def receive_count(self):
        """Message field 'receive_count'."""
        return self._receive_count

    @receive_count.setter
    def receive_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'receive_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'receive_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._receive_count = value

    @builtins.property
    def enqueued_count(self):
        """Message field 'enqueued_count'."""
        return self._enqueued_count

    @enqueued_count.setter
    def enqueued_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'enqueued_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'enqueued_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._enqueued_count = value

    @builtins.property
    def ack_count(self):
        """Message field 'ack_count'."""
        return self._ack_count

    @ack_count.setter
    def ack_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'ack_count' field must be of type 'int'"
            assert value >= 0 and value < 18446744073709551616, \
                "The 'ack_count' field must be an unsigned integer in [0, 18446744073709551615]"
        self._ack_count = value
