# generated from rosidl_generator_py/resource/_idl.py.em
# with input from crazyflie_interfaces:msg/Status.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Status(type):
    """Metaclass of message 'Status'."""

    _CREATE_ROS_MESSAGE = None
    _CONVERT_FROM_PY = None
    _CONVERT_TO_PY = None
    _DESTROY_ROS_MESSAGE = None
    _TYPE_SUPPORT = None

    __constants = {
        'SUPERVISOR_INFO_CAN_BE_ARMED': 1,
        'SUPERVISOR_INFO_IS_ARMED': 2,
        'SUPERVISOR_INFO_AUTO_ARM': 4,
        'SUPERVISOR_INFO_CAN_FLY': 8,
        'SUPERVISOR_INFO_IS_FLYING': 16,
        'SUPERVISOR_INFO_IS_TUMBLED': 32,
        'SUPERVISOR_INFO_IS_LOCKED': 64,
        'PM_STATE_BATTERY': 0,
        'PM_STATE_CHARGING': 1,
        'PM_STATE_CHARGED': 2,
        'PM_STATE_LOW_POWER': 3,
        'PM_STATE_SHUTDOWN': 4,
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
                'crazyflie_interfaces.msg.Status')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__status
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__status
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__status
            cls._TYPE_SUPPORT = module.type_support_msg__msg__status
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__status

            from std_msgs.msg import Header
            if Header.__class__._TYPE_SUPPORT is None:
                Header.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
            'SUPERVISOR_INFO_CAN_BE_ARMED': cls.__constants['SUPERVISOR_INFO_CAN_BE_ARMED'],
            'SUPERVISOR_INFO_IS_ARMED': cls.__constants['SUPERVISOR_INFO_IS_ARMED'],
            'SUPERVISOR_INFO_AUTO_ARM': cls.__constants['SUPERVISOR_INFO_AUTO_ARM'],
            'SUPERVISOR_INFO_CAN_FLY': cls.__constants['SUPERVISOR_INFO_CAN_FLY'],
            'SUPERVISOR_INFO_IS_FLYING': cls.__constants['SUPERVISOR_INFO_IS_FLYING'],
            'SUPERVISOR_INFO_IS_TUMBLED': cls.__constants['SUPERVISOR_INFO_IS_TUMBLED'],
            'SUPERVISOR_INFO_IS_LOCKED': cls.__constants['SUPERVISOR_INFO_IS_LOCKED'],
            'PM_STATE_BATTERY': cls.__constants['PM_STATE_BATTERY'],
            'PM_STATE_CHARGING': cls.__constants['PM_STATE_CHARGING'],
            'PM_STATE_CHARGED': cls.__constants['PM_STATE_CHARGED'],
            'PM_STATE_LOW_POWER': cls.__constants['PM_STATE_LOW_POWER'],
            'PM_STATE_SHUTDOWN': cls.__constants['PM_STATE_SHUTDOWN'],
        }

    @property
    def SUPERVISOR_INFO_CAN_BE_ARMED(self):
        """Message constant 'SUPERVISOR_INFO_CAN_BE_ARMED'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_CAN_BE_ARMED']

    @property
    def SUPERVISOR_INFO_IS_ARMED(self):
        """Message constant 'SUPERVISOR_INFO_IS_ARMED'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_IS_ARMED']

    @property
    def SUPERVISOR_INFO_AUTO_ARM(self):
        """Message constant 'SUPERVISOR_INFO_AUTO_ARM'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_AUTO_ARM']

    @property
    def SUPERVISOR_INFO_CAN_FLY(self):
        """Message constant 'SUPERVISOR_INFO_CAN_FLY'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_CAN_FLY']

    @property
    def SUPERVISOR_INFO_IS_FLYING(self):
        """Message constant 'SUPERVISOR_INFO_IS_FLYING'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_IS_FLYING']

    @property
    def SUPERVISOR_INFO_IS_TUMBLED(self):
        """Message constant 'SUPERVISOR_INFO_IS_TUMBLED'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_IS_TUMBLED']

    @property
    def SUPERVISOR_INFO_IS_LOCKED(self):
        """Message constant 'SUPERVISOR_INFO_IS_LOCKED'."""
        return Metaclass_Status.__constants['SUPERVISOR_INFO_IS_LOCKED']

    @property
    def PM_STATE_BATTERY(self):
        """Message constant 'PM_STATE_BATTERY'."""
        return Metaclass_Status.__constants['PM_STATE_BATTERY']

    @property
    def PM_STATE_CHARGING(self):
        """Message constant 'PM_STATE_CHARGING'."""
        return Metaclass_Status.__constants['PM_STATE_CHARGING']

    @property
    def PM_STATE_CHARGED(self):
        """Message constant 'PM_STATE_CHARGED'."""
        return Metaclass_Status.__constants['PM_STATE_CHARGED']

    @property
    def PM_STATE_LOW_POWER(self):
        """Message constant 'PM_STATE_LOW_POWER'."""
        return Metaclass_Status.__constants['PM_STATE_LOW_POWER']

    @property
    def PM_STATE_SHUTDOWN(self):
        """Message constant 'PM_STATE_SHUTDOWN'."""
        return Metaclass_Status.__constants['PM_STATE_SHUTDOWN']


class Status(metaclass=Metaclass_Status):
    """
    Message class 'Status'.

    Constants:
      SUPERVISOR_INFO_CAN_BE_ARMED
      SUPERVISOR_INFO_IS_ARMED
      SUPERVISOR_INFO_AUTO_ARM
      SUPERVISOR_INFO_CAN_FLY
      SUPERVISOR_INFO_IS_FLYING
      SUPERVISOR_INFO_IS_TUMBLED
      SUPERVISOR_INFO_IS_LOCKED
      PM_STATE_BATTERY
      PM_STATE_CHARGING
      PM_STATE_CHARGED
      PM_STATE_LOW_POWER
      PM_STATE_SHUTDOWN
    """

    __slots__ = [
        '_header',
        '_supervisor_info',
        '_battery_voltage',
        '_pm_state',
        '_rssi',
        '_num_rx_broadcast',
        '_num_tx_broadcast',
        '_num_rx_unicast',
        '_num_tx_unicast',
        '_latency_unicast',
    ]

    _fields_and_field_types = {
        'header': 'std_msgs/Header',
        'supervisor_info': 'uint16',
        'battery_voltage': 'float',
        'pm_state': 'uint8',
        'rssi': 'uint8',
        'num_rx_broadcast': 'uint32',
        'num_tx_broadcast': 'uint32',
        'num_rx_unicast': 'uint32',
        'num_tx_unicast': 'uint32',
        'latency_unicast': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['std_msgs', 'msg'], 'Header'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint8'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from std_msgs.msg import Header
        self.header = kwargs.get('header', Header())
        self.supervisor_info = kwargs.get('supervisor_info', int())
        self.battery_voltage = kwargs.get('battery_voltage', float())
        self.pm_state = kwargs.get('pm_state', int())
        self.rssi = kwargs.get('rssi', int())
        self.num_rx_broadcast = kwargs.get('num_rx_broadcast', int())
        self.num_tx_broadcast = kwargs.get('num_tx_broadcast', int())
        self.num_rx_unicast = kwargs.get('num_rx_unicast', int())
        self.num_tx_unicast = kwargs.get('num_tx_unicast', int())
        self.latency_unicast = kwargs.get('latency_unicast', int())

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
        if self.header != other.header:
            return False
        if self.supervisor_info != other.supervisor_info:
            return False
        if self.battery_voltage != other.battery_voltage:
            return False
        if self.pm_state != other.pm_state:
            return False
        if self.rssi != other.rssi:
            return False
        if self.num_rx_broadcast != other.num_rx_broadcast:
            return False
        if self.num_tx_broadcast != other.num_tx_broadcast:
            return False
        if self.num_rx_unicast != other.num_rx_unicast:
            return False
        if self.num_tx_unicast != other.num_tx_unicast:
            return False
        if self.latency_unicast != other.latency_unicast:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def header(self):
        """Message field 'header'."""
        return self._header

    @header.setter
    def header(self, value):
        if __debug__:
            from std_msgs.msg import Header
            assert \
                isinstance(value, Header), \
                "The 'header' field must be a sub message of type 'Header'"
        self._header = value

    @builtins.property
    def supervisor_info(self):
        """Message field 'supervisor_info'."""
        return self._supervisor_info

    @supervisor_info.setter
    def supervisor_info(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'supervisor_info' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'supervisor_info' field must be an unsigned integer in [0, 65535]"
        self._supervisor_info = value

    @builtins.property
    def battery_voltage(self):
        """Message field 'battery_voltage'."""
        return self._battery_voltage

    @battery_voltage.setter
    def battery_voltage(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'battery_voltage' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'battery_voltage' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._battery_voltage = value

    @builtins.property
    def pm_state(self):
        """Message field 'pm_state'."""
        return self._pm_state

    @pm_state.setter
    def pm_state(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'pm_state' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'pm_state' field must be an unsigned integer in [0, 255]"
        self._pm_state = value

    @builtins.property
    def rssi(self):
        """Message field 'rssi'."""
        return self._rssi

    @rssi.setter
    def rssi(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rssi' field must be of type 'int'"
            assert value >= 0 and value < 256, \
                "The 'rssi' field must be an unsigned integer in [0, 255]"
        self._rssi = value

    @builtins.property
    def num_rx_broadcast(self):
        """Message field 'num_rx_broadcast'."""
        return self._num_rx_broadcast

    @num_rx_broadcast.setter
    def num_rx_broadcast(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_rx_broadcast' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_rx_broadcast' field must be an unsigned integer in [0, 4294967295]"
        self._num_rx_broadcast = value

    @builtins.property
    def num_tx_broadcast(self):
        """Message field 'num_tx_broadcast'."""
        return self._num_tx_broadcast

    @num_tx_broadcast.setter
    def num_tx_broadcast(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_tx_broadcast' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_tx_broadcast' field must be an unsigned integer in [0, 4294967295]"
        self._num_tx_broadcast = value

    @builtins.property
    def num_rx_unicast(self):
        """Message field 'num_rx_unicast'."""
        return self._num_rx_unicast

    @num_rx_unicast.setter
    def num_rx_unicast(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_rx_unicast' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_rx_unicast' field must be an unsigned integer in [0, 4294967295]"
        self._num_rx_unicast = value

    @builtins.property
    def num_tx_unicast(self):
        """Message field 'num_tx_unicast'."""
        return self._num_tx_unicast

    @num_tx_unicast.setter
    def num_tx_unicast(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_tx_unicast' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'num_tx_unicast' field must be an unsigned integer in [0, 4294967295]"
        self._num_tx_unicast = value

    @builtins.property
    def latency_unicast(self):
        """Message field 'latency_unicast'."""
        return self._latency_unicast

    @latency_unicast.setter
    def latency_unicast(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'latency_unicast' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'latency_unicast' field must be an unsigned integer in [0, 65535]"
        self._latency_unicast = value
