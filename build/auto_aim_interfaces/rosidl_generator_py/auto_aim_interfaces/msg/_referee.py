# generated from rosidl_generator_py/resource/_idl.py.em
# with input from auto_aim_interfaces:msg/Referee.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_Referee(type):
    """Metaclass of message 'Referee'."""

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
            module = import_type_support('auto_aim_interfaces')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'auto_aim_interfaces.msg.Referee')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__msg__referee
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__msg__referee
            cls._CONVERT_TO_PY = module.convert_to_py_msg__msg__referee
            cls._TYPE_SUPPORT = module.type_support_msg__msg__referee
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__msg__referee

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class Referee(metaclass=Metaclass_Referee):
    """Message class 'Referee'."""

    __slots__ = [
        '_event_data',
        '_time',
        '_rfid',
        '_base_hp',
        '_sentry_hp',
        '_outpost_hp',
        '_projectile_allowance_17mm',
    ]

    _fields_and_field_types = {
        'event_data': 'uint32',
        'time': 'uint16',
        'rfid': 'uint32',
        'base_hp': 'uint16',
        'sentry_hp': 'uint16',
        'outpost_hp': 'uint16',
        'projectile_allowance_17mm': 'uint16',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint32'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
        rosidl_parser.definition.BasicType('uint16'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.event_data = kwargs.get('event_data', int())
        self.time = kwargs.get('time', int())
        self.rfid = kwargs.get('rfid', int())
        self.base_hp = kwargs.get('base_hp', int())
        self.sentry_hp = kwargs.get('sentry_hp', int())
        self.outpost_hp = kwargs.get('outpost_hp', int())
        self.projectile_allowance_17mm = kwargs.get('projectile_allowance_17mm', int())

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
        if self.event_data != other.event_data:
            return False
        if self.time != other.time:
            return False
        if self.rfid != other.rfid:
            return False
        if self.base_hp != other.base_hp:
            return False
        if self.sentry_hp != other.sentry_hp:
            return False
        if self.outpost_hp != other.outpost_hp:
            return False
        if self.projectile_allowance_17mm != other.projectile_allowance_17mm:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def event_data(self):
        """Message field 'event_data'."""
        return self._event_data

    @event_data.setter
    def event_data(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'event_data' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'event_data' field must be an unsigned integer in [0, 4294967295]"
        self._event_data = value

    @builtins.property
    def time(self):
        """Message field 'time'."""
        return self._time

    @time.setter
    def time(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'time' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'time' field must be an unsigned integer in [0, 65535]"
        self._time = value

    @builtins.property
    def rfid(self):
        """Message field 'rfid'."""
        return self._rfid

    @rfid.setter
    def rfid(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'rfid' field must be of type 'int'"
            assert value >= 0 and value < 4294967296, \
                "The 'rfid' field must be an unsigned integer in [0, 4294967295]"
        self._rfid = value

    @builtins.property
    def base_hp(self):
        """Message field 'base_hp'."""
        return self._base_hp

    @base_hp.setter
    def base_hp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'base_hp' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'base_hp' field must be an unsigned integer in [0, 65535]"
        self._base_hp = value

    @builtins.property
    def sentry_hp(self):
        """Message field 'sentry_hp'."""
        return self._sentry_hp

    @sentry_hp.setter
    def sentry_hp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'sentry_hp' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'sentry_hp' field must be an unsigned integer in [0, 65535]"
        self._sentry_hp = value

    @builtins.property
    def outpost_hp(self):
        """Message field 'outpost_hp'."""
        return self._outpost_hp

    @outpost_hp.setter
    def outpost_hp(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'outpost_hp' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'outpost_hp' field must be an unsigned integer in [0, 65535]"
        self._outpost_hp = value

    @builtins.property
    def projectile_allowance_17mm(self):
        """Message field 'projectile_allowance_17mm'."""
        return self._projectile_allowance_17mm

    @projectile_allowance_17mm.setter
    def projectile_allowance_17mm(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'projectile_allowance_17mm' field must be of type 'int'"
            assert value >= 0 and value < 65536, \
                "The 'projectile_allowance_17mm' field must be an unsigned integer in [0, 65535]"
        self._projectile_allowance_17mm = value
