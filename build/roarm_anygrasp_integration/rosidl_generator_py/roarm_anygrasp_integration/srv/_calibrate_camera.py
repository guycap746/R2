# generated from rosidl_generator_py/resource/_idl.py.em
# with input from roarm_anygrasp_integration:srv/CalibrateCamera.idl
# generated code does not contain a copyright notice


# Import statements for member types

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_CalibrateCamera_Request(type):
    """Metaclass of message 'CalibrateCamera_Request'."""

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
            module = import_type_support('roarm_anygrasp_integration')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'roarm_anygrasp_integration.srv.CalibrateCamera_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__calibrate_camera__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__calibrate_camera__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__calibrate_camera__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__calibrate_camera__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__calibrate_camera__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CalibrateCamera_Request(metaclass=Metaclass_CalibrateCamera_Request):
    """Message class 'CalibrateCamera_Request'."""

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


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_CalibrateCamera_Response(type):
    """Metaclass of message 'CalibrateCamera_Response'."""

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
            module = import_type_support('roarm_anygrasp_integration')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'roarm_anygrasp_integration.srv.CalibrateCamera_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__calibrate_camera__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__calibrate_camera__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__calibrate_camera__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__calibrate_camera__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__calibrate_camera__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class CalibrateCamera_Response(metaclass=Metaclass_CalibrateCamera_Response):
    """Message class 'CalibrateCamera_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_primary_reprojection_error',
        '_side_reprojection_error',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'primary_reprojection_error': 'double',
        'side_reprojection_error': 'double',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
        rosidl_parser.definition.BasicType('double'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.primary_reprojection_error = kwargs.get('primary_reprojection_error', float())
        self.side_reprojection_error = kwargs.get('side_reprojection_error', float())

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
        if self.success != other.success:
            return False
        if self.message != other.message:
            return False
        if self.primary_reprojection_error != other.primary_reprojection_error:
            return False
        if self.side_reprojection_error != other.side_reprojection_error:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def success(self):
        """Message field 'success'."""
        return self._success

    @success.setter
    def success(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'success' field must be of type 'bool'"
        self._success = value

    @builtins.property
    def message(self):
        """Message field 'message'."""
        return self._message

    @message.setter
    def message(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'message' field must be of type 'str'"
        self._message = value

    @builtins.property
    def primary_reprojection_error(self):
        """Message field 'primary_reprojection_error'."""
        return self._primary_reprojection_error

    @primary_reprojection_error.setter
    def primary_reprojection_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'primary_reprojection_error' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'primary_reprojection_error' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._primary_reprojection_error = value

    @builtins.property
    def side_reprojection_error(self):
        """Message field 'side_reprojection_error'."""
        return self._side_reprojection_error

    @side_reprojection_error.setter
    def side_reprojection_error(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'side_reprojection_error' field must be of type 'float'"
            assert not (value < -1.7976931348623157e+308 or value > 1.7976931348623157e+308) or math.isinf(value), \
                "The 'side_reprojection_error' field must be a double in [-1.7976931348623157e+308, 1.7976931348623157e+308]"
        self._side_reprojection_error = value


class Metaclass_CalibrateCamera(type):
    """Metaclass of service 'CalibrateCamera'."""

    _TYPE_SUPPORT = None

    @classmethod
    def __import_type_support__(cls):
        try:
            from rosidl_generator_py import import_type_support
            module = import_type_support('roarm_anygrasp_integration')
        except ImportError:
            import logging
            import traceback
            logger = logging.getLogger(
                'roarm_anygrasp_integration.srv.CalibrateCamera')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__calibrate_camera

            from roarm_anygrasp_integration.srv import _calibrate_camera
            if _calibrate_camera.Metaclass_CalibrateCamera_Request._TYPE_SUPPORT is None:
                _calibrate_camera.Metaclass_CalibrateCamera_Request.__import_type_support__()
            if _calibrate_camera.Metaclass_CalibrateCamera_Response._TYPE_SUPPORT is None:
                _calibrate_camera.Metaclass_CalibrateCamera_Response.__import_type_support__()


class CalibrateCamera(metaclass=Metaclass_CalibrateCamera):
    from roarm_anygrasp_integration.srv._calibrate_camera import CalibrateCamera_Request as Request
    from roarm_anygrasp_integration.srv._calibrate_camera import CalibrateCamera_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
