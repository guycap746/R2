# generated from rosidl_generator_py/resource/_idl.py.em
# with input from roarm_anygrasp_integration:srv/SelectGrasp.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_SelectGrasp_Request(type):
    """Metaclass of message 'SelectGrasp_Request'."""

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
                'roarm_anygrasp_integration.srv.SelectGrasp_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__select_grasp__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__select_grasp__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__select_grasp__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__select_grasp__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__select_grasp__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SelectGrasp_Request(metaclass=Metaclass_SelectGrasp_Request):
    """Message class 'SelectGrasp_Request'."""

    __slots__ = [
        '_selected_grasp_index',
        '_show_more_candidates',
    ]

    _fields_and_field_types = {
        'selected_grasp_index': 'int32',
        'show_more_candidates': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.selected_grasp_index = kwargs.get('selected_grasp_index', int())
        self.show_more_candidates = kwargs.get('show_more_candidates', bool())

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
        if self.selected_grasp_index != other.selected_grasp_index:
            return False
        if self.show_more_candidates != other.show_more_candidates:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def selected_grasp_index(self):
        """Message field 'selected_grasp_index'."""
        return self._selected_grasp_index

    @selected_grasp_index.setter
    def selected_grasp_index(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'selected_grasp_index' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'selected_grasp_index' field must be an integer in [-2147483648, 2147483647]"
        self._selected_grasp_index = value

    @builtins.property
    def show_more_candidates(self):
        """Message field 'show_more_candidates'."""
        return self._show_more_candidates

    @show_more_candidates.setter
    def show_more_candidates(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'show_more_candidates' field must be of type 'bool'"
        self._show_more_candidates = value


# Import statements for member types

# already imported above
# import builtins

import math  # noqa: E402, I100

# already imported above
# import rosidl_parser.definition


class Metaclass_SelectGrasp_Response(type):
    """Metaclass of message 'SelectGrasp_Response'."""

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
                'roarm_anygrasp_integration.srv.SelectGrasp_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__select_grasp__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__select_grasp__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__select_grasp__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__select_grasp__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__select_grasp__response

            from geometry_msgs.msg import PoseStamped
            if PoseStamped.__class__._TYPE_SUPPORT is None:
                PoseStamped.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class SelectGrasp_Response(metaclass=Metaclass_SelectGrasp_Response):
    """Message class 'SelectGrasp_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_selected_pose',
        '_confidence_score',
        '_grasp_width',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'selected_pose': 'geometry_msgs/PoseStamped',
        'confidence_score': 'float',
        'grasp_width': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        from geometry_msgs.msg import PoseStamped
        self.selected_pose = kwargs.get('selected_pose', PoseStamped())
        self.confidence_score = kwargs.get('confidence_score', float())
        self.grasp_width = kwargs.get('grasp_width', float())

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
        if self.selected_pose != other.selected_pose:
            return False
        if self.confidence_score != other.confidence_score:
            return False
        if self.grasp_width != other.grasp_width:
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
    def selected_pose(self):
        """Message field 'selected_pose'."""
        return self._selected_pose

    @selected_pose.setter
    def selected_pose(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
            assert \
                isinstance(value, PoseStamped), \
                "The 'selected_pose' field must be a sub message of type 'PoseStamped'"
        self._selected_pose = value

    @builtins.property
    def confidence_score(self):
        """Message field 'confidence_score'."""
        return self._confidence_score

    @confidence_score.setter
    def confidence_score(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'confidence_score' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'confidence_score' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._confidence_score = value

    @builtins.property
    def grasp_width(self):
        """Message field 'grasp_width'."""
        return self._grasp_width

    @grasp_width.setter
    def grasp_width(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'grasp_width' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'grasp_width' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._grasp_width = value


class Metaclass_SelectGrasp(type):
    """Metaclass of service 'SelectGrasp'."""

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
                'roarm_anygrasp_integration.srv.SelectGrasp')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__select_grasp

            from roarm_anygrasp_integration.srv import _select_grasp
            if _select_grasp.Metaclass_SelectGrasp_Request._TYPE_SUPPORT is None:
                _select_grasp.Metaclass_SelectGrasp_Request.__import_type_support__()
            if _select_grasp.Metaclass_SelectGrasp_Response._TYPE_SUPPORT is None:
                _select_grasp.Metaclass_SelectGrasp_Response.__import_type_support__()


class SelectGrasp(metaclass=Metaclass_SelectGrasp):
    from roarm_anygrasp_integration.srv._select_grasp import SelectGrasp_Request as Request
    from roarm_anygrasp_integration.srv._select_grasp import SelectGrasp_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
