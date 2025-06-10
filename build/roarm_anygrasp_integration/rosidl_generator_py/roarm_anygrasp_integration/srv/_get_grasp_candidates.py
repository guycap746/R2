# generated from rosidl_generator_py/resource/_idl.py.em
# with input from roarm_anygrasp_integration:srv/GetGraspCandidates.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_GetGraspCandidates_Request(type):
    """Metaclass of message 'GetGraspCandidates_Request'."""

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
                'roarm_anygrasp_integration.srv.GetGraspCandidates_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_grasp_candidates__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_grasp_candidates__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_grasp_candidates__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_grasp_candidates__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_grasp_candidates__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class GetGraspCandidates_Request(metaclass=Metaclass_GetGraspCandidates_Request):
    """Message class 'GetGraspCandidates_Request'."""

    __slots__ = [
        '_num_candidates',
        '_min_confidence',
    ]

    _fields_and_field_types = {
        'num_candidates': 'int32',
        'min_confidence': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.num_candidates = kwargs.get('num_candidates', int())
        self.min_confidence = kwargs.get('min_confidence', float())

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
        if self.num_candidates != other.num_candidates:
            return False
        if self.min_confidence != other.min_confidence:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def num_candidates(self):
        """Message field 'num_candidates'."""
        return self._num_candidates

    @num_candidates.setter
    def num_candidates(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'num_candidates' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'num_candidates' field must be an integer in [-2147483648, 2147483647]"
        self._num_candidates = value

    @builtins.property
    def min_confidence(self):
        """Message field 'min_confidence'."""
        return self._min_confidence

    @min_confidence.setter
    def min_confidence(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_confidence' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_confidence' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_confidence = value


# Import statements for member types

# Member 'confidence_scores'
# Member 'grasp_widths'
# Member 'quality_scores'
# Member 'original_indices'
import array  # noqa: E402, I100

# already imported above
# import builtins

# already imported above
# import math

# already imported above
# import rosidl_parser.definition


class Metaclass_GetGraspCandidates_Response(type):
    """Metaclass of message 'GetGraspCandidates_Response'."""

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
                'roarm_anygrasp_integration.srv.GetGraspCandidates_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__get_grasp_candidates__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__get_grasp_candidates__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__get_grasp_candidates__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__get_grasp_candidates__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__get_grasp_candidates__response

            from builtin_interfaces.msg import Time
            if Time.__class__._TYPE_SUPPORT is None:
                Time.__class__.__import_type_support__()

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


class GetGraspCandidates_Response(metaclass=Metaclass_GetGraspCandidates_Response):
    """Message class 'GetGraspCandidates_Response'."""

    __slots__ = [
        '_grasp_poses',
        '_confidence_scores',
        '_grasp_widths',
        '_quality_scores',
        '_original_indices',
        '_detection_status',
        '_total_grasps_detected',
        '_detection_timestamp',
    ]

    _fields_and_field_types = {
        'grasp_poses': 'sequence<geometry_msgs/PoseStamped>',
        'confidence_scores': 'sequence<float>',
        'grasp_widths': 'sequence<float>',
        'quality_scores': 'sequence<float>',
        'original_indices': 'sequence<int32>',
        'detection_status': 'string',
        'total_grasps_detected': 'int32',
        'detection_timestamp': 'builtin_interfaces/Time',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseStamped')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('int32')),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['builtin_interfaces', 'msg'], 'Time'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.grasp_poses = kwargs.get('grasp_poses', [])
        self.confidence_scores = array.array('f', kwargs.get('confidence_scores', []))
        self.grasp_widths = array.array('f', kwargs.get('grasp_widths', []))
        self.quality_scores = array.array('f', kwargs.get('quality_scores', []))
        self.original_indices = array.array('i', kwargs.get('original_indices', []))
        self.detection_status = kwargs.get('detection_status', str())
        self.total_grasps_detected = kwargs.get('total_grasps_detected', int())
        from builtin_interfaces.msg import Time
        self.detection_timestamp = kwargs.get('detection_timestamp', Time())

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
        if self.grasp_poses != other.grasp_poses:
            return False
        if self.confidence_scores != other.confidence_scores:
            return False
        if self.grasp_widths != other.grasp_widths:
            return False
        if self.quality_scores != other.quality_scores:
            return False
        if self.original_indices != other.original_indices:
            return False
        if self.detection_status != other.detection_status:
            return False
        if self.total_grasps_detected != other.total_grasps_detected:
            return False
        if self.detection_timestamp != other.detection_timestamp:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def grasp_poses(self):
        """Message field 'grasp_poses'."""
        return self._grasp_poses

    @grasp_poses.setter
    def grasp_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseStamped
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
                 all(isinstance(v, PoseStamped) for v in value) and
                 True), \
                "The 'grasp_poses' field must be a set or sequence and each value of type 'PoseStamped'"
        self._grasp_poses = value

    @builtins.property
    def confidence_scores(self):
        """Message field 'confidence_scores'."""
        return self._confidence_scores

    @confidence_scores.setter
    def confidence_scores(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'confidence_scores' array.array() must have the type code of 'f'"
            self._confidence_scores = value
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
                "The 'confidence_scores' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._confidence_scores = array.array('f', value)

    @builtins.property
    def grasp_widths(self):
        """Message field 'grasp_widths'."""
        return self._grasp_widths

    @grasp_widths.setter
    def grasp_widths(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'grasp_widths' array.array() must have the type code of 'f'"
            self._grasp_widths = value
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
                "The 'grasp_widths' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._grasp_widths = array.array('f', value)

    @builtins.property
    def quality_scores(self):
        """Message field 'quality_scores'."""
        return self._quality_scores

    @quality_scores.setter
    def quality_scores(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'f', \
                "The 'quality_scores' array.array() must have the type code of 'f'"
            self._quality_scores = value
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
                "The 'quality_scores' field must be a set or sequence and each value of type 'float' and each float in [-340282346600000016151267322115014000640.000000, 340282346600000016151267322115014000640.000000]"
        self._quality_scores = array.array('f', value)

    @builtins.property
    def original_indices(self):
        """Message field 'original_indices'."""
        return self._original_indices

    @original_indices.setter
    def original_indices(self, value):
        if isinstance(value, array.array):
            assert value.typecode == 'i', \
                "The 'original_indices' array.array() must have the type code of 'i'"
            self._original_indices = value
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
                 all(isinstance(v, int) for v in value) and
                 all(val >= -2147483648 and val < 2147483648 for val in value)), \
                "The 'original_indices' field must be a set or sequence and each value of type 'int' and each integer in [-2147483648, 2147483647]"
        self._original_indices = array.array('i', value)

    @builtins.property
    def detection_status(self):
        """Message field 'detection_status'."""
        return self._detection_status

    @detection_status.setter
    def detection_status(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'detection_status' field must be of type 'str'"
        self._detection_status = value

    @builtins.property
    def total_grasps_detected(self):
        """Message field 'total_grasps_detected'."""
        return self._total_grasps_detected

    @total_grasps_detected.setter
    def total_grasps_detected(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'total_grasps_detected' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'total_grasps_detected' field must be an integer in [-2147483648, 2147483647]"
        self._total_grasps_detected = value

    @builtins.property
    def detection_timestamp(self):
        """Message field 'detection_timestamp'."""
        return self._detection_timestamp

    @detection_timestamp.setter
    def detection_timestamp(self, value):
        if __debug__:
            from builtin_interfaces.msg import Time
            assert \
                isinstance(value, Time), \
                "The 'detection_timestamp' field must be a sub message of type 'Time'"
        self._detection_timestamp = value


class Metaclass_GetGraspCandidates(type):
    """Metaclass of service 'GetGraspCandidates'."""

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
                'roarm_anygrasp_integration.srv.GetGraspCandidates')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__get_grasp_candidates

            from roarm_anygrasp_integration.srv import _get_grasp_candidates
            if _get_grasp_candidates.Metaclass_GetGraspCandidates_Request._TYPE_SUPPORT is None:
                _get_grasp_candidates.Metaclass_GetGraspCandidates_Request.__import_type_support__()
            if _get_grasp_candidates.Metaclass_GetGraspCandidates_Response._TYPE_SUPPORT is None:
                _get_grasp_candidates.Metaclass_GetGraspCandidates_Response.__import_type_support__()


class GetGraspCandidates(metaclass=Metaclass_GetGraspCandidates):
    from roarm_anygrasp_integration.srv._get_grasp_candidates import GetGraspCandidates_Request as Request
    from roarm_anygrasp_integration.srv._get_grasp_candidates import GetGraspCandidates_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
