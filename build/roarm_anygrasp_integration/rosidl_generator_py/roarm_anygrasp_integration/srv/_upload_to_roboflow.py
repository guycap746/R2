# generated from rosidl_generator_py/resource/_idl.py.em
# with input from roarm_anygrasp_integration:srv/UploadToRoboflow.idl
# generated code does not contain a copyright notice


# Import statements for member types

# Member 'confidence_scores'
import array  # noqa: E402, I100

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_UploadToRoboflow_Request(type):
    """Metaclass of message 'UploadToRoboflow_Request'."""

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
                'roarm_anygrasp_integration.srv.UploadToRoboflow_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__upload_to_roboflow__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__upload_to_roboflow__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__upload_to_roboflow__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__upload_to_roboflow__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__upload_to_roboflow__request

            from geometry_msgs.msg import PoseArray
            if PoseArray.__class__._TYPE_SUPPORT is None:
                PoseArray.__class__.__import_type_support__()

            from sensor_msgs.msg import Image
            if Image.__class__._TYPE_SUPPORT is None:
                Image.__class__.__import_type_support__()

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UploadToRoboflow_Request(metaclass=Metaclass_UploadToRoboflow_Request):
    """Message class 'UploadToRoboflow_Request'."""

    __slots__ = [
        '_rgb_image',
        '_grasp_poses',
        '_confidence_scores',
        '_class_labels',
        '_scene_description',
        '_object_types',
        '_include_annotations',
        '_upload_immediately',
    ]

    _fields_and_field_types = {
        'rgb_image': 'sensor_msgs/Image',
        'grasp_poses': 'geometry_msgs/PoseArray',
        'confidence_scores': 'sequence<float>',
        'class_labels': 'sequence<string>',
        'scene_description': 'string',
        'object_types': 'string',
        'include_annotations': 'boolean',
        'upload_immediately': 'boolean',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.NamespacedType(['sensor_msgs', 'msg'], 'Image'),  # noqa: E501
        rosidl_parser.definition.NamespacedType(['geometry_msgs', 'msg'], 'PoseArray'),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.BasicType('float')),  # noqa: E501
        rosidl_parser.definition.UnboundedSequence(rosidl_parser.definition.UnboundedString()),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        from sensor_msgs.msg import Image
        self.rgb_image = kwargs.get('rgb_image', Image())
        from geometry_msgs.msg import PoseArray
        self.grasp_poses = kwargs.get('grasp_poses', PoseArray())
        self.confidence_scores = array.array('f', kwargs.get('confidence_scores', []))
        self.class_labels = kwargs.get('class_labels', [])
        self.scene_description = kwargs.get('scene_description', str())
        self.object_types = kwargs.get('object_types', str())
        self.include_annotations = kwargs.get('include_annotations', bool())
        self.upload_immediately = kwargs.get('upload_immediately', bool())

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
        if self.rgb_image != other.rgb_image:
            return False
        if self.grasp_poses != other.grasp_poses:
            return False
        if self.confidence_scores != other.confidence_scores:
            return False
        if self.class_labels != other.class_labels:
            return False
        if self.scene_description != other.scene_description:
            return False
        if self.object_types != other.object_types:
            return False
        if self.include_annotations != other.include_annotations:
            return False
        if self.upload_immediately != other.upload_immediately:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def rgb_image(self):
        """Message field 'rgb_image'."""
        return self._rgb_image

    @rgb_image.setter
    def rgb_image(self, value):
        if __debug__:
            from sensor_msgs.msg import Image
            assert \
                isinstance(value, Image), \
                "The 'rgb_image' field must be a sub message of type 'Image'"
        self._rgb_image = value

    @builtins.property
    def grasp_poses(self):
        """Message field 'grasp_poses'."""
        return self._grasp_poses

    @grasp_poses.setter
    def grasp_poses(self, value):
        if __debug__:
            from geometry_msgs.msg import PoseArray
            assert \
                isinstance(value, PoseArray), \
                "The 'grasp_poses' field must be a sub message of type 'PoseArray'"
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
    def class_labels(self):
        """Message field 'class_labels'."""
        return self._class_labels

    @class_labels.setter
    def class_labels(self, value):
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
                 all(isinstance(v, str) for v in value) and
                 True), \
                "The 'class_labels' field must be a set or sequence and each value of type 'str'"
        self._class_labels = value

    @builtins.property
    def scene_description(self):
        """Message field 'scene_description'."""
        return self._scene_description

    @scene_description.setter
    def scene_description(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'scene_description' field must be of type 'str'"
        self._scene_description = value

    @builtins.property
    def object_types(self):
        """Message field 'object_types'."""
        return self._object_types

    @object_types.setter
    def object_types(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'object_types' field must be of type 'str'"
        self._object_types = value

    @builtins.property
    def include_annotations(self):
        """Message field 'include_annotations'."""
        return self._include_annotations

    @include_annotations.setter
    def include_annotations(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'include_annotations' field must be of type 'bool'"
        self._include_annotations = value

    @builtins.property
    def upload_immediately(self):
        """Message field 'upload_immediately'."""
        return self._upload_immediately

    @upload_immediately.setter
    def upload_immediately(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'upload_immediately' field must be of type 'bool'"
        self._upload_immediately = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_UploadToRoboflow_Response(type):
    """Metaclass of message 'UploadToRoboflow_Response'."""

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
                'roarm_anygrasp_integration.srv.UploadToRoboflow_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__upload_to_roboflow__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__upload_to_roboflow__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__upload_to_roboflow__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__upload_to_roboflow__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__upload_to_roboflow__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class UploadToRoboflow_Response(metaclass=Metaclass_UploadToRoboflow_Response):
    """Message class 'UploadToRoboflow_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_roboflow_image_id',
        '_upload_url',
        '_annotation_count',
        '_dataset_version',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'roboflow_image_id': 'string',
        'upload_url': 'string',
        'annotation_count': 'int32',
        'dataset_version': 'string',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.roboflow_image_id = kwargs.get('roboflow_image_id', str())
        self.upload_url = kwargs.get('upload_url', str())
        self.annotation_count = kwargs.get('annotation_count', int())
        self.dataset_version = kwargs.get('dataset_version', str())

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
        if self.roboflow_image_id != other.roboflow_image_id:
            return False
        if self.upload_url != other.upload_url:
            return False
        if self.annotation_count != other.annotation_count:
            return False
        if self.dataset_version != other.dataset_version:
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
    def roboflow_image_id(self):
        """Message field 'roboflow_image_id'."""
        return self._roboflow_image_id

    @roboflow_image_id.setter
    def roboflow_image_id(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'roboflow_image_id' field must be of type 'str'"
        self._roboflow_image_id = value

    @builtins.property
    def upload_url(self):
        """Message field 'upload_url'."""
        return self._upload_url

    @upload_url.setter
    def upload_url(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'upload_url' field must be of type 'str'"
        self._upload_url = value

    @builtins.property
    def annotation_count(self):
        """Message field 'annotation_count'."""
        return self._annotation_count

    @annotation_count.setter
    def annotation_count(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'annotation_count' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'annotation_count' field must be an integer in [-2147483648, 2147483647]"
        self._annotation_count = value

    @builtins.property
    def dataset_version(self):
        """Message field 'dataset_version'."""
        return self._dataset_version

    @dataset_version.setter
    def dataset_version(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'dataset_version' field must be of type 'str'"
        self._dataset_version = value


class Metaclass_UploadToRoboflow(type):
    """Metaclass of service 'UploadToRoboflow'."""

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
                'roarm_anygrasp_integration.srv.UploadToRoboflow')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__upload_to_roboflow

            from roarm_anygrasp_integration.srv import _upload_to_roboflow
            if _upload_to_roboflow.Metaclass_UploadToRoboflow_Request._TYPE_SUPPORT is None:
                _upload_to_roboflow.Metaclass_UploadToRoboflow_Request.__import_type_support__()
            if _upload_to_roboflow.Metaclass_UploadToRoboflow_Response._TYPE_SUPPORT is None:
                _upload_to_roboflow.Metaclass_UploadToRoboflow_Response.__import_type_support__()


class UploadToRoboflow(metaclass=Metaclass_UploadToRoboflow):
    from roarm_anygrasp_integration.srv._upload_to_roboflow import UploadToRoboflow_Request as Request
    from roarm_anygrasp_integration.srv._upload_to_roboflow import UploadToRoboflow_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
