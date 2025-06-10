# generated from rosidl_generator_py/resource/_idl.py.em
# with input from roarm_anygrasp_integration:srv/ConfigureRoboflow.idl
# generated code does not contain a copyright notice


# Import statements for member types

import builtins  # noqa: E402, I100

import math  # noqa: E402, I100

import rosidl_parser.definition  # noqa: E402, I100


class Metaclass_ConfigureRoboflow_Request(type):
    """Metaclass of message 'ConfigureRoboflow_Request'."""

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
                'roarm_anygrasp_integration.srv.ConfigureRoboflow_Request')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__configure_roboflow__request
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__configure_roboflow__request
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__configure_roboflow__request
            cls._TYPE_SUPPORT = module.type_support_msg__srv__configure_roboflow__request
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__configure_roboflow__request

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ConfigureRoboflow_Request(metaclass=Metaclass_ConfigureRoboflow_Request):
    """Message class 'ConfigureRoboflow_Request'."""

    __slots__ = [
        '_api_key',
        '_workspace_name',
        '_project_name',
        '_dataset_version',
        '_auto_upload',
        '_include_failed_grasps',
        '_annotation_format',
        '_min_confidence_for_upload',
    ]

    _fields_and_field_types = {
        'api_key': 'string',
        'workspace_name': 'string',
        'project_name': 'string',
        'dataset_version': 'string',
        'auto_upload': 'boolean',
        'include_failed_grasps': 'boolean',
        'annotation_format': 'string',
        'min_confidence_for_upload': 'float',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('float'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.api_key = kwargs.get('api_key', str())
        self.workspace_name = kwargs.get('workspace_name', str())
        self.project_name = kwargs.get('project_name', str())
        self.dataset_version = kwargs.get('dataset_version', str())
        self.auto_upload = kwargs.get('auto_upload', bool())
        self.include_failed_grasps = kwargs.get('include_failed_grasps', bool())
        self.annotation_format = kwargs.get('annotation_format', str())
        self.min_confidence_for_upload = kwargs.get('min_confidence_for_upload', float())

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
        if self.api_key != other.api_key:
            return False
        if self.workspace_name != other.workspace_name:
            return False
        if self.project_name != other.project_name:
            return False
        if self.dataset_version != other.dataset_version:
            return False
        if self.auto_upload != other.auto_upload:
            return False
        if self.include_failed_grasps != other.include_failed_grasps:
            return False
        if self.annotation_format != other.annotation_format:
            return False
        if self.min_confidence_for_upload != other.min_confidence_for_upload:
            return False
        return True

    @classmethod
    def get_fields_and_field_types(cls):
        from copy import copy
        return copy(cls._fields_and_field_types)

    @builtins.property
    def api_key(self):
        """Message field 'api_key'."""
        return self._api_key

    @api_key.setter
    def api_key(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'api_key' field must be of type 'str'"
        self._api_key = value

    @builtins.property
    def workspace_name(self):
        """Message field 'workspace_name'."""
        return self._workspace_name

    @workspace_name.setter
    def workspace_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'workspace_name' field must be of type 'str'"
        self._workspace_name = value

    @builtins.property
    def project_name(self):
        """Message field 'project_name'."""
        return self._project_name

    @project_name.setter
    def project_name(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'project_name' field must be of type 'str'"
        self._project_name = value

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

    @builtins.property
    def auto_upload(self):
        """Message field 'auto_upload'."""
        return self._auto_upload

    @auto_upload.setter
    def auto_upload(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'auto_upload' field must be of type 'bool'"
        self._auto_upload = value

    @builtins.property
    def include_failed_grasps(self):
        """Message field 'include_failed_grasps'."""
        return self._include_failed_grasps

    @include_failed_grasps.setter
    def include_failed_grasps(self, value):
        if __debug__:
            assert \
                isinstance(value, bool), \
                "The 'include_failed_grasps' field must be of type 'bool'"
        self._include_failed_grasps = value

    @builtins.property
    def annotation_format(self):
        """Message field 'annotation_format'."""
        return self._annotation_format

    @annotation_format.setter
    def annotation_format(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'annotation_format' field must be of type 'str'"
        self._annotation_format = value

    @builtins.property
    def min_confidence_for_upload(self):
        """Message field 'min_confidence_for_upload'."""
        return self._min_confidence_for_upload

    @min_confidence_for_upload.setter
    def min_confidence_for_upload(self, value):
        if __debug__:
            assert \
                isinstance(value, float), \
                "The 'min_confidence_for_upload' field must be of type 'float'"
            assert not (value < -3.402823466e+38 or value > 3.402823466e+38) or math.isinf(value), \
                "The 'min_confidence_for_upload' field must be a float in [-3.402823466e+38, 3.402823466e+38]"
        self._min_confidence_for_upload = value


# Import statements for member types

# already imported above
# import builtins

# already imported above
# import rosidl_parser.definition


class Metaclass_ConfigureRoboflow_Response(type):
    """Metaclass of message 'ConfigureRoboflow_Response'."""

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
                'roarm_anygrasp_integration.srv.ConfigureRoboflow_Response')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._CREATE_ROS_MESSAGE = module.create_ros_message_msg__srv__configure_roboflow__response
            cls._CONVERT_FROM_PY = module.convert_from_py_msg__srv__configure_roboflow__response
            cls._CONVERT_TO_PY = module.convert_to_py_msg__srv__configure_roboflow__response
            cls._TYPE_SUPPORT = module.type_support_msg__srv__configure_roboflow__response
            cls._DESTROY_ROS_MESSAGE = module.destroy_ros_message_msg__srv__configure_roboflow__response

    @classmethod
    def __prepare__(cls, name, bases, **kwargs):
        # list constant names here so that they appear in the help text of
        # the message class under "Data and other attributes defined here:"
        # as well as populate each message instance
        return {
        }


class ConfigureRoboflow_Response(metaclass=Metaclass_ConfigureRoboflow_Response):
    """Message class 'ConfigureRoboflow_Response'."""

    __slots__ = [
        '_success',
        '_message',
        '_workspace_url',
        '_project_url',
        '_total_images_in_dataset',
    ]

    _fields_and_field_types = {
        'success': 'boolean',
        'message': 'string',
        'workspace_url': 'string',
        'project_url': 'string',
        'total_images_in_dataset': 'int32',
    }

    SLOT_TYPES = (
        rosidl_parser.definition.BasicType('boolean'),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.UnboundedString(),  # noqa: E501
        rosidl_parser.definition.BasicType('int32'),  # noqa: E501
    )

    def __init__(self, **kwargs):
        assert all('_' + key in self.__slots__ for key in kwargs.keys()), \
            'Invalid arguments passed to constructor: %s' % \
            ', '.join(sorted(k for k in kwargs.keys() if '_' + k not in self.__slots__))
        self.success = kwargs.get('success', bool())
        self.message = kwargs.get('message', str())
        self.workspace_url = kwargs.get('workspace_url', str())
        self.project_url = kwargs.get('project_url', str())
        self.total_images_in_dataset = kwargs.get('total_images_in_dataset', int())

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
        if self.workspace_url != other.workspace_url:
            return False
        if self.project_url != other.project_url:
            return False
        if self.total_images_in_dataset != other.total_images_in_dataset:
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
    def workspace_url(self):
        """Message field 'workspace_url'."""
        return self._workspace_url

    @workspace_url.setter
    def workspace_url(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'workspace_url' field must be of type 'str'"
        self._workspace_url = value

    @builtins.property
    def project_url(self):
        """Message field 'project_url'."""
        return self._project_url

    @project_url.setter
    def project_url(self, value):
        if __debug__:
            assert \
                isinstance(value, str), \
                "The 'project_url' field must be of type 'str'"
        self._project_url = value

    @builtins.property
    def total_images_in_dataset(self):
        """Message field 'total_images_in_dataset'."""
        return self._total_images_in_dataset

    @total_images_in_dataset.setter
    def total_images_in_dataset(self, value):
        if __debug__:
            assert \
                isinstance(value, int), \
                "The 'total_images_in_dataset' field must be of type 'int'"
            assert value >= -2147483648 and value < 2147483648, \
                "The 'total_images_in_dataset' field must be an integer in [-2147483648, 2147483647]"
        self._total_images_in_dataset = value


class Metaclass_ConfigureRoboflow(type):
    """Metaclass of service 'ConfigureRoboflow'."""

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
                'roarm_anygrasp_integration.srv.ConfigureRoboflow')
            logger.debug(
                'Failed to import needed modules for type support:\n' +
                traceback.format_exc())
        else:
            cls._TYPE_SUPPORT = module.type_support_srv__srv__configure_roboflow

            from roarm_anygrasp_integration.srv import _configure_roboflow
            if _configure_roboflow.Metaclass_ConfigureRoboflow_Request._TYPE_SUPPORT is None:
                _configure_roboflow.Metaclass_ConfigureRoboflow_Request.__import_type_support__()
            if _configure_roboflow.Metaclass_ConfigureRoboflow_Response._TYPE_SUPPORT is None:
                _configure_roboflow.Metaclass_ConfigureRoboflow_Response.__import_type_support__()


class ConfigureRoboflow(metaclass=Metaclass_ConfigureRoboflow):
    from roarm_anygrasp_integration.srv._configure_roboflow import ConfigureRoboflow_Request as Request
    from roarm_anygrasp_integration.srv._configure_roboflow import ConfigureRoboflow_Response as Response

    def __init__(self):
        raise NotImplementedError('Service classes can not be instantiated')
