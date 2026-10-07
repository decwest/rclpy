# Copyright 2017 Open Source Robotics Foundation, Inc.
#
# Licensed under the Apache License, Version 2.0 (the "License");
# you may not use this file except in compliance with the License.
# You may obtain a copy of the License at
#
#     http://www.apache.org/licenses/LICENSE-2.0
#
# Unless required by applicable law or agreed to in writing, software
# distributed under the License is distributed on an "AS IS" BASIS,
# WITHOUT WARRANTIES OR CONDITIONS OF ANY KIND, either express or implied.
# See the License for the specific language governing permissions and
# limitations under the License.

# flake8: noqa: A005


from pathlib import Path
from typing import Union

from rclpy.impl.implementation_singleton import rclpy_implementation as _rclpy
from rclpy.impl.logging_severity import LoggingSeverity as LoggingSeverity
from rclpy.impl.rcutils_logger import RcutilsLogger


_root_logger = RcutilsLogger()


def get_logger(name: str) -> RcutilsLogger:
    if not name:
        raise ValueError('Logger name must not be empty.')
    return _root_logger.get_child(name)


def get_logger_names(base_logger_name: Union[str, None] = None) -> list[str]:
    """
    Return sorted, unique registered logger names in this process.

    With a base name, return its exact match and dot-separated descendants.
    This filter describes logger names, not ownership by ROS nodes. With None,
    return all registered names. Names survive logger object destruction and
    context shutdown, but are cleared by logging.shutdown() or clear_config().
    Merely setting a level or emitting a log does not register a name.

    :raises ValueError: if the base name is empty
    """
    if base_logger_name == '':
        raise ValueError('Base logger name must not be empty.')
    return _rclpy.rclpy_logging_get_logger_names(base_logger_name)


def initialize() -> None:
    _rclpy.rclpy_logging_initialize()


def shutdown() -> None:
    _rclpy.rclpy_logging_shutdown()


def clear_config() -> None:
    """Clear the configuration of the logging system, e.g. logger levels."""
    shutdown()
    initialize()


def set_logger_level(name: str, level: Union[int, LoggingSeverity],
                     detailed_error: bool = False) -> None:
    level = LoggingSeverity(level)
    _rclpy.rclpy_logging_set_logger_level(name, level, detailed_error)


def get_logger_effective_level(name: str) -> LoggingSeverity:
    logger_level = _rclpy.rclpy_logging_get_logger_effective_level(name)
    return LoggingSeverity(logger_level)


def get_logger_level(name: str) -> LoggingSeverity:
    logger_level = _rclpy.rclpy_logging_get_logger_level(name)
    return LoggingSeverity(logger_level)


def get_logging_severity_from_string(log_severity: str) -> LoggingSeverity:
    return LoggingSeverity(
        _rclpy.rclpy_logging_severity_level_from_string(log_severity))


def get_logging_directory() -> Path:
    """
    Return the current logging directory being used.

    For more details, :func:`~rcl_logging_interface.rcl_logging_get_logging_directory`
    """
    return Path(_rclpy.rclpy_logging_get_logging_directory())
