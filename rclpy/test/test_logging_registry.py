# Copyright 2026 Fumiya Ohnishi
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

import gc
from typing import Iterator
from unittest.mock import patch

import pytest

import rclpy
from rclpy.context import Context
from rclpy.impl.rcutils_logger import RcutilsLogger
from rclpy.logging import get_logger
from rclpy.logging import get_logger_names
from rclpy.logging import LoggingSeverity


@pytest.fixture(autouse=True)
def reset_logging_registry() -> Iterator[None]:
    rclpy.logging.shutdown()
    yield
    gc.collect()
    rclpy.logging.shutdown()


def test_unlogged_names_survive_object_destruction_before_ros_init() -> None:
    assert not rclpy.ok()
    assert get_logger_names() == []
    logger = get_logger('registry_py')
    child = logger.get_child('child')
    grandchild = child.get_child('grandchild')
    assert get_logger('registry_py.child').name == child.name
    assert RcutilsLogger('registry_py.direct').name == 'registry_py.direct'
    del grandchild, child, logger
    gc.collect()
    assert get_logger_names('registry_py') == [
        'registry_py', 'registry_py.child', 'registry_py.child.grandchild', 'registry_py.direct']
    assert '' not in get_logger_names()


def test_filter_and_snapshot() -> None:
    for name in ('foo.z', 'foobar', 'foo.a.deep', 'foo', 'fo'):
        get_logger(name)
    snapshot = get_logger_names('foo')
    assert snapshot == ['foo', 'foo.a.deep', 'foo.z']
    assert get_logger_names('absent') == []
    get_logger('foo.new')
    assert snapshot == ['foo', 'foo.a.deep', 'foo.z']
    with pytest.raises(ValueError):
        get_logger_names('')
    with pytest.raises(TypeError):
        get_logger_names(123)  # type: ignore[arg-type]


def test_levels_are_independent_of_registration() -> None:
    rclpy.logging.set_logger_level('configured_only', LoggingSeverity.DEBUG)
    assert get_logger_names() == []
    rclpy.logging.set_logger_level('registry_py', LoggingSeverity.WARN)
    rclpy.logging.set_logger_level('registry_py.explicit', LoggingSeverity.DEBUG)
    logger = get_logger('registry_py')
    child = logger.get_child('child')
    explicit = logger.get_child('explicit')
    assert rclpy.logging.get_logger_level(child.name) == LoggingSeverity.UNSET
    assert child.get_effective_level() == LoggingSeverity.WARN
    assert explicit.get_effective_level() == LoggingSeverity.DEBUG
    assert 'configured_only' not in get_logger_names()


@pytest.mark.parametrize('enable_rosout', [False, True])
def test_node_logger_and_context_shutdown(enable_rosout: bool) -> None:
    context = Context()
    rclpy.init(context=context)
    node = rclpy.create_node(
        'registry_node', namespace='/ns', context=context,
        enable_rosout=enable_rosout, enable_logger_service=False)
    try:
        child = node.get_logger().get_child('vision')
        assert child.name == 'ns.registry_node.vision'
        del child
    finally:
        node.destroy_node()
        context.shutdown()
    assert get_logger_names('ns.registry_node') == [
        'ns.registry_node', 'ns.registry_node.vision']


def test_explicit_logging_shutdown_clears_names() -> None:
    get_logger('old_session')
    rclpy.logging.shutdown()
    assert get_logger_names() == []
    get_logger('new_session')
    assert get_logger_names() == ['new_session']
    rclpy.logging.clear_config()
    assert get_logger_names() == []


def test_registration_failure_precedes_rosout_registration() -> None:
    logger = get_logger('registry_py')
    with patch('rclpy.impl.rcutils_logger._rclpy.rclpy_logging_register_logger',
               side_effect=RuntimeError('allocation failed')):
        with patch('rclpy.impl.rcutils_logger._rclpy.rclpy_logging_rosout_add_sublogger') as add:
            with pytest.raises(RuntimeError, match='allocation failed'):
                logger.get_child('failed')
            add.assert_not_called()
    assert get_logger_names('registry_py') == ['registry_py']
