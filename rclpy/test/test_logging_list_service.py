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

from typing import Optional
import unittest
from unittest.mock import patch

from rcl_interfaces.msg import LoggerLevel
from rcl_interfaces.srv import GetLoggerLevels, ListLoggers, SetLoggerLevels
import rclpy
from rclpy.context import Context
from rclpy.executors import SingleThreadedExecutor
from rclpy.logging import get_logger, get_logger_level, LoggingSeverity, set_logger_level
from rclpy.logging_service import LoggingService
from rclpy.node import Node


class TestListLoggersService(unittest.TestCase):

    def setUp(self) -> None:
        rclpy.logging.shutdown()
        self.context = Context()
        rclpy.init(context=self.context)
        self.executor = SingleThreadedExecutor(context=self.context)
        self.nodes: list[Node] = []

    def tearDown(self) -> None:
        self.executor.shutdown()
        for node in reversed(self.nodes):
            node.destroy_node()
        self.context.shutdown()
        rclpy.logging.shutdown()

    def create_node(
        self, name: str, namespace: str = '/', *, enable_logger_service: bool = True,
        enable_rosout: bool = True, cli_args: Optional[list[str]] = None
    ) -> Node:
        node = rclpy.create_node(
            name, namespace=namespace, context=self.context,
            enable_logger_service=enable_logger_service, enable_rosout=enable_rosout,
            cli_args=cli_args)
        self.nodes.append(node)
        self.executor.add_node(node)
        return node

    def list_loggers(self, node: Node) -> list[str]:
        client = node.create_client(ListLoggers, node.get_fully_qualified_name() + '/list_loggers')
        try:
            self.assertTrue(client.wait_for_service(2))
            future = client.call_async(ListLoggers.Request())
            self.executor.spin_until_future_complete(future, 5)
            self.assertTrue(future.done())
            response = future.result()
            assert response is not None
            return list(response.names)
        finally:
            node.destroy_client(client)

    def assert_hierarchy(self, enable_rosout: bool) -> None:
        node = self.create_node(
            'original', '/original_ns', enable_rosout=enable_rosout,
            cli_args=['--ros-args', '-r', '__node:=renamed', '-r', '__ns:=/robot'])
        base = 'robot.renamed'
        self.assertEqual(base, node.get_logger().name)
        child = node.get_logger().get_child('z')
        del child
        child = node.get_logger().get_child('a')
        grandchild = child.get_child('deep')
        self.assertEqual(child.name, get_logger(base + '.a').name)
        del grandchild, child
        get_logger(base + '.independent')
        get_logger(base + '_other')
        get_logger('rclcpp')
        set_logger_level(base + '.configured_only', LoggingSeverity.WARN)
        self.assertEqual(
            [base, base + '.a', base + '.a.deep', base + '.independent', base + '.z'],
            self.list_loggers(node))

    def test_hierarchy_after_remapping(self) -> None:
        self.assert_hierarchy(enable_rosout=True)

    def test_hierarchy_without_rosout(self) -> None:
        self.assert_hierarchy(enable_rosout=False)

    def test_current_registry_and_existing_level_services(self) -> None:
        node = self.create_node('discovery')
        first = self.list_loggers(node)
        self.assertEqual(['discovery'], first)
        child = node.get_logger().get_child('unlogged')
        names = self.list_loggers(node)
        self.assertEqual(['discovery', 'discovery.unlogged'], names)
        self.assertEqual(['discovery'], first)
        self.assertEqual(LoggingSeverity.UNSET, get_logger_level(child.name))

        set_client = node.create_client(SetLoggerLevels, '/discovery/set_logger_levels')
        try:
            self.assertTrue(set_client.wait_for_service(2))
            request = SetLoggerLevels.Request()
            request.levels = [LoggerLevel(name=names[-1], level=int(LoggingSeverity.WARN))]
            future = set_client.call_async(request)
            self.executor.spin_until_future_complete(future, 5)
            self.assertTrue(future.done())
            response = future.result()
            assert response is not None
            self.assertEqual(1, len(response.results))
            self.assertTrue(response.results[0].successful)
        finally:
            node.destroy_client(set_client)

        get_client = node.create_client(GetLoggerLevels, '/discovery/get_logger_levels')
        try:
            self.assertTrue(get_client.wait_for_service(2))
            get_request = GetLoggerLevels.Request()
            get_request.names = names
            get_future = get_client.call_async(get_request)
            self.executor.spin_until_future_complete(get_future, 5)
            self.assertTrue(get_future.done())
            get_response = get_future.result()
            assert get_response is not None
            self.assertEqual(
                [LoggingSeverity.UNSET, LoggingSeverity.WARN],
                [level.level for level in get_response.levels])
        finally:
            node.destroy_client(get_client)
        self.assertEqual(LoggingSeverity.WARN, child.get_effective_level())
        self.assertEqual(names, self.list_loggers(node))

    def test_service_is_disabled_by_default(self) -> None:
        node = rclpy.create_node('disabled_list', context=self.context)
        self.nodes.append(node)
        client = node.create_client(ListLoggers, '/disabled_list/list_loggers')
        try:
            self.assertFalse(client.wait_for_service(0.1))
        finally:
            node.destroy_client(client)

    def test_shared_process_uses_names_without_tracking_ownership(self) -> None:
        foo = self.create_node('foo')
        foobar = self.create_node('foobar')
        nested = self.create_node('bar', '/foo')
        child = foo.get_logger().get_child('child')
        self.assertEqual('foo.child', child.name)
        self.assertEqual(['foo', 'foo.bar', 'foo.child'], self.list_loggers(foo))
        self.assertEqual(['foobar'], self.list_loggers(foobar))
        self.assertEqual(['foo.bar'], self.list_loggers(nested))

    def test_no_special_case_for_system_logger_names(self) -> None:
        node = self.create_node('rclcpp', enable_rosout=False)
        child = node.get_logger().get_child('child')
        self.assertEqual('rclcpp.child', child.name)
        self.assertEqual(['rclcpp', 'rclcpp.child'], self.list_loggers(node))

    def test_enumeration_failure_is_not_an_empty_success_response(self) -> None:
        node = self.create_node('failure', enable_logger_service=False)
        service = LoggingService(node)
        response = ListLoggers.Response()
        response.names = ['unchanged']
        with patch('rclpy.logging_service.rclpy.logging.get_logger_names',
                   side_effect=RuntimeError('enumeration failed')):
            with self.assertRaisesRegex(RuntimeError, 'enumeration failed'):
                service._list_loggers(ListLoggers.Request(), response)
        self.assertEqual(['unchanged'], response.names)
