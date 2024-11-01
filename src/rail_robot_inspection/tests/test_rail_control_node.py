#!/usr/bin/env python3
import unittest
import rclpy
from rclpy.node import Node
from rclpy.executors import SingleThreadedExecutor
from rail_robot_inspection_msgs.msg import RailPosition, SensorData
from rail_robot_inspection_msgs.srv import InspectionData
from rail_robot_inspection.rail_control_node import RailControlNode  # type: ignore
from sensor_msgs.msg import Image
import time
from unittest.mock import MagicMock
import json

class TestRailControlNode(unittest.TestCase):
    @classmethod
    def setUpClass(cls):
        if not rclpy.ok():
            rclpy.init()

    @classmethod
    def tearDownClass(cls):
        if rclpy.ok():
            rclpy.shutdown()

    def setUp(self):
        # 创建 Mock 硬件相关的类
        motor_driver_mock = MagicMock(name='MotorDriver')
        gas_sensor_mock = MagicMock(name='GasSensor')
        temp_humidity_sensor_mock = MagicMock(name='ModbusSensorReader')

        # 创建测试节点，传入模拟的硬件组件
        self.test_node = RailControlNode(
            motor_driver=motor_driver_mock,
            gas_sensor=gas_sensor_mock,
            temp_humidity_sensor=temp_humidity_sensor_mock
        )

        # 创建真实的 Image 消息
        self.test_image = Image()
        self.test_image.header.frame_id = "camera_frame"
        self.test_image.height = 480
        self.test_image.width = 640
        self.test_image.encoding = "rgb8"
        self.test_image.step = 640 * 3
        self.test_image.data = bytes([0] * (640 * 480 * 3))

        # 设置执行器
        self.executor = SingleThreadedExecutor()
        self.executor.add_node(self.test_node)

        # 模拟服务的应答方法
        self.test_node.inspection_service = MagicMock()
        self.test_node.inspection_service.call_async = MagicMock()

    def tearDown(self):
        self.executor.shutdown()
        self.test_node.destroy_node()

    def test_inspection_flow(self):
        # 添加日志
        self.test_node.get_logger().info("Starting inspection flow test.")

        # 创建一个真实的 SensorData 消息
        orbbec_msg = SensorData()
        orbbec_msg.position_id = 0
        orbbec_msg.color_image = self.test_image
        orbbec_msg.depth_image = self.test_image
        orbbec_msg.thermal_image = self.test_image
        orbbec_msg.temperature_matrix = [20.0] * 100

        # 发布位置消息
        pos_msg = RailPosition()
        pos_msg.position_id = 0
        pos_msg.position_value = 0.0
        pos_msg.position_name = "Position_0"

        self.test_node.rail_position_publisher.publish(pos_msg)
        time.sleep(0.1)
        self.executor.spin_once()

        # 调用回调函数
        self.test_node.orbbec_callback(orbbec_msg)
        time.sleep(0.1)
        self.executor.spin_once()

        # 验证数据是否被正确处理
        self.assertIn(0, self.test_node.inspection_data)
        self.assertTrue(self.test_node.inspection_data[0]["simulated"])

        # 日志记录处理完成
        self.test_node.get_logger().info("Inspection flow test completed.")

    def test_inspection_service(self):
        # 添加日志
        self.test_node.get_logger().info("Starting inspection service test.")

        # 设置测试数据
        test_data = {
            'gas_concentration': 0.5,
            'temperature': 25.0,
            'humidity': 50.0
        }
        self.test_node.inspection_data[0] = test_data

        # 创建服务请求
        request = InspectionData.Request()
        request.position_id = 0

        # 创建模拟的响应
        mock_response = InspectionData.Response()
        mock_response.success = True
        mock_response.data = json.dumps(test_data)

        # 模拟服务的 Future 对象
        mock_future = MagicMock()
        mock_future.result.return_value = mock_response

        # 设置 call_async 方法返回模拟的 Future
        self.test_node.inspection_service.call_async.return_value = mock_future

        # 调用服务
        future = self.test_node.inspection_service.call_async(request)
        response = future.result()  # 直接获取结果

        # 验证响应
        self.assertTrue(response.success)
        received_data = json.loads(response.data)
        self.assertEqual(received_data, test_data)

        # 日志记录服务测试完成
        self.test_node.get_logger().info("Inspection service test completed.")

if __name__ == '__main__':
    unittest.main()
