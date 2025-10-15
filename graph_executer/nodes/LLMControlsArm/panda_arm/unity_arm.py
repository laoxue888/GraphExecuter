#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from rclpy.node import Node
from std_msgs.msg import Float64MultiArray
from sensor_msgs.msg import JointState
from panda_arm_msg.srv import ControlRvizArm, ControlRvizArm_Request, ControlUnityArm, ControlUnityArm_Request
from panda_arm_msg.srv import ControlUnityArmGripper, ControlUnityArmGripper_Request
import rclpy
from openai import OpenAI
from tf2_ros import TransformException
from tf2_ros.buffer import Buffer
from tf2_ros.transform_listener import TransformListener
from geometry_msgs.msg import TransformStamped
import json
from rclpy.node import Node
import re
from Qt import QtCore, QtWidgets
import os
from utils.general import find_nodes_folder

__all__ = ['UnityArmPositionControlNode', 'DeepSeekControlNode']
BASE_DIR = os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__))))

class NodeWidgetWrapper(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """
    def __init__(self, parent=None):
        super(NodeWidgetWrapper, self).__init__(parent)
        class MyCustomWidget(QtWidgets.QWidget):
            """
            Custom widget to be embedded inside a node.
            """
            def __init__(self, parent=None):
                super(MyCustomWidget, self).__init__(parent)
                
                self.btn = QtWidgets.QPushButton("get_end_effector_pose")

                layout = QtWidgets.QHBoxLayout(self)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.addWidget(self.btn)

                self.btn.clicked.connect(self.get_end_pose)

            def set_node_obj(self, obj):
                """"""
                self.node_obj = obj

            def get_end_pose(self,):
                """"""
                if not self.node_obj.is_created_node:
                    self.node_obj.create_ros2_node()
                self.node_obj.get_end_pose()
                self.node_obj.get_joint_states()
        # set the name for node property.
        self.set_name('my_widget')
        # set the custom widget.
        self.set_custom_widget(MyCustomWidget())
        # connect up the signals & slots.
        self.wire_signals()
    def wire_signals(self):
        """"""
    def get_value(self):
        """"""
    def set_value(self, value):
        """"""

class UnityArmPositionControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'UnityArmPositionControlNode'

    def __init__(self):
        super().__init__()
        self.add_input('position_x')
        self.add_input('position_y')
        self.add_input('position_z')
        self.add_input('orientation_x')
        self.add_input('orientation_y')
        self.add_input('orientation_z')
        self.add_input('orientation_w')
        self.add_input('gripper_position')
        self.add_input('answer')
        self.add_output('text_out')
        self.is_created_node = False

    def create_ros2_node(self,):
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.client_arm = self.srv_node.create_client(ControlRvizArm, 'control_rviz_arm')
        self.client_gripper = self.srv_node.create_client(ControlUnityArmGripper, 'control_unity_arm_gripper')
        self.client_send_trajectory = self.srv_node.create_client(ControlUnityArm, 'control_unity_arm')
        self.sub_end_pose = self.srv_node.create_subscription(
            Float64MultiArray,
            '/end_effector_pose',
            self.end_pose_listener_callback,
            10)
        self.sub_end_pose  # prevent unused variable warning
        self.end_pose = None
        self.is_get_end_pose = False

        self.sub_joint_states = self.srv_node.create_subscription(
            JointState,
            '/joint_states',
            self.joint_states_listener_callback,
            10)
        self.sub_joint_states  # prevent unused variable warning
        self.joint_states = None
        self.is_get_joint_states = False

        self.is_created_node=True

    def delete_ros2_node(self,):
        """"""
        self.srv_node.destroy_node()
        self.is_created_node = False
    
    def joint_states_listener_callback(self, msg):
        """"""
        self.joint_states = msg
        # print(f"end_pose: {self.end_pose}")
        self.is_get_joint_states = True

    def end_pose_listener_callback(self, msg):
        """"""
        self.end_pose = msg
        # print(f"end_pose: {self.end_pose}")
        self.is_get_end_pose = True

    def get_joint_states(self,):
        """"""
        while not self.is_get_joint_states:
            rclpy.spin_once(self.srv_node)
        joint_states = self.joint_states
        self.is_get_joint_states=False
        finger_joint_posi = joint_states.position[-1]
        self.set_property('gripper_position', finger_joint_posi)

    def get_end_pose(self,):
        # 通过话题获取当前的机械臂位置状态
        while not self.is_get_end_pose:
            rclpy.spin_once(self.srv_node)
        end_pose = self.end_pose.data[:]
        # 重置标志位
        self.is_get_end_pose = False

        # set property
        self.set_property('position.x', str(end_pose[0]))
        self.set_property('position.y', str(end_pose[1]))
        self.set_property('position.z', str(end_pose[2]))
        self.set_property('orientation.x', str(end_pose[3]))
        self.set_property('orientation.y', str(end_pose[4]))
        self.set_property('orientation.z', str(end_pose[5]))
        self.set_property('orientation.w', str(end_pose[6]))

        return end_pose

    def execute(self):
        """"""
        if not self.is_created_node:
            self.create_ros2_node()
        while not self.client_arm.wait_for_service(timeout_sec=1.0):
            self.srv_node.get_logger().info('client_arm not available, waiting again...')
        while not self.client_gripper.wait_for_service(timeout_sec=1.0):
            self.srv_node.get_logger().info('client_gripper not available, waiting again...')
        # 通过服务发送位置信号
        request_arm = ControlRvizArm_Request()

        position_x = getattr(self.input(0).connected_ports()[0].node(), 'position_x')
        position_y = getattr(self.input(0).connected_ports()[0].node(), 'position_y')
        position_z = getattr(self.input(0).connected_ports()[0].node(), 'position_z')
        orientation_x = getattr(self.input(0).connected_ports()[0].node(), 'orientation_x')
        orientation_y = getattr(self.input(0).connected_ports()[0].node(), 'orientation_y')
        orientation_z = getattr(self.input(0).connected_ports()[0].node(), 'orientation_z')
        orientation_w = getattr(self.input(0).connected_ports()[0].node(), 'orientation_w')
        gripper_position = getattr(self.input(0).connected_ports()[0].node(), 'gripper_position')
        request_arm.position = [float(position_x), 
                            float(position_y), 
                            float(position_z),
                            float(orientation_x),
                            float(orientation_y),
                            float(orientation_z),
                            float(orientation_w)]
        request_arm.gripper_width = float(gripper_position)
        
        future_arm = self.client_arm.call_async(request_arm)
        rclpy.spin_until_future_complete(self.srv_node, future_arm) # 重复运行这个会报：IndexError: wait set index too big

        response_arm = future_arm.result()

        if response_arm.success:
            # # # 这里通过服务发送轨迹到panda_joint_monitor的线程
            while not self.client_send_trajectory.wait_for_service(timeout_sec=1.0):
                self.srv_node.get_logger().info('client_send_trajectory not available, waiting again...')
            request_trajectory = ControlUnityArm_Request()
            request_trajectory.joint_trajectory = response_arm.joint_trajectory # 将规划结果的轨迹添加到请求中

            future_send_trajectory = self.client_send_trajectory.call_async(request_trajectory)
            rclpy.spin_until_future_complete(self.srv_node, future_send_trajectory) # 重复运行这个会报：IndexError: wait set index too big

            response_send_trajectory = future_send_trajectory.result()

            if response_send_trajectory.success:
                self.srv_node.get_logger().info("Trajectory sent successfully.")
            else:
                self.srv_node.get_logger().info("Failed to send trajectory.")

            self.messageSignal.emit(f'{request_arm.position} execution was successful.')
            # self.text_out = "执行成功！"
        else:
            self.messageSignal.emit(f'{request_arm.position} execution failed.')
            # self.text_out = "执行失败！"

        request_gripper = ControlUnityArmGripper_Request()
        request_gripper.gripper_position = float(gripper_position)

        future_gripper = self.client_gripper.call_async(request_gripper)

        rclpy.spin_until_future_complete(self.srv_node, future_gripper)

        response_gripper = future_gripper.result()

        if response_gripper.success:
            self.messageSignal.emit(f'response_gripper execution was successful.')
            # self.text_out = "执行成功！"
        else:
            self.messageSignal.emit(f'response_gripper execution failed.')
            # self.text_out = "执行失败！"
        setattr(self, 'text_out', getattr(self.input(0).connected_ports()[0].node(), 'answer'))
        self.delete_ros2_node()

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class DeepSeekControlNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'DeepSeekControlNode'

    def __init__(self):
        super().__init__()
        self.add_input('text_in')
        self.add_output('position_x')
        self.add_output('position_y')
        self.add_output('position_z')
        self.add_output('orientation_x')
        self.add_output('orientation_y')
        self.add_output('orientation_z')
        self.add_output('orientation_w')
        self.add_output('gripper_position')
        self.add_output('answer')

        self.add_text_input('max_mem_len', label="Max Memory Length")
        self.set_property("max_mem_len", "20")

        api_path = os.path.join(BASE_DIR, "res", "api", "llm.json")
        with open(api_path, 'r') as f:
            api_file = json.load(f)
            deepseek_api = api_file["deepseek_api"]

        self.client = OpenAI(api_key=deepseek_api, base_url="https://api.deepseek.com")
        system_prompt = """
                        你是一个六自由度的机械臂，其中的answer答复要有拟人性。

                        请按照用户的意图给出机械臂末端的位置和姿态。

                        json输出示例:
                        {
                            "position_x": 0.3, 
                            "position_y": 0.3, 
                            "position_z": 0.2, 
                            "orientation_x": 0.0,
                            "orientation_y": 0.0,
                            "orientation_z": 0.0,
                            "orientation_w": 1.0,
                            "gripper_position": 0.01, #范围是:0.01~0.04
                            "answer": "已给出目标位置和姿态，后续将进行规划执行动作。"
                        }
                        """
        system_message = {"role": "system", "content": system_prompt}

        self.messages = [system_message]

        self.is_created_node = False

    def create_ros2_node(self,):
        """"""
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.cli = self.srv_node.create_client(ControlRvizArm, 'control_rviz_arm')
        self.sub_end_pose = self.srv_node.create_subscription(
            Float64MultiArray,
            '/end_effector_pose',
            self.end_pose_listener_callback,
            10)
        self.sub_end_pose  # prevent unused variable warning

        self.end_pose = None
        self.is_get_end_pose = False
        self.is_created_node = True

    def delete_ros2_node(self,):
        """"""
        self.srv_node.destroy_node()
        self.is_created_node = False

    def end_pose_listener_callback(self, msg):
        """"""
        self.end_pose = msg
        # print(f"end_pose: {self.end_pose}")
        self.is_get_end_pose = True

    def execute(self):
        """"""
        if not self.is_created_node:
            self.create_ros2_node()

        while not self.cli.wait_for_service(timeout_sec=1.0):
            self.srv_node.get_logger().info('service not available, waiting again...')

        # 获取语音节点的文本输入
        text_in = self.input(0).connected_ports()[0].node().text_out
        
        # 通过话题获取当前的机械臂位置状态
        # position, rotation = self.get_end_effector_pose()
        while not self.is_get_end_pose:
            rclpy.spin_once(self.srv_node)
        end_pose = self.end_pose.data[:]
        # 重置标志位
        self.is_get_end_pose = False

        user_prompt = """
                        机械臂末端的当前位置和姿态是： "position_x": %s,"position_y": %s,"position_z": %s,"orientation_x": %s,"orientation_y": %s,"orientation_z": %s,"orientation_w": %s,
                        用户的意图是：%s
                    """ % (end_pose[0], end_pose[1], end_pose[2], end_pose[3], end_pose[4], end_pose[5], end_pose[6], text_in)

        self.messages.append({"role": "user", "content": user_prompt})

        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输入： "role": "user", "content": {user_prompt}')

        # 创建聊天请求
        chat_completion = self.client.chat.completions.create(
            messages=self.messages, model="deepseek-chat", )
        assistant_message = chat_completion.choices[0].message.content

        pattern = r'\{([^{}]*)\}'
        matches = re.findall(pattern, assistant_message)
        assistant_message = '{' + matches[0] + '}'

        self.messages.append(chat_completion.choices[0].message)

        # 如果超出最长记录长度，删除第二个消息
        if len(self.messages) > int(self.get_property("max_mem_len")):
            del self.messages[1:3]

        # 加判断，解析返回的json数据
        data_json = json.loads(assistant_message)
        self.messageSignal.emit(f'{self.NODE_NAME}的LLM输出：{data_json}')
        # self.text_out = data_json['answer']
        # self.messageSignal.emit(assistant_message)

        # 解析返回信号，通过服务发送位置信号
        setattr(self, 'position_x', data_json['position_x'])
        setattr(self, 'position_y', data_json['position_y'])
        setattr(self, 'position_z', data_json['position_z'])
        setattr(self, 'orientation_x', data_json['orientation_x'])
        setattr(self, 'orientation_y', data_json['orientation_y'])
        setattr(self, 'orientation_z', data_json['orientation_z'])
        setattr(self, 'orientation_w', data_json['orientation_w'])
        setattr(self, 'gripper_position', data_json['gripper_position'])
        setattr(self, 'answer', data_json['answer'])
        self.delete_ros2_node()

        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

