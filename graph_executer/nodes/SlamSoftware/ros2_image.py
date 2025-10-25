#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
from rclpy.node import Node
from sensor_msgs.msg import Image,CompressedImage
import cv2
from PySide6.QtCore import QThread, Signal, QMutex, QMutexLocker
from PySide6.QtWidgets import (
    QApplication, QWidget, QPushButton, QLabel, QVBoxLayout
)
import time
import rclpy
import numpy as np

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('Ros2ImageNode',)


class WorkerThread(QThread):
    # 定义信号：传递字符串类型的进度信息（PySide6 用 Signal，而非 pyqtSignal）
    def __init__(self, ros2_node):
        super().__init__()
        self.is_run = False

        self.ros2_node = ros2_node

    def run(self):
        """"""
        while self.is_run and rclpy.ok():  # 增加rclpy状态判断
            rclpy.spin_once(self.ros2_node, timeout_sec=0.01)  # 增加超时，避免阻塞
            self.msleep(1)  # 微小延迟，降低CPU占用

    def stop(self):
        self.is_run = False
        self.wait()  # 等待线程安全退出

class Ros2ImageNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Publish Ros2 Image Topic'

    def __init__(self):
        super().__init__()
        self.add_input('image_data')
        self.add_text_input('topic', 'Topic', '/camera/image/compressed')
        self.add_text_input('quality', 'JPEG Quality', '90')  # 增加压缩质量参数
        self.add_output('spin_once')

        self.is_created_node = False
        self.is_spin_node = False

        self.mutex = QMutex()

    def create_ros2_node(self,):
        """"""
        self.pub_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.publisher_image = self.pub_node.create_publisher(
            CompressedImage, 
            self.get_property('topic'), 
            10
        )

        self.thread_spin_node = WorkerThread(self.pub_node)

        with QMutexLocker(self.mutex):
            self.thread_spin_node.ros2_node = True
        # 启动线程
        self.thread_spin_node.start()

        self.is_created_node = True

    def delete_ros2_node(self,):
        """"""
        if self.thread_spin_node:
            self.thread_spin_node.stop()  # 调用安全停止方法
            self.thread_spin_node = None
        
        if hasattr(self, 'pub_node'):
            self.pub_node.destroy_node()
        self.is_created_node = False

    def cv2_to_compressed_imgmsg(self, cv_image):
        """将OpenCV图像转换为CompressedImage消息"""
        # 确保图像是BGR格式（OpenCV默认）
        if len(cv_image.shape) == 3 and cv_image.shape[2] == 3:
            encoding = "jpeg"
            # 获取压缩质量参数（1-100，越高质量越好）
            try:
                quality = int(self.get_property('quality'))
                quality = max(1, min(100, quality))  # 限制范围
            except ValueError:
                quality = 90  # 默认值

            # 编码为JPEG
            encode_param = [int(cv2.IMWRITE_JPEG_QUALITY), quality]
            result, encimg = cv2.imencode('.jpg', cv_image, encode_param)
            if not result:
                raise RuntimeError("Could not encode image to JPEG")

            # 构建CompressedImage消息
            msg = CompressedImage()
            msg.header.stamp = self.pub_node.get_clock().now().to_msg()  # 增加时间戳
            msg.format = "jpeg"
            msg.data = np.array(encimg).tobytes()
            return msg
        else:
            raise ValueError("Unsupported image format (expected 3-channel BGR)")

    def execute(self):
        """节点执行函数"""
        image_data = getattr(self.input(0).connected_ports()[0].node(), self.input(0).connected_ports()[0].name())

        if image_data.img is None:
            print('No data to publish')
            return

        if not self.is_created_node:
            self.create_ros2_node()

        compressed_msg = self.cv2_to_compressed_imgmsg(image_data.img)
        self.publisher_image.publish(compressed_msg)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def set_widget_parent(self, parent):
        self.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        if self.is_created_node:
            self.delete_ros2_node()

    def _del_node(self):
        """删除节点前调用"""
        if self.is_created_node:
            self.delete_ros2_node()

