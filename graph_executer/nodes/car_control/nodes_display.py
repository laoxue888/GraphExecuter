#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from rclpy.node import Node
import rclpy
import json
from rclpy.node import Node
import re
from Qt import QtCore, QtWidgets
from sensor_msgs.msg import Image, CompressedImage
from cv_bridge import CvBridge
import cv2
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
from PySide6.Qt import *
import time
from utils.general import get_execution_order
# from panda_arm_msg.srv import YoloImage, YoloImage_Request
import pyqtgraph as pg
import numpy as np
from ultralytics import YOLO
import os
import logging
import math, threading
import jpeg4py as jpeg


BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

__all__ = ['SpinRos2Node','ImageDisplayByTopic']

class SpinThread(threading.Thread):
    def __init__(self, ros2_node):
        super(SpinThread, self).__init__()
        self.ros2_node = ros2_node
        self.is_running = True

        self.executor = rclpy.executors.MultiThreadedExecutor()
        self.executor.add_node(self.ros2_node)

    def run(self):
        while self.is_running:
            self.executor.spin_once()
            time.sleep(0.05)
            # print("SpinThread is running...")

class SpinRos2Node(BaseNode, QObject):
    __identifier__ = 'nodes.car_control.display'
    NODE_NAME = 'Spin ros2 node'

    def __init__(self):
        super(SpinRos2Node, self).__init__()
        self.add_input('ros2_node')

        self.add_checkbox('isDisplayImageLoop', text='开启图像显示循环')

        chk_widget = self.get_widget("isDisplayImageLoop")
        chk_widget.value_changed.connect(self.chk_value_changed)

        self.is_loop = False
    
    def chk_value_changed(self):
        """复选框值改变时的回调函数"""
        if self.get_property("isDisplayImageLoop"):
            self.is_loop = True
        else:
            self.is_loop = False
            try:
                self.spin_thread.is_running = False
                self.spin_thread = None
            except Exception as e:
                print(f"Error stopping spin thread: {e}")
                self.spin_thread = None

    def execute(self):
        """"""
        self.ros2_node = self.input(0).connected_ports()[0].node().ros2_node
        if not self.is_loop:
            rclpy.spin_once(self.ros2_node)
        else:
            # execution_order = get_execution_order(self)[:-1]
            # while self.is_loop:
            #     for node in execution_order:
            #         if hasattr(node, 'execute'):
            #             node.execute() # 运行节点
            #     ros2_node = self.input(0).connected_ports()[0].node().ros2_node
            #     rclpy.spin_once(ros2_node)
                # print(time.time())
            self.spin_thread = SpinThread(self.ros2_node)
            self.spin_thread.setDaemon(True)
            self.spin_thread.is_running = True
            self.spin_thread.start()
            self.spin_thread.join()
        
        self.messageSignal.emit(f'{self.NODE_NAME} executed.')

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

class VideoThread(QThread):
    # 定义一个信号，用于发送处理后的数据
    update_image = Signal(QPixmap)
    
    def __init__(self, widget_window=None):
        super().__init__()
        self.widget_window = widget_window

        self.num_frame = 0
        self.num_time = 0.0
        self.before_time = None
        self.cv_bridge = CvBridge()

        # model_path = os.path.join(BASE_DIR, 'res', 'models', 'YOLO', 'best.pt')
        # self.yolo_model = YOLO(model_path)

        self.fps = 0.0

    def camera_callback(self,data):
        """显示图像的函数，要使用信号来调用，不可外部调用"""
        image_array = jpeg.JPEG(np.frombuffer(data.data, dtype=np.uint8)).decode()
        # print(image_array.shape) # (480, 640, 3)

        height, width, channel = image_array.shape
        bytesPerLine = 3 * width
        qimage = QImage(image_array.data, width, height, bytesPerLine, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(qimage)

        self.update_image.emit(pixmap.scaled(self.widget_window.label_img.size(), Qt.KeepAspectRatio))


class ImageDisplayWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        from ui.nodes.nodes_display import ui_image_display
        
        self.ui = ui_image_display.Ui_ImageDisplayForm()
        self.ui.setupUi(self)
        # logging.getLogger().setLevel(logging.WARNING)
        self.label_img = QLabel(self)
        self.label_img.setMouseTracking(True)
        self.ui.verticalLayout.addWidget(self.label_img)

        # 创建工作线程
        self.video_thread = VideoThread(self)
        self.video_thread.start()
        self.video_thread.update_image.connect(self.label_img.setPixmap)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def closeEvent(self, event):
        self.node_obj.set_property("open_window", False)
        return super().closeEvent(event)

class NodeWidgetButton(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """
    def __init__(self, parent=None):
        super(NodeWidgetButton, self).__init__(parent)
        # set the name for node property.
        self.set_name('button_widget')
        # set the custom widget.
        class MyCustomWidget(QtWidgets.QWidget):
            """
            Custom widget to be embedded inside a node.
            """
            def __init__(self, parent=None):
                super(MyCustomWidget, self).__init__(parent)
                self.btn = QtWidgets.QPushButton('get image topic')
                layout = QtWidgets.QHBoxLayout(self)
                layout.setContentsMargins(0, 0, 0, 0)
                layout.addWidget(self.btn)
                self.btn.clicked.connect(self.get_image_topic)
            def get_image_topic(self):
                """"""
                if not self.node_obj.is_created_ros2_node:
                    self.node_obj.create_ros2_node()

                # rclpy.spin_once(self.srv_node)
                topic_names_and_types = self.node_obj.srv_node.get_topic_names_and_types()
                image_topic = []
                # print("topic_names_and_types: ", topic_names_and_types)
                for topic_name, topic_types in topic_names_and_types:
                    # print(f"Topic: {topic_name}, Types: {topic_types}")
                    if topic_types[0] == "sensor_msgs/msg/CompressedImage":
                        print(f"Topic: {topic_name} is Image type")
                        image_topic.append(topic_name)

                print("image_topic: ", image_topic)
                self.node_obj.set_property("image_topic", image_topic)

                # self.node_obj.delete_ros2_node()

            def set_node_obj(self, obj):
                """"""
                self.node_obj = obj

        self.btn_widget = MyCustomWidget()
        self.set_custom_widget(self.btn_widget)
        # connect up the signals & slots.
        self.wire_signals()
    def wire_signals(self):
        """"""
    def get_value(self):
        """"""
    def set_value(self, value):
        """"""

class ImageDisplayByTopic(BaseNode, QObject):
    __identifier__ = 'nodes.car_control.display'
    NODE_NAME = 'Image display by topic'

    def __init__(self):
        super(ImageDisplayByTopic, self).__init__()
        # self.add_input('start')
        self.add_output('ros2_node')

        self.is_created_ros2_node = False

        btn_widget = NodeWidgetButton(self.view)
        btn_widget.btn_widget.set_node_obj(self)
        self.add_custom_widget(btn_widget, tab='Custom')

        self.add_combo_menu("image_topic", "image_topic")
        self.add_checkbox("open_window", text='show window')
        self.myui=ImageDisplayWidget()
        self.myui.set_node_obj(self)
        self.chk_value_changed()

        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        combo_menu_widget = self.get_widget("image_topic")
        combo_menu_widget.value_changed.connect(self.topic_changed)
        self.ros2_node = None

        self.update_timer = QtCore.QTimer()
        self.update_timer.timeout.connect(self.myui.update)
        self.update_timer.start(500)

    def execute(self):
        """节点执行函数"""
        if not self.is_created_ros2_node:
            self.create_ros2_node()
        self.ros2_node = self.srv_node

    def topic_changed(self):
        """"""
        if not self.is_created_ros2_node:
            self.create_ros2_node()
        combo_menu=self.get_widget("image_topic")
        combo_menu_text=combo_menu.get_value()
        if combo_menu_text == "":
            return
        self.sub_img = self.srv_node.create_subscription(CompressedImage, combo_menu_text.strip(), self.myui.video_thread.camera_callback, 10)
        print(f"topic_changed: {combo_menu_text.strip()}")

    def chk_value_changed(self):
        if self.get_property("open_window"):
            self.myui.show() 
        else:
            self.myui.close()

    def create_ros2_node(self,):
        self.srv_node = Node(self.NODE_NAME.replace(' ', '_'))
        self.is_created_ros2_node=True

    def delete_ros2_node(self,):
        """"""
        if self.is_created_ros2_node:
            self.srv_node.destroy_node()
            self.is_created_ros2_node = False

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.myui.close()

    def _del_node(self):
        """删除节点前调用"""
        self.myui.close()
        self.delete_ros2_node()
        self.myui.video_thread.quit()

    



