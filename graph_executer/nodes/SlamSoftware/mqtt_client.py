#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
import cv2
import paho.mqtt.client as mqtt
import numpy as np
import argparse
import threading
from queue import Queue
import time
import json
from datetime import datetime
import os
from PySide6.QtCore import QObject, Signal
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
import numpy as np
import cv2
from PySide6.QtGui import QImage, QPixmap, QPolygonF, QPen, QBrush, QColor
import math
from PySide6.QtWidgets import QGraphicsPixmapItem, QGraphicsScene, QWidget
from PySide6.QtCore import Qt, QPointF, QTimer

from utils.general import get_execution_order

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
# from yolov8_msgs.msg import Yolov8Inference
# from cv_bridge import CvBridge

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('MQTTClientNode', "SpinMQTTClientNode",)


class ImageData:
    def __init__(self, img=None, metadata=None, delay_ms=0):
        self.img = img
        self.metadata = metadata
        self.delay_ms = delay_ms

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_mouse = False
        self.image_data = ImageData()
        self.ui_obj = None

        self.timer = QTimer()
        self.timer.timeout.connect(self.update)

        self.save_directory = "/root/workspace/downloads/calaibration_images/"
        
    def update(self):
        """"""
        self.ui_obj.ui.label.setText("Press the left mouse button in the window to save the image.")

    def setImage(self, image:ImageData):
        self.image_data.img = image.img
        self.image_data.metadata = image.metadata
        self.image_data.delay_ms = image.delay_ms

    def setUiObj(self,ui_obj):
        """"""
        if ui_obj is not None:
            self.ui_obj = ui_obj

    def mouseMoveEvent(self,event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

    def setSaveDirectory(self, directory):
        if os.path.isdir(directory):
            self.save_directory = directory

    def mousePressEvent(self, event):
        self.click_mouse = True
        # 保存图像
        if event.button() == Qt.LeftButton:
            print("left click")
            if self.image_data.img is not None:
                save_path = self.save_directory
                if not os.path.exists(save_path):
                    os.makedirs(save_path)
                timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
                filename = os.path.join(save_path, f"image_{timestamp}.png")
                cv2.imwrite(filename, self.image_data.img)
                print(f"The image has been saved as: {filename}")
                self.ui_obj.ui.label.setText(f"The image has been saved as: {filename}")
                self.timer.start(3000)
        elif event.button() == Qt.RightButton:
            print("right click")

class ImageDisplayWidget(QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    mqtt_callback_signal=Signal(ImageData)

    def __init__(self, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        from .ui.ui_mqtt import Ui_Form

        self.ui = Ui_Form()
        self.ui.setupUi(self)
        self.resize(700, 580)
        self.scene = GraphicsScene(self.ui.graphicsView)
        self.scene.setUiObj(self)

        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setMouseTracking(True)

        self.mqtt_callback_signal.connect(self.mqtt_callback)
        self.image_data = ImageData()

    def mqtt_callback(self, data: ImageData):
        """"""
        if data.img is None:
            return
        self.image_data.img = data.img
        self.image_data.metadata = data.metadata
        self.image_data.delay_ms = data.delay_ms
        self.scene.setImage(self.image_data)
        self.scene.clear()
        rgb_image = cv2.cvtColor(data.img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(pixmap_item)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def closeEvent(self, event):
        self.node_obj.set_property("open_window", False)
        return super().closeEvent(event)


class MQTTClientNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Mqtt Client'

    def __init__(self):
        super().__init__()
        # self.add_input('image_data')
        self.add_output('image_data')
        self.image_display_widget = ImageDisplayWidget()
        self.image_display_widget.set_node_obj(self)

        self.add_checkbox("open_window", text='show window')
        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        self.add_text_input("broker", 'Broker',"172.17.0.2")
        self.add_text_input("port", 'Port', '1883')
        self.add_text_input("topic", 'Topic', 'camera/image')

        # 初始化MQTT客户端
        self.mqtt_client = mqtt.Client()
        self.mqtt_client.on_connect = self.on_connect
        self.mqtt_client.on_message = self.on_message
        self.mqtt_client_is_connected = False
        # self.image_queue = Queue(maxsize=10)
        self.count_image_received = 0
        self.image_data = ImageData()

    def on_connect(self, client, userdata, flags, rc):
        """连接到MQTT服务器后的回调函数"""
        if rc == 0:
            print("已成功连接到MQTT服务器")
            # 连接成功后订阅话题
            client.subscribe(self.get_property("topic"))
            print(f"已订阅话题: {self.get_property('topic')}")
        else:
            print(f"连接失败，错误代码: {rc}")
        
        self.mqtt_client_is_connected = True
    
    def on_message(self, client, userdata, msg):
        """收到MQTT消息时的回调函数"""
        try:
            # 分割元数据和图像数据（使用约定的分隔符）
            separator = b'||SEPARATOR||'
            parts = msg.payload.split(separator, 1)
            
            if len(parts) != 2:
                print("❌ 消息格式错误，未找到正确的分隔符")
                return
                
            metadata_bytes, img_bytes = parts
            
            # 解析元数据（包含时间戳）
            metadata = json.loads(metadata_bytes.decode('utf-8'))
            
            # 验证图像数据完整性
            if len(img_bytes) != metadata.get("image_length", 0):
                print(f"❌ 图像数据不完整，预期{metadata['image_length']}字节，实际{len(img_bytes)}字节")
                return
            
            # 解码图像
            nparr = np.frombuffer(img_bytes, np.uint8)
            img = cv2.imdecode(nparr, cv2.IMREAD_COLOR) # np.array
            
            if img is not None:
                # 计算接收延迟（毫秒）
                receive_time_ms = int(time.time() * 1000)
                delay_ms = receive_time_ms - metadata.get("timestamp_ms", receive_time_ms)

                self.image_data.img = img
                self.image_data.metadata = metadata
                self.image_data.delay_ms = delay_ms
                
                # 发送信号到UI线程更新图像
                self.image_display_widget.mqtt_callback_signal.emit(self.image_data)
                
                # 定期打印统计信息
                if hasattr(self, 'count_image_received'):
                    self.count_image_received += 1
                    if self.count_image_received % 10 == 0:
                        print(f"📊 已接收 {self.count_image_received} 帧，最新时间: {metadata['timestamp']}")
                else:
                    self.count_image_received = 1

                if self.count_image_received % 1000000 == 0:
                    self.count_image_received = 0

        except Exception as e:
            print(f"处理图像时出错: {e}")


    def chk_value_changed(self):
        if self.get_property("open_window"):
            self.image_display_widget.show()
        else:
            self.image_display_widget.close()

        print("open_window:", self.get_property("open_window"))

    def execute(self):
        """节点执行函数"""
        if not self.mqtt_client_is_connected:
            # 连接到MQTT服务器
            print(f"连接到MQTT服务器: {self.get_property('broker')}:{self.get_property('port')}")
            self.mqtt_client.connect(self.get_property('broker'), int(self.get_property('port')), 60)

        self.mqtt_client.loop() # 启动一次循环

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def set_widget_parent(self, parent):
        self.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.image_display_widget.close()
        del self.image_display_widget
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        self.image_display_widget.close()
        del self.image_display_widget
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

class SpinMQTTClientNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'Spin Node'

    def __init__(self):
        super().__init__()
        self.add_input('spin_once')
        self.add_checkbox('is_loop', text='is_loop')

    def execute(self):
        """节点执行函数"""
        if not self.get_property("is_loop"):
            return
        else:
            execution_order = get_execution_order(self)[:-1]
            while self.get_property("is_loop"):
                for node in execution_order:
                    if hasattr(node, 'execute'):
                        node.execute() # 运行节点

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

