#!/usr/bin/env python3
# -*- coding: utf-8 -*-
import os
from PySide6.QtCore import QObject, Signal
from NodeGraphQt import BaseNode, NodeBaseWidget
from utils.general import find_nodes_folder
import numpy as np
import cv2
from PySide6.QtGui import QImage, QPixmap, QPolygonF, QPen, QBrush, QColor
import math
from PySide6.QtWidgets import QGraphicsPixmapItem, QGraphicsScene, QWidget
from PySide6.QtCore import Qt, QPointF
from utils.general import get_execution_order

# ROS
import rclpy
from rclpy.node import Node
from sensor_msgs.msg import Image
from std_msgs.msg import Float64MultiArray
from yolov8_msgs.msg import Yolov8Inference
from cv_bridge import CvBridge

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ('YoloObbNode', "SpinYoloObbNode")

class SpinYoloObbNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'spin_yolo_obb'

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

    def set_widget_parent(self, parent):
        self.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        # self.myui.close()
        # del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        # self.myui.close()
        # del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

class GraphicsScene(QGraphicsScene):
    def __init__(self, parent=None):
        QGraphicsScene.__init__(self, parent)
        self.mouse_x = 0
        self.mouse_y = 0
        self.click_mouse = False

    def mouseMoveEvent(self,event):
        self.mouse_x = event.scenePos().x()
        self.mouse_y = event.scenePos().y()

    def mousePressEvent(self, event):
        self.click_mouse = True


class ImageDisplayWidget(QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    yolo_callback_signal=Signal(Yolov8Inference)

    def __init__(self, parent=None):
        super(ImageDisplayWidget, self).__init__(parent)
        from .ui.ui_yolo_obb import Ui_YoloObbForm

        self.ui = Ui_YoloObbForm()
        self.ui.setupUi(self)
        self.scene = GraphicsScene(self.ui.graphicsView)
        self.ui.graphicsView.setScene(self.scene)
        self.ui.graphicsView.setMouseTracking(True)
        # logging.getLogger().setLevel(logging.WARNING)
        # self.label_img = QLabel(self)
        # self.label_img.setMouseTracking(True)
        # self.ui.verticalLayout.addWidget(self.label_img)
        self.yolo_callback_signal.connect(self.yolo_callback)

        self.bridge = CvBridge()
        self.img = np.zeros([480, 640, 3])

        self.brush = QBrush(QColor(255,255,255,255))
        self.target_point = [0,0,0]
        self.fx = 253.93635749816895
        self.fy = 253.93635749816895
        self.cx = 320
        self.cy = 240
        self.z = 0.7
        self.init_x = 0.2 # camera positon - robot arm link 0 initial position
        self.init_y = 0.6
    
    def yolo_callback(self, data:Yolov8Inference):
        """"""
        # # global img
        self.scene.clear()
        self.img=self.img.astype(np.uint8)  #python类型转换
        rgb_image = cv2.cvtColor(self.img, cv2.COLOR_BGR2RGB)
        height, width, channel = rgb_image.shape
        q_image = QImage(rgb_image.data, width, height, 3 * width, QImage.Format_RGB888)
        pixmap = QPixmap.fromImage(q_image)
        pixmap_item = QGraphicsPixmapItem(pixmap)
        self.scene.addItem(pixmap_item)

        for r in data.yolov8_inference:
            points = np.array(r.coordinates).astype(np.int32).reshape([4, 2])
            middle_point = np.sum(points, 0)/4
            dist = math.sqrt((self.scene.mouse_x - middle_point[0])**2 + (self.scene.mouse_y - middle_point[1])**2)
            qpoly = QPolygonF([QPointF(p[0], p[1]) for p in points])

            if dist < 15:

                self.scene.addPolygon(qpoly, QPen(QColor(255,0,0,255)), QBrush(QColor(255,0,0,100)))   

                if self.scene.click_mouse:

                    self.target_point[0] = -self.z*(middle_point[1] - self.cy)/self.fy + self.init_x
                    self.target_point[1] = -self.z*(middle_point[0] - self.cx)/self.fx + self.init_y
                    dist1 = math.sqrt((points[0][0] - points[1][0])**2 + (points[0][1] - points[1][1])**2)
                    dist2 = math.sqrt((points[1][0] - points[2][0])**2 + (points[1][1] - points[2][1])**2)
                    
                    if(dist1 > dist2):
                        denominator = points[0][0] - points[1][0]
                        if denominator == 0:
                            angle = math.pi/2
                        else:
                            angle = math.atan2(points[0][1] - points[1][1], denominator)
                    else:
                        denominator = points[1][0] - points[2][0]
                        if denominator == 0:
                            angle = math.pi/2
                        else:
                            angle = math.atan2(points[1][1] - points[2][1], denominator)

                    self.target_point[2] = math.pi/2 - angle
                    target_point_pub = Float64MultiArray(data=self.target_point)  
                    self.node_obj.pub.publish(target_point_pub) 
                    self.scene.click_mouse = False
            else:
                self.scene.addPolygon(qpoly, QPen(QColor(0,0,255,255)), QBrush(QColor(0,0,255,100)))  

            self.scene.addEllipse(middle_point[0] - 2, middle_point[1] - 2, 4, 4, QPen(Qt.green), QBrush(Qt.green))     

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def closeEvent(self, event):
        # self.node_obj.set_property("open_window", False)
        return super().closeEvent(event)


class YoloObbNode(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'yolo_obb'

    def __init__(self):
        super().__init__()
        # self.add_input('image_data')
        self.add_output('spin_once')

        # self.myui.scene = GraphicsScene(self.ui.graphicsView)
        # self.ui.graphicsView.setScene(self.myui.scene)
        # self.ui.graphicsView.setMouseTracking(True)
        self.myui=ImageDisplayWidget()
        self.myui.set_node_obj(self)

        self.add_checkbox("open_window", text='show window')
        window_widget = self.get_widget("open_window")
        window_widget.value_changed.connect(self.chk_value_changed)

        self.bridge = CvBridge()
        self.img = np.zeros([480, 640, 3])

        self.is_created_node = False

    def chk_value_changed(self):
        if self.get_property("open_window"):
            self.myui.show()
        else:
            self.myui.close()

        print("open_window:", self.get_property("open_window"))

    def create_ros2_node(self,):
        """"""
        pre = self.__identifier__
        pre = pre.replace('.', '_')
        self.camera_subscriber = Node('{}_{}_image_subscriber'.format(pre, self.NODE_NAME))
        self.sub = self.camera_subscriber.create_subscription(Image, '/image_raw', self.camera_callback, 10)

        self.yolo_subscriber = Node('{}_{}_yolo_subscriber'.format(pre, self.NODE_NAME))
        self.sub = self.yolo_subscriber.create_subscription(Yolov8Inference, '/Yolov8_Inference', self.yolo_callback, 10)

        self.pub_node = Node('{}_{}_pub_path'.format(pre, self.NODE_NAME))
        self.pub = self.pub_node.create_publisher(Float64MultiArray, '/target_point', 10)
        self.is_created_node = True

    def delete_ros2_node(self,):
        """"""
        try:# 先尝试移除节点
            self.camera_subscriber.destroy_node()
            self.yolo_subscriber.destroy_node()
            self.pub_node.destroy_node()
            self.is_created_node = False
        except:
            pass

    def execute(self):
        """节点执行函数"""
        if not self.is_created_node:
            self.create_ros2_node()

        rclpy.spin_once(self.camera_subscriber, timeout_sec=0.1)
        rclpy.spin_once(self.yolo_subscriber, timeout_sec=0.1)

    def camera_callback(self, data):
        self.img = self.bridge.imgmsg_to_cv2(data, "bgr8")
        self.myui.img = self.img.copy()

    def yolo_callback(self, data):
        """"""
        self.myui.yolo_callback_signal.emit(data)
        

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def set_widget_parent(self, parent):
        self.setParent(parent)

    def close_node(self,):
        """整个软件窗体关闭时调用"""
        self.myui.close()
        del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()

    def _del_node(self):
        """删除节点前调用"""
        self.myui.close()
        del self.myui
        # self.video_thread.quit()  # 一定要在这里释放线程
        # self.video_thread.wait()
