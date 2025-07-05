#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode, NodeBaseWidget
from Qt import QtCore, QtWidgets
from PySide6.QtCore import *
from PySide6.QtWidgets import *
from PySide6.QtGui import *
import numpy as np
# from .yolov13.ultralytics import YOLO
import os, requests
from utils.general import find_nodes_folder
import cv2
from utils.general import get_execution_order
import time

BASE_DIR = os.path.dirname(os.path.abspath(__file__))
__all__ = ['YOLOV13Node']

class YOLOV13Node(BaseNode, QObject):
    __identifier__ = find_nodes_folder(__file__)[1]
    NODE_NAME = 'yolov13'

    def __init__(self):
        super(YOLOV13Node, self).__init__()
        self.add_input('image_data')
        self.add_output('processed_image_data')

    def execute(self):
        """节点执行函数"""


    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal

    def close_node(self,):
        """整个软件窗体关闭时调用"""

    def _del_node(self):
        """删除节点前调用"""
