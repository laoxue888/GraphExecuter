#!/usr/bin/env python3
# -*- coding: utf-8 -*-
from NodeGraphQt import BaseNode
import speech_recognition as sr
import pyttsx3

__all__ = ['SpeechRecognitionNode', 'SpeechSpeakNode']

class SpeechRecognitionNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Speech Recognition Node'

    def __init__(self):
        super(SpeechRecognitionNode, self).__init__()
        # 初始化语音识别
        self.recognizer = sr.Recognizer()
        self.microphone = sr.Microphone()
        self.add_output('text')
        self.text = ""

    def execute(self):
        with self.microphone as source:
            self.messageSignal.emit(f"{self.name()} 请说话...")
            self.recognizer.adjust_for_ambient_noise(source)  # 降噪
            audio = self.recognizer.listen(source)
        try:
            text = self.recognizer.recognize_whisper(audio, language='zh')  # 中文识别
            self.messageSignal.emit(f"{self.name()} Output result: {text}")
            self.text = text
            return text
        except sr.UnknownValueError:
            self.messageSignal.emit(f"{self.name()} Output result: 无法识别语音")
            return ""
        except sr.RequestError:
            self.messageSignal.emit(f"{self.name()} Output result: 语音服务不可用")
            return ""

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal


class SpeechSpeakNode(BaseNode):
    """打印节点，输出结果"""
    __identifier__ = 'nodes.speech'
    NODE_NAME = 'Speak Node'

    def __init__(self):
        super(SpeechSpeakNode, self).__init__()
        # 初始化语音引擎
        self.engine = pyttsx3.init()
        # 设置语音属性（可选）
        voices = self.engine.getProperty('voices')
        self.engine.setProperty('voice', voices[0].id)  # 0通常是英文男声，1可能是英文女声，中文可能需要其他设置
        self.engine.setProperty('rate', 150)  # 语速
        self.add_input('text')

        self.add_checkbox('isConversationLoop', text='是否开启对话循环')

    def execute(self):
        """"""
        text = self.input(0).connected_ports()[0].node().text
        self.messageSignal.emit(f"{self.name()} Output result: {text}")
        self.engine.say(text)
        self.engine.runAndWait()

        if self.get_property('isConversationLoop'):
            # 遍历前置节点，并保存到一个列表中
            self.get_execution_order(self) # 遍历前置节点，再执行它们

    def get_execution_order(self, obj_node):
        """获取从指定节点开始的下游节点执行顺序（拓扑排序）"""
        visited = set()
        # execution_order = []

        def visit_up(node):
            if node in visited:
                return
            # 首先处理所有上游节点
            for port in node.inputs().values():
                for connected_port in port.connected_ports():
                    visit_up(connected_port.node())
            visited.add(node)
            node.set_messageSignal(self.messageSignal)
            if hasattr(node, 'execute'):
                 node.execute() # 运行节点
        visit_up(obj_node)

    def set_messageSignal(self, messageSignal):
        self.messageSignal = messageSignal