import queue
from enum import Enum
from PySide6.QtCore import QObject, Signal, Slot
from PySide6.QtWidgets import QWidget
from abc import abstractmethod
import numpy as np
import gc


class PluginType(Enum):
    Acquisition = 0
    Processing = 1
    Display = 2
    Extra = 3



class Parameters:
    def __init__(self):
        """所有的设置参数，用于插件传递给主程序"""
        # Acquisition相关的设置
        self.ascan = 0
        self.bscan = 0
        self.bscanNum = 0
        self.bufferNum = 0
        # Processing相关的设置

        # Display相关的设置


class DataBuffer:
    """
    每次采集一个缓存
    """
    def __init__(self):
        """"""
        self.ascan = -1
        self.bscan = -1
        self.bscanNum = -1
        self.bufferNum = -1
        self.cunrentBuffer = 0
        self.bscanCurrentInex = 0
        self.volume = None # 一个volume中存在多个buffer

    def nextBuffer(self):
        """"""
        self.cunrentBuffer += 1
        if self.cunrentBuffer > self.bufferNum:
            self.cunrentBuffer = 0

    def allocateMemory(self):
        """分配存储空间"""
        self.releaseMemory()
        assert (self.ascan > 0 and self.bscan > 0 and self.bscanNum > 0 and self.bufferNum > 0), "缓冲大小无法确定"
        self.cunrentBuffer = 0 # 重置
        self.volume = np.zeros((self.bufferNum * self.bscanNum, self.bscan, self.ascan), dtype=int)

    def releaseMemory(self):
        """释放存储空间"""
        del self.volume
        gc.collect()

    def getCurrentBuffer(self, bufferIndex):
        """获取缓存大小"""
        assert 0 <= bufferIndex < self.bufferNum, "超出数组索引范围"
        buffer = self.volume[bufferIndex * self.bscanNum:(bufferIndex + 1) * self.bscanNum, :, :]
        return buffer

class Shared(QWidget):
    pluginMessage = Signal(str)
    def __init__(self):
        """"""
        super().__init__()
        # 插件类型
        self.pluginType = None
        # 插件名称
        self.pluginName = None

    @abstractmethod
    def startAcquisition(self, pluginName):
        """"""

    @abstractmethod
    def stopAcquisition(self, pluginName):
        """"""

    @abstractmethod
    def recordAcquisition(self, flag):
        """保存数据"""

class AcquisitionPlugin(Shared):

    finishedAcquisitionOneFrame = Signal(DataBuffer)
    acquisitionParametersChanged = Signal(Parameters)

    def __init__(self):
        super().__init__()
        """"""
        self.dataBuffer = DataBuffer()

    def setAcquisitionQueue(self, acquisitionQueue: queue.Queue):
        """获取主程序定义的采集队列"""
        self.acquisitionQueue: queue.Queue = acquisitionQueue

class ProcessingPlugin(Shared):
    def __init__(self):
        super().__init__()
        """"""

        # 默认顺序，因为从上往下的
        self.pluginOrder = -1

    @abstractmethod
    def processingData(self, data: any, meta: dict):
        """CPU或者GPU处理数据"""

    @abstractmethod
    def getWindow(self)->any:
        """"""


class DisplayPlugin(Shared):

    def __init__(self):
        super().__init__()

    @abstractmethod
    def finishedProcessOneBufferData(self):
        """"""

    @abstractmethod
    def setDataVolume(self, dataVolumeAddress):
        """"""

    @abstractmethod
    def getWindow(self) -> any:
        """"""
