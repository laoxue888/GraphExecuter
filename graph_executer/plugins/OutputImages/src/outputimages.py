
from .settingsdialog import ProcessingSettingsDialog
from PySide6.QtCore import Signal, QThread
from ...shared import *
import os,json


class OutputImages(ProcessingPlugin):

    def __init__(self):
        super().__init__()

        # 插件类型
        self.pluginType = PluginType.Processing

        # 插件名称
        self.pluginName = type(self).__name__

        # 插件描述
        self.pluginDescription = "It is a RemoveBackgroundNoise"

        # json_path = os.path.join(os.path.dirname(os.path.dirname(os.path.dirname(os.path.abspath(__file__)))), "plugin_settings.json")
        # with open(json_path, 'r') as f:
        #     data_json = json.load(f)

        # # 插件顺序
        # self.pluginOrder = data_json[self.__class__.__name__]["order"]  # 从0开始

        # self.background_noise_remove = BackgroundNoiseRemove()

    def initGui(self, parent):
        """"""
        self.setParent(parent)
        self.window = ProcessingSettingsDialog(self)

    def getWindow(self):
        return self.window

    def processingData(self, data: any, meta: dict):
        """CPU或者GPU处理数据，只处理一个buffer
            data: (B, H, W)
        """
        # self.buffer = data
        # print(f'OutputImages buffer shape: {self.buffer.shape}')

        # data2Dfft = torch.abs(torch.fft.fft(self.buffer, dim=2))
        # # bscan = data2Dfft[:, :, 0:self.num_aline_points // 2]
        # bscan = data2Dfft
        # imageCal = 20 * torch.log10(bscan * bscan)
        # imageCal[imageCal > 255] = 255
        # imageCal[imageCal < 0] = 0
        # self.buffer = imageCal

        # return self.buffer, meta

    def showSettingsDialog(self):
        try:
            self.window.close()
            self.window.showUI()
        except:
            pass


# 下列代码必须加上，用于getattr识别
OutputImages = OutputImages