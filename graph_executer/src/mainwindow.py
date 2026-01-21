# coding=utf-8
from ui.ui_mainwindow import Ui_MainWindow
from PySide6.QtWidgets import QMainWindow, QApplication, QSizePolicy, QDockWidget, QWidget, QHBoxLayout
from PySide6.QtCore import QSettings, QObject, QEventLoop, QTimer
from utils.general import *
from  src.messageconsole import MessageConsole
from PySide6.QtCore import Signal, Slot, QSize, Qt
import datetime
from PySide6.QtGui import QIcon, QTextCursor
from src.updatelog import UpdateLog
from collections import OrderedDict
from src.GraphFlow import GraphFlow
import webbrowser
import json
from PySide6 import QtCore
from src.plugin_manager import PluginManager

BASE_DIR = os.path.dirname(os.path.dirname(os.path.abspath(__file__)))

class MainWindow(QMainWindow, QObject):
    messageSignal = Signal(str)

    def __init__(self, parent=None):
        super().__init__(parent)

        self.ui = Ui_MainWindow()
        self.ui.setupUi(self)

        self.setDockNestingEnabled(True)  # DockWidget可以重叠
        self.ui.centralwidget.setMaximumWidth(0.1) # 缩小中间控件宽度，从而使得dockwidget可以充满整个窗口

        self.userSettings = OrderedDict()
        # save custom data format: 0: data；1: type, like as str、int、float etc
        self.userSettings['graph_session'] = ['', 'str']
        self.settings_dir = os.path.join(BASE_DIR, "settings")

        self.loadSettings()

        self.setWindowIcon(QIcon(os.path.join(BASE_DIR, "settings", "myicon.png")))
        self.setIconSize(QSize(22, 22))

        self.updatelog = UpdateLog(self)
        self.ui.actionWindow1.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(0))
        self.ui.actionWindow2.triggered.connect(lambda: self.ui.stackedWidget.setCurrentIndex(1))

        self.messageconsole = MessageConsole()
        self.messageSignal.connect(self.messageconsole.showMessage)
        # self.ui.verticalLayout_msg.addWidget(self.messageconsole)
        self.messageSignal.emit("Start time: {}".format(datetime.datetime.now()))

        self.dockConsole = QDockWidget("Message Console", self)
        self.dockConsole.setObjectName("Message Console")
        self.dockConsole.setFloating(False)
        self.dockConsole.setVisible(True)
        # self.messageconsole = MessageConsole()
        self.dockConsole.setWidget(self.messageconsole)
        self.messageconsole.setVisible(True)
        self.addDockWidget(Qt.DockWidgetArea.BottomDockWidgetArea, self.dockConsole)
        self.dockConsole.setTitleBarWidget(QWidget())
        self.dockConsole.setMinimumWidth(300)

        # 插件管理器
        self.dockWidgets = {}
        self.dockWidgetsList = []
        self.pluginManager = PluginManager()
        # self.loadPlugins()

        self.initGui()
        self.loadBasicPlugins()

    def makeDockWidgetForPlugins(self, plugin):
        """"""
        dockWidget = QDockWidget(plugin.pluginName)
        # dockWidget.setParent(self)
        dockWidget.setObjectName(plugin.pluginName)
        # dockWidget.setVisible(False)
        # dockWidget.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetClosable | QDockWidget.DockWidgetFloatable)
        # dockWidget.setFeatures(QDockWidget.DockWidgetMovable | QDockWidget.DockWidgetFloatable | QDockWidget.DockWidgetFeature.DockWidgetClosable)

        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Preferred)
        dockWidget.setSizePolicy(sizePolicy)

        # 初始化插件GUI
        widget = QWidget(dockWidget)
        widget.setContentsMargins(0, 0, 0, 0)
        plugin.initGui(widget)
        layout = QHBoxLayout(widget)
        # dockWidget.setLayout(self.layout)
        # dockWidget.setWidget(plugin.getWindow())  # 一定要将ui类放入，而不是插件的整个类
        layout.addWidget(plugin.getWindow())
        layout.setContentsMargins(0,0,0,0)
        dockWidget.setWidget(widget)
        widget.setVisible(True)

        dockWidget.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        dockWidget.setMinimumWidth(400)
        dockWidget.setMinimumHeight(300)

        if len(self.dockWidgetsList) > 0:
            self.tabifyDockWidget(self.dockWidgetsList[len(self.dockWidgetsList) - 1], dockWidget)
        else:
            self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, dockWidget, Qt.Orientation.Horizontal)

        dockWidget.setVisible(True)

        self.dockWidgets[plugin.pluginName] = dockWidget
        self.dockWidgetsList.append(dockWidget)
    
    def loadBasicPlugins(self):
        """加载插件"""
        # module_path = "plugins.OutputImages"
        # pluginName = "OutputImages"
        # self.pluginManager.loadPlugin(module_path, pluginName)
        # self.makeDockWidgetForPlugins(self.pluginManager.usePlugin(pluginName))

    def initGui(self, ):
        """"""
        self.graph = GraphFlow(self.messageSignal, self)

        self.graph_path = os.path.join(BASE_DIR, "res", "graphs", "process_graph.json")
        try:
            self.graph.graph.load_session(self.graph_path)
        except Exception as err:
            self.messageSignal.emit(err)
            try:
                os.remove(self.graph_path)
                self.graph._model.session = self.graph_path
            except Exception as err:
                pass

        self.dock_graph = QDockWidget("Graph", self)
        self.dock_graph.setObjectName("Graph")
        sizePolicy = QSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        self.dock_graph.setSizePolicy(sizePolicy)
        # self.dockConsole.setFloating(False)
        self.dock_graph.setVisible(True)
        self.dock_graph.setWidget(self.graph.graph_widget)

        self.graph.graph_widget.setVisible(True)
        self.dock_graph.setAllowedAreas(Qt.DockWidgetArea.LeftDockWidgetArea | Qt.DockWidgetArea.RightDockWidgetArea)
        self.addDockWidget(Qt.DockWidgetArea.RightDockWidgetArea, self.dock_graph, Qt.Orientation.Horizontal)
        self.dock_graph.setMinimumWidth(300)

        self.ui.actionexecute_graph.triggered.connect(self.graph.execute_all_nodes)
        self.ui.actionexetute_from_goal_node.triggered.connect(self.graph.execute_selected_nodes)

        self.ui.actionexetute_from_goal_node.setIcon(
            QIcon(os.path.join(BASE_DIR, "settings", "BtnIcon","from_obj_node.png")))
        self.ui.actionexecute_graph.setIcon(
            QIcon(os.path.join(BASE_DIR, "settings", "BtnIcon", "all_graph.png")))

        def openLogDir():
            os.startfile(os.path.dirname(self.messageconsole.logPath))
        self.ui.actionOpen_log_dir.triggered.connect(openLogDir)

        # 打开用户教程
        def openUserGuide():
            file_path = os.path.join(BASE_DIR, 'docs/userguide/site/index.html')
            webbrowser.open(file_path)
        self.ui.actionUserGuide.triggered.connect(openUserGuide)

        def openUpdateLog():
            self.updatelog.show()
        self.ui.actionUpdate_log.triggered.connect(openUpdateLog)

        # change ui display language
        with open(os.path.join(BASE_DIR, "settings", "languages", "en.json"), "r", encoding="utf-8") as f:
            en_data = json.load(f)
            self.ui.menuHelp.setTitle(en_data["main_window"]["menuHelp"])
            self.ui.actionUpdate_log.setText(en_data["main_window"]["actionUpdate_log"])
            self.ui.actionUserGuide.setText(en_data["main_window"]["actionUserGuide"])
            self.ui.actionOpen_log_dir.setText(en_data["main_window"]["actionOpen_log_dir"])
            self.ui.menuWindow.setTitle(en_data["main_window"]["menuWindow"])
            self.ui.menuTools.setTitle(en_data["main_window"]["menuTools"])
            self.ui.actionexetute_from_goal_node.setText(en_data["main_window"]["actionexetute_from_goal_node"])
            self.ui.actionexecute_graph.setText(en_data["main_window"]["actionexecute_graph"])
            self.ui.actionWindow1.setText(en_data["main_window"]["actionWindow1"])
            self.ui.actionWindow2.setText(en_data["main_window"]["actionWindow2"])

    def saveSettings(self):
        """"""
        settingsPath = os.path.join(self.settings_dir, self.__class__.__name__, "UserSettings.conf")
        if os.path.exists(settingsPath):
            settings = QSettings(settingsPath, QSettings.NativeFormat)
        else:
            settings = QSettings(settingsPath, QSettings.NativeFormat)

        settings.beginGroup("user_settings")
        for key, value in self.userSettings.items():
            print(key, self.userSettings[key][0])
            settings.setValue(key, self.userSettings[key][0])

        settings.endGroup()
        settings.sync()  # 同步

    def loadSettings(self):
        """"""
        try:
            settingsPath = os.path.join(self.settings_dir, self.__class__.__name__, "UserSettings.conf")
            settings = QSettings(settingsPath, QSettings.NativeFormat)
            for key, value in self.userSettings.items():
                if value[1]=='bool':
                    self.userSettings[key][0] = True if settings.value("user_settings/{}".format(key))=='true' else False
                else:
                    self.userSettings[key][0] = eval(value[1])(settings.value("user_settings/{}".format(key)))
        except Exception as err:
            self.messageSignal.emit(err)

    def closeEvent(self, event):
        # self.userSettings['graph_session'][0] = self.graph.graph.current_session()
        self.saveSettings()
        self.messageSignal.emit("Close time: {}".format(datetime.datetime.now()))
        # self.graph.save_session()
        self.graph.graph.save_session(self.graph_path)
        self.graph.close_event()
        return super().closeEvent(event)



        




