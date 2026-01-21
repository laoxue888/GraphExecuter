import importlib
from PySide6.QtWidgets import QWidget
from PySide6.QtWidgets import QMessageBox

class PluginManager(QWidget):
    def __init__(self):
        """初始化插件管理器，设置插件字典容器"""
        super().__init__()
        self.plugins = {}
    def pluginExist(self, pluginName) -> bool:
        """检查指定插件是否存在，不存在则显示错误提示"""
        if pluginName in self.plugins.keys():
            return True
        else:
            QMessageBox.critical(self, "error", f"No {pluginName} plugin")
            return False

    def usePlugin(self, pluginName):
        """获取指定插件实例"""
        return self.plugins[pluginName]

    def loadPlugin(self, module_name, class_name):
        """动态加载指定模块和类名的插件"""
        try:
            module = importlib.import_module(module_name)
            plugin_class = getattr(module, class_name)
            print(f'plugin_class: {plugin_class}')
            plugin = plugin_class() #初始化了插件
            self.plugins[class_name] = plugin
            print(f"Completed loading the {class_name} plugin")
            return True
        except Exception as err:
            print(f"Could not load plugin {class_name} from module {module_name}, because {err}")
            return False

    def loadPluginWithParent(self, module_name, class_name, parent):
        """动态加载指定模块和类名的插件，并设置父组件"""
        try:
            module = importlib.import_module(module_name)
            plugin_class = getattr(module, class_name)
            print(f'plugin_class: {plugin_class}')
            plugin = plugin_class(parent) #初始化了插件
            self.plugins[class_name] = plugin
            print(f"Completed loading the {class_name} plugin")
            return True
        except Exception as err:
            print(f"Could not load plugin {class_name} from module {module_name}, because {err}")
            return False

    def unregisterPlugin(self, name):
        """注销指定名称的插件"""
        if name in self.plugins:
            del self.plugins[name]

    def runPlugins(self):
        """运行所有已加载的插件"""
        for plugin in self.plugins.values():
            plugin.run()

