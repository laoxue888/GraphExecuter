from Qt import QtCore, QtWidgets
from NodeGraphQt import BaseNode, NodeBaseWidget
from ui.nodes.ui_read_data import Ui_ReadDataForm
import os
from pathlib import Path
from utils.general import getFoldersPath
import time

__all__=['ReadLargeTxtDataNode']

BASE_PATH = Path(__file__).parent.resolve()

class ReadDataForm(QtWidgets.QWidget):
    def __init__(self,):
        super().__init__()
        self.ui = Ui_ReadDataForm()
        self.ui.setupUi(self)

class MyCustomWidget(QtWidgets.QWidget):
    """
    Custom widget to be embedded inside a node.
    """
    def __init__(self, parent=None):
        super(MyCustomWidget, self).__init__(parent)
        self.read_data_form = ReadDataForm()
        layout = QtWidgets.QHBoxLayout(self)
        layout.setContentsMargins(0, 0, 0, 0)
        layout.addWidget(self.read_data_form)

        self.read_data_form.ui.pushButton_open_folder.clicked.connect(self.open_folder)

    def set_node_obj(self, obj):
        """"""
        self.node_obj = obj

    def open_folder(self,):
        """"""
        if self.node_obj.get_property('folder_path').strip():
            file_path = getFoldersPath(self.node_obj.get_property('folder_path').strip())
        else:
            file_path = getFoldersPath('/')
        if file_path:
            self.node_obj.set_property('folder_path', file_path[0])

class NodeWidgetWrapper(NodeBaseWidget):
    """
    Wrapper that allows the widget to be added in a node object.
    """

    def __init__(self, parent=None):
        super(NodeWidgetWrapper, self).__init__(parent)

        # set the name for node property.
        self.set_name('my_widget')
        # set the custom widget.
        self.set_custom_widget(MyCustomWidget())
        # connect up the signals & slots.
        self.wire_signals()

    def wire_signals(self):
        """"""
        widget = self.get_custom_widget()
        # wire up the button.
        # widget.btn_go.clicked.connect(self.on_btn_go_clicked)

    def on_btn_go_clicked(self):
        """"""
        # print('Clicked on node: "{}"'.format(self.node.name()))

    def get_value(self):
        """"""

    def set_value(self, value):
        """"""

class ReadLargeTxtDataNode(BaseNode):
    """
    Example node.
    """
    # set a unique node identifier.
    __identifier__ = 'nodes.read.data'
    # set the initial default node name.
    NODE_NAME = 'Read large txt data node'
    def __init__(self):
        super(ReadLargeTxtDataNode, self).__init__()

        # create input and output port.
        # self.add_input('')
        self.add_output('Output')

        # add custom widget to node with "node.view" as the parent.
        node_widget = NodeWidgetWrapper(self.view)
        node_widget.get_custom_widget().set_node_obj(self)
        self.add_custom_widget(node_widget, tab='Custom')
        self.set_icon(os.path.join(BASE_PATH, 'icon', 'star.png'))
        self.add_text_input('folder_path', '', tab='widgets')

    def execute(self):
        return "ReadLargeTxtDataNode executed"

    def get_value(self):
        return self.execute()


