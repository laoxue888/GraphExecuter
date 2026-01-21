
from PySide6.QtWidgets import QWidget
from ..ui.ui_dialog import Ui_Form

class ProcessingSettingsDialog(QWidget):
    def __init__(self, parent):
        super().__init__()
        self.setParent(parent)
        self.ui = Ui_Form()
        self.ui.setupUi(self)

    def showUI(self):
        """"""
        self.show()


