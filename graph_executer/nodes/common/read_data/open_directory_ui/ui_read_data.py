# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'read_data.ui'
##
## Created by: Qt User Interface Compiler version 6.9.1
##
## WARNING! All changes made in this file will be lost when recompiling UI file!
################################################################################

from PySide6.QtCore import (QCoreApplication, QDate, QDateTime, QLocale,
    QMetaObject, QObject, QPoint, QRect,
    QSize, QTime, QUrl, Qt)
from PySide6.QtGui import (QBrush, QColor, QConicalGradient, QCursor,
    QFont, QFontDatabase, QGradient, QIcon,
    QImage, QKeySequence, QLinearGradient, QPainter,
    QPalette, QPixmap, QRadialGradient, QTransform)
from PySide6.QtWidgets import (QApplication, QGridLayout, QPushButton, QSizePolicy,
    QWidget)

class Ui_OpenDirectoryForm(object):
    def setupUi(self, OpenDirectoryForm):
        if not OpenDirectoryForm.objectName():
            OpenDirectoryForm.setObjectName(u"OpenDirectoryForm")
        OpenDirectoryForm.resize(228, 24)
        self.gridLayout = QGridLayout(OpenDirectoryForm)
        self.gridLayout.setSpacing(0)
        self.gridLayout.setObjectName(u"gridLayout")
        self.gridLayout.setContentsMargins(0, 0, 0, 0)
        self.pushButton_open_folder = QPushButton(OpenDirectoryForm)
        self.pushButton_open_folder.setObjectName(u"pushButton_open_folder")

        self.gridLayout.addWidget(self.pushButton_open_folder, 0, 0, 1, 1)


        self.retranslateUi(OpenDirectoryForm)

        QMetaObject.connectSlotsByName(OpenDirectoryForm)
    # setupUi

    def retranslateUi(self, OpenDirectoryForm):
        OpenDirectoryForm.setWindowTitle(QCoreApplication.translate("OpenDirectoryForm", u"Form", None))
        self.pushButton_open_folder.setText(QCoreApplication.translate("OpenDirectoryForm", u"Open folder", None))
    # retranslateUi

