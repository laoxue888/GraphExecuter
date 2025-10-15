# -*- coding: utf-8 -*-

################################################################################
## Form generated from reading UI file 'yolo_obb.ui'
##
## Created by: Qt User Interface Compiler version 6.8.3
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
from PySide6.QtWidgets import (QApplication, QGraphicsView, QSizePolicy, QVBoxLayout,
    QWidget)

class Ui_YoloObbForm(object):
    def setupUi(self, YoloObbForm):
        if not YoloObbForm.objectName():
            YoloObbForm.setObjectName(u"YoloObbForm")
        YoloObbForm.resize(400, 300)
        self.verticalLayout = QVBoxLayout(YoloObbForm)
        self.verticalLayout.setObjectName(u"verticalLayout")
        self.graphicsView = QGraphicsView(YoloObbForm)
        self.graphicsView.setObjectName(u"graphicsView")

        self.verticalLayout.addWidget(self.graphicsView)


        self.retranslateUi(YoloObbForm)

        QMetaObject.connectSlotsByName(YoloObbForm)
    # setupUi

    def retranslateUi(self, YoloObbForm):
        YoloObbForm.setWindowTitle(QCoreApplication.translate("YoloObbForm", u"Form", None))
    # retranslateUi

