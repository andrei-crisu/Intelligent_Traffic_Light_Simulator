# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'maindesign.ui'
#
# Created by: PyQt5 UI code generator 5.15.9
#
# WARNING: Any manual changes made to this file will be lost when pyuic5 is
# run again.  Do not edit this file unless you know what you are doing.


from PyQt5 import QtCore, QtGui, QtWidgets


class Ui_Widget(object):
    def setupUi(self, Widget):
        Widget.setObjectName("Widget")
        Widget.setEnabled(True)
        Widget.resize(800, 600)
        self.groupBox = QtWidgets.QGroupBox(Widget)
        self.groupBox.setGeometry(QtCore.QRect(9, 9, 524, 456))
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox.sizePolicy().hasHeightForWidth())
        self.groupBox.setSizePolicy(sizePolicy)
        self.groupBox.setTitle("")
        self.groupBox.setObjectName("groupBox")
        self.horizontalLayout = QtWidgets.QHBoxLayout(self.groupBox)
        self.horizontalLayout.setObjectName("horizontalLayout")
        self.groupBox_2 = QtWidgets.QGroupBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.groupBox_2.sizePolicy().hasHeightForWidth())
        self.groupBox_2.setSizePolicy(sizePolicy)
        self.groupBox_2.setTitle("")
        self.groupBox_2.setObjectName("groupBox_2")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.groupBox_2)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.mainLabel = QtWidgets.QLabel(self.groupBox_2)
        font = QtGui.QFont()
        font.setPointSize(18)
        self.mainLabel.setFont(font)
        self.mainLabel.setObjectName("mainLabel")
        self.verticalLayout_3.addWidget(self.mainLabel)
        self.textEdit = QtWidgets.QTextEdit(self.groupBox_2)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_3.addWidget(self.textEdit)
        self.horizontalLayout.addWidget(self.groupBox_2)
        self.groupBox_4 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_4)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        self.groupBox_3 = QtWidgets.QGroupBox(self.groupBox_4)
        self.groupBox_3.setTitle("")
        self.groupBox_3.setObjectName("groupBox_3")
        self.verticalLayout_2 = QtWidgets.QVBoxLayout(self.groupBox_3)
        self.verticalLayout_2.setObjectName("verticalLayout_2")
        self.Button_1 = QtWidgets.QPushButton(self.groupBox_3)
        self.Button_1.setObjectName("Button_1")
        self.verticalLayout_2.addWidget(self.Button_1)
        self.Button_2 = QtWidgets.QPushButton(self.groupBox_3)
        self.Button_2.setObjectName("Button_2")
        self.verticalLayout_2.addWidget(self.Button_2)
        self.Button_3 = QtWidgets.QPushButton(self.groupBox_3)
        self.Button_3.setObjectName("Button_3")
        self.verticalLayout_2.addWidget(self.Button_3)
        self.Button_4 = QtWidgets.QPushButton(self.groupBox_3)
        self.Button_4.setObjectName("Button_4")
        self.verticalLayout_2.addWidget(self.Button_4)
        self.Button_5 = QtWidgets.QPushButton(self.groupBox_3)
        self.Button_5.setObjectName("Button_5")
        self.verticalLayout_2.addWidget(self.Button_5)
        self.verticalLayout_4.addWidget(self.groupBox_3)
        spacerItem = QtWidgets.QSpacerItem(20, 240, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem)
        self.horizontalLayout.addWidget(self.groupBox_4)
        spacerItem1 = QtWidgets.QSpacerItem(80, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem1)

        self.retranslateUi(Widget)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    def retranslateUi(self, Widget):
        _translate = QtCore.QCoreApplication.translate
        Widget.setWindowTitle(_translate("Widget", "Widget"))
        self.mainLabel.setText(_translate("Widget", "Mesaj receptionat"))
        self.Button_1.setText(_translate("Widget", "PushButton"))
        self.Button_2.setText(_translate("Widget", "PushButton"))
        self.Button_3.setText(_translate("Widget", "PushButton"))
        self.Button_4.setText(_translate("Widget", "PushButton"))
        self.Button_5.setText(_translate("Widget", "PushButton"))
