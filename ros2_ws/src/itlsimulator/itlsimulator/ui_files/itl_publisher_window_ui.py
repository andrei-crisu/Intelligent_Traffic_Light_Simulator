# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'c:\Users\crisu\Desktop\RC_App_ROS2\workspace2\ros2_ws\src\itlsimulator\itlsimulator\ui_files\itl_publisher_window.ui'
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
        Widget.resize(735, 551)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(Widget.sizePolicy().hasHeightForWidth())
        Widget.setSizePolicy(sizePolicy)
        self.gridLayout = QtWidgets.QGridLayout(Widget)
        self.gridLayout.setObjectName("gridLayout")
        self.splitter = QtWidgets.QSplitter(Widget)
        self.splitter.setOrientation(QtCore.Qt.Horizontal)
        self.splitter.setChildrenCollapsible(False)
        self.splitter.setObjectName("splitter")
        self.groupBox = QtWidgets.QGroupBox(self.splitter)
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
        font = QtGui.QFont()
        font.setPointSize(14)
        self.textEdit.setFont(font)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_3.addWidget(self.textEdit)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.sendMessageButton = QtWidgets.QPushButton(self.groupBox_2)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sendMessageButton.sizePolicy().hasHeightForWidth())
        self.sendMessageButton.setSizePolicy(sizePolicy)
        self.sendMessageButton.setObjectName("sendMessageButton")
        self.horizontalLayout_2.addWidget(self.sendMessageButton)
        spacerItem1 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addWidget(self.groupBox_2)
        self.groupBox_4 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_4)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        spacerItem2 = QtWidgets.QSpacerItem(20, 240, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem2)
        self.horizontalLayout.addWidget(self.groupBox_4)
        spacerItem3 = QtWidgets.QSpacerItem(80, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem3)
        self.itlGroupBox = QtWidgets.QGroupBox(self.splitter)
        self.itlGroupBox.setMinimumSize(QtCore.QSize(300, 500))
        self.itlGroupBox.setTitle("")
        self.itlGroupBox.setObjectName("itlGroupBox")
        self.verticalLayout_5 = QtWidgets.QVBoxLayout(self.itlGroupBox)
        self.verticalLayout_5.setObjectName("verticalLayout_5")
        self.itlTrafficLightBox = QtWidgets.QGroupBox(self.itlGroupBox)
        self.itlTrafficLightBox.setMinimumSize(QtCore.QSize(0, 100))
        self.itlTrafficLightBox.setTitle("")
        self.itlTrafficLightBox.setObjectName("itlTrafficLightBox")
        self.verticalLayout_7 = QtWidgets.QVBoxLayout(self.itlTrafficLightBox)
        self.verticalLayout_7.setObjectName("verticalLayout_7")
        self.widgetITL = QtWidgets.QWidget(self.itlTrafficLightBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.widgetITL.sizePolicy().hasHeightForWidth())
        self.widgetITL.setSizePolicy(sizePolicy)
        self.widgetITL.setMinimumSize(QtCore.QSize(0, 240))
        self.widgetITL.setObjectName("widgetITL")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widgetITL)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout_7.addWidget(self.widgetITL)
        self.verticalLayout_5.addWidget(self.itlTrafficLightBox)
        self.controlsBox = QtWidgets.QGroupBox(self.itlGroupBox)
        self.controlsBox.setMinimumSize(QtCore.QSize(0, 30))
        self.controlsBox.setTitle("")
        self.controlsBox.setObjectName("controlsBox")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.controlsBox)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem4)
        self.sendMessageButton_2 = QtWidgets.QPushButton(self.controlsBox)
        self.sendMessageButton_2.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sendMessageButton_2.sizePolicy().hasHeightForWidth())
        self.sendMessageButton_2.setSizePolicy(sizePolicy)
        self.sendMessageButton_2.setObjectName("sendMessageButton_2")
        self.horizontalLayout_3.addWidget(self.sendMessageButton_2)
        self.sendMessageButton_3 = QtWidgets.QPushButton(self.controlsBox)
        self.sendMessageButton_3.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.sendMessageButton_3.sizePolicy().hasHeightForWidth())
        self.sendMessageButton_3.setSizePolicy(sizePolicy)
        self.sendMessageButton_3.setObjectName("sendMessageButton_3")
        self.horizontalLayout_3.addWidget(self.sendMessageButton_3)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem6)
        self.verticalLayout_6.addLayout(self.verticalLayout)
        self.verticalLayout_5.addWidget(self.controlsBox)
        self.gridLayout.addWidget(self.splitter, 0, 0, 1, 1)

        self.retranslateUi(Widget)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    def retranslateUi(self, Widget):
        _translate = QtCore.QCoreApplication.translate
        Widget.setWindowTitle(_translate("Widget", "Widget"))
        self.mainLabel.setText(_translate("Widget", "Mesaj:"))
        self.sendMessageButton.setText(_translate("Widget", "publish"))
        self.sendMessageButton_2.setText(_translate("Widget", "---"))
        self.sendMessageButton_3.setText(_translate("Widget", "---"))
