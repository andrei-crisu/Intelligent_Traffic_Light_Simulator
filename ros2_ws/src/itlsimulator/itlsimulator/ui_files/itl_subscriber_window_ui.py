# -*- coding: utf-8 -*-

# Form implementation generated from reading ui file 'c:\Users\crisu\Desktop\RC_App_ROS2\trafficSimulator_rc_project\ros2_ws\src\itlsimulator\itlsimulator\ui_files\itl_subscriber_window.ui'
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
        self.displayMessageBox = QtWidgets.QGroupBox(self.groupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Expanding)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.displayMessageBox.sizePolicy().hasHeightForWidth())
        self.displayMessageBox.setSizePolicy(sizePolicy)
        self.displayMessageBox.setTitle("")
        self.displayMessageBox.setObjectName("displayMessageBox")
        self.verticalLayout_3 = QtWidgets.QVBoxLayout(self.displayMessageBox)
        self.verticalLayout_3.setObjectName("verticalLayout_3")
        self.mainLabel = QtWidgets.QLabel(self.displayMessageBox)
        font = QtGui.QFont()
        font.setPointSize(18)
        self.mainLabel.setFont(font)
        self.mainLabel.setObjectName("mainLabel")
        self.verticalLayout_3.addWidget(self.mainLabel)
        self.textEdit = QtWidgets.QTextEdit(self.displayMessageBox)
        font = QtGui.QFont()
        font.setPointSize(14)
        self.textEdit.setFont(font)
        self.textEdit.setUndoRedoEnabled(False)
        self.textEdit.setReadOnly(True)
        self.textEdit.setObjectName("textEdit")
        self.verticalLayout_3.addWidget(self.textEdit)
        self.comStatusBox = QtWidgets.QGroupBox(self.displayMessageBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.comStatusBox.sizePolicy().hasHeightForWidth())
        self.comStatusBox.setSizePolicy(sizePolicy)
        self.comStatusBox.setMinimumSize(QtCore.QSize(0, 30))
        self.comStatusBox.setTitle("")
        self.comStatusBox.setObjectName("comStatusBox")
        self.horizontalLayout_5 = QtWidgets.QHBoxLayout(self.comStatusBox)
        self.horizontalLayout_5.setObjectName("horizontalLayout_5")
        self.statusCheckBox = QtWidgets.QCheckBox(self.comStatusBox)
        self.statusCheckBox.setEnabled(False)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.statusCheckBox.setFont(font)
        self.statusCheckBox.setCheckable(True)
        self.statusCheckBox.setChecked(True)
        self.statusCheckBox.setTristate(False)
        self.statusCheckBox.setObjectName("statusCheckBox")
        self.horizontalLayout_5.addWidget(self.statusCheckBox)
        self.verticalLayout_3.addWidget(self.comStatusBox)
        self.horizontalLayout_2 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_2.setObjectName("horizontalLayout_2")
        spacerItem = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem)
        self.onComButton = QtWidgets.QPushButton(self.displayMessageBox)
        self.onComButton.setEnabled(False)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.onComButton.sizePolicy().hasHeightForWidth())
        self.onComButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.onComButton.setFont(font)
        self.onComButton.setObjectName("onComButton")
        self.horizontalLayout_2.addWidget(self.onComButton)
        self.offComButton = QtWidgets.QPushButton(self.displayMessageBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.offComButton.sizePolicy().hasHeightForWidth())
        self.offComButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.offComButton.setFont(font)
        self.offComButton.setObjectName("offComButton")
        self.horizontalLayout_2.addWidget(self.offComButton)
        self.clearScreenButton = QtWidgets.QPushButton(self.displayMessageBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.clearScreenButton.sizePolicy().hasHeightForWidth())
        self.clearScreenButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.clearScreenButton.setFont(font)
        self.clearScreenButton.setObjectName("clearScreenButton")
        self.horizontalLayout_2.addWidget(self.clearScreenButton)
        spacerItem1 = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.MinimumExpanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_2.addItem(spacerItem1)
        self.verticalLayout_3.addLayout(self.horizontalLayout_2)
        self.horizontalLayout.addWidget(self.displayMessageBox)
        self.groupBox_4 = QtWidgets.QGroupBox(self.groupBox)
        self.groupBox_4.setTitle("")
        self.groupBox_4.setObjectName("groupBox_4")
        self.verticalLayout_4 = QtWidgets.QVBoxLayout(self.groupBox_4)
        self.verticalLayout_4.setObjectName("verticalLayout_4")
        spacerItem2 = QtWidgets.QSpacerItem(20, 160, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout_4.addItem(spacerItem2)
        self.horizontalLayout.addWidget(self.groupBox_4)
        spacerItem3 = QtWidgets.QSpacerItem(10, 20, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout.addItem(spacerItem3)
        self.itlGroupBox = QtWidgets.QGroupBox(self.splitter)
        self.itlGroupBox.setMinimumSize(QtCore.QSize(300, 280))
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
        self.widgetITL.setMinimumSize(QtCore.QSize(0, 120))
        self.widgetITL.setObjectName("widgetITL")
        self.horizontalLayout_4 = QtWidgets.QHBoxLayout(self.widgetITL)
        self.horizontalLayout_4.setObjectName("horizontalLayout_4")
        self.verticalLayout_7.addWidget(self.widgetITL)
        self.verticalLayout_5.addWidget(self.itlTrafficLightBox)
        self.itlControlsBox = QtWidgets.QGroupBox(self.itlGroupBox)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Preferred, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.itlControlsBox.sizePolicy().hasHeightForWidth())
        self.itlControlsBox.setSizePolicy(sizePolicy)
        self.itlControlsBox.setMinimumSize(QtCore.QSize(0, 30))
        self.itlControlsBox.setTitle("")
        self.itlControlsBox.setObjectName("itlControlsBox")
        self.verticalLayout_6 = QtWidgets.QVBoxLayout(self.itlControlsBox)
        self.verticalLayout_6.setObjectName("verticalLayout_6")
        self.verticalLayout = QtWidgets.QVBoxLayout()
        self.verticalLayout.setObjectName("verticalLayout")
        self.horizontalLayout_3 = QtWidgets.QHBoxLayout()
        self.horizontalLayout_3.setObjectName("horizontalLayout_3")
        spacerItem4 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem4)
        self.autoItlButton = QtWidgets.QPushButton(self.itlControlsBox)
        self.autoItlButton.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.autoItlButton.sizePolicy().hasHeightForWidth())
        self.autoItlButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.autoItlButton.setFont(font)
        self.autoItlButton.setObjectName("autoItlButton")
        self.horizontalLayout_3.addWidget(self.autoItlButton)
        self.offItlButton = QtWidgets.QPushButton(self.itlControlsBox)
        self.offItlButton.setEnabled(True)
        sizePolicy = QtWidgets.QSizePolicy(QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Fixed)
        sizePolicy.setHorizontalStretch(0)
        sizePolicy.setVerticalStretch(0)
        sizePolicy.setHeightForWidth(self.offItlButton.sizePolicy().hasHeightForWidth())
        self.offItlButton.setSizePolicy(sizePolicy)
        font = QtGui.QFont()
        font.setPointSize(10)
        font.setBold(True)
        font.setWeight(75)
        self.offItlButton.setFont(font)
        self.offItlButton.setObjectName("offItlButton")
        self.horizontalLayout_3.addWidget(self.offItlButton)
        spacerItem5 = QtWidgets.QSpacerItem(40, 20, QtWidgets.QSizePolicy.Fixed, QtWidgets.QSizePolicy.Minimum)
        self.horizontalLayout_3.addItem(spacerItem5)
        self.verticalLayout.addLayout(self.horizontalLayout_3)
        spacerItem6 = QtWidgets.QSpacerItem(20, 40, QtWidgets.QSizePolicy.Minimum, QtWidgets.QSizePolicy.Fixed)
        self.verticalLayout.addItem(spacerItem6)
        self.verticalLayout_6.addLayout(self.verticalLayout)
        self.verticalLayout_5.addWidget(self.itlControlsBox)
        self.gridLayout.addWidget(self.splitter, 0, 0, 1, 1)

        self.retranslateUi(Widget)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    def retranslateUi(self, Widget):
        _translate = QtCore.QCoreApplication.translate
        Widget.setWindowTitle(_translate("Widget", "Widget"))
        self.mainLabel.setText(_translate("Widget", "Mesaj:"))
        self.statusCheckBox.setText(_translate("Widget", "Show messages status"))
        self.onComButton.setText(_translate("Widget", "OnCom"))
        self.offComButton.setText(_translate("Widget", "OffCom"))
        self.clearScreenButton.setText(_translate("Widget", "Clear"))
        self.autoItlButton.setText(_translate("Widget", "Auto ITL"))
        self.offItlButton.setText(_translate("Widget", "Off ITL"))
