



import sys
import rclpy
from PyQt5 import QtWidgets
from PyQt5.QtCore import QThread
from PyQt5.QtWidgets import QWidget, QApplication
from std_msgs.msg import String
from PyQt5 import QtCore, QtGui, QtWidgets


#---------------------------------------------------------------------------

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

        #self.retranslateUi(Widget)
        QtCore.QMetaObject.connectSlotsByName(Widget)

    # def retranslateUi(self, Widget):
    #     _translate = QtCore.QCoreApplication.translate
    #     Widget.setWindowTitle(_translate("Widget", "Widget"))
    #     self.mainLabel.setText(_translate("Widget", "Mesaj receptionat"))
    #     self.Button_1.setText(_translate("Widget", "PushButton"))
    #     self.Button_2.setText(_translate("Widget", "PushButton"))
    #     self.Button_3.setText(_translate("Widget", "PushButton"))
    #     self.Button_4.setText(_translate("Widget", "PushButton"))
    #     self.Button_5.setText(_translate("Widget", "PushButton"))

#---------------------------------------------------------------------------
class SubscriberThread(QThread):
    msg_received = pyqtSignal(str)

    def __init__(self, topic):
        super().__init__()
        self.node = rclpy.create_node('qt_subscriber_thread')
        self.subscriber = self.node.create_subscription(
            String,
            topic,
            self.subscriber_callback,
            qos_profile=qos_profile_sensor_data)

    def subscriber_callback(self, msg):
        self.msg_received.emit(msg.data)

    def __del__(self):
        self.node.destroy_node()
        rclpy.shutdown()
        del self.subscription

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()
        rclpy.shutdown()


#---------------------------------------------------------------------------
class MainWindow(QtWidgets.QMainWindow):
    def __init__(self, topic):
        super().__init__()

        # create a new QWidget object and set the Ui_Widget as its child
        self.widget = QWidget()
        self.ui = Ui_Widget()
        self.ui.setupUi(self.widget)

        # set the central widget to the newly created QWidget object
        self.setCentralWidget(self.widget)

        # Connect the click event functions to the buttons
        self.ui.Button_1.clicked.connect(self.on_Button_1_clicked)
        self.ui.Button_2.clicked.connect(self.on_Button_2_clicked)
        self.ui.Button_3.clicked.connect(self.on_Button_3_clicked)
        self.ui.Button_4.clicked.connect(self.on_Button_4_clicked)
        self.ui.Button_5.clicked.connect(self.on_Button_5_clicked)

        # Initialize message string
        self.message = ""

        # Set up the ROS2 node and subscriber
        self.node = rclpy.create_node('listener_node')
        self.sub = self.node.create_subscription(String, 'topic007', self.callback, 10)

        # Start the ROS2 node
        self.timer = self.startTimer(10) # 10 milliseconds


        self.subscriber_thread = SubscriberThread(topic)
        self.subscriber_thread.msg_received.connect(self.update_gui)
        self.subscriber_thread.start()

    def update_gui(self, msg):
        # Update the text in the QTextEdit widget with the most recent message
        self.text_edit.append(msg)
        QApplication.processEvents()

    def timerEvent(self, event):
        rclpy.spin_once(self.node)

    # Define the click event functions for the buttons
    def on_Button_1_clicked(self):
        self.message = "Button 1 clicked!"
        print(self.message)

    def on_Button_2_clicked(self):
        self.message = "Button 2 clicked!"

    def on_Button_3_clicked(self):
        self.message = "Button 3 clicked!"

    def on_Button_4_clicked(self):
        self.message = "Button 4 clicked!"

    def on_Button_5_clicked(self):
        self.message = "Button 5 clicked!"

    # Define the callback function for the subscriber
    def callback(self, msg):
        self.ui.textEdit.append(msg.data)


def main(args=None):
    # Initialize the ROS 2 node
    rclpy.init(args=args)

    # Create the QApplication instance
    app = QApplication(sys.argv)

    # Create the main window
    main_window = MainWindow(topic='topic007')
    main_window.show()

    # Start the QApplication event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
