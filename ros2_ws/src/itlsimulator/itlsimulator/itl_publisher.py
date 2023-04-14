import sys
import os
import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit,QWidget
from PyQt5 import uic
import rclpy
from std_msgs.msg import String

class TalkerNode:
    def __init__(self):
        self.node = rclpy.create_node('talker_node')
        self.publisher = self.node.create_publisher(String, 'topic007', 10)

    def send_message(self, message):
        msg = String()
        msg.data = message
        self.publisher.publish(msg)

class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()
        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        ui_file_path = os.path.join(os.path.dirname(__file__), 'ui_files','mainwindow.ui')
        uic.loadUi(ui_file_path, self.central_widget)
         # Set the window title
        self.setWindowTitle("ITL ROS 2 :: Publisher")
        self.central_widget.mainLabel.setText("Published message:");
        self.talker = TalkerNode()

        self.central_widget.sendMessageButton.clicked.connect(self.send_message)

    def send_message(self):
        message = self.central_widget.textEdit.toPlainText()
        self.talker.send_message(message)

def main():
    # Initialize the ROS 2 node
    rclpy.init(args=sys.argv)
    # Create the QApplication instance
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
