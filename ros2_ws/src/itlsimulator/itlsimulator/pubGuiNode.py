import sys
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton, QLineEdit
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
        self.setWindowTitle('Talker')
        self.setGeometry(100, 100, 300, 150)

        self.talker = TalkerNode()

        self.textbox = QLineEdit(self)
        self.textbox.move(20, 20)
        self.textbox.resize(200, 25)

        self.button = QPushButton('Send', self)
        self.button.move(20, 60)
        self.button.resize(100, 25)
        self.button.clicked.connect(self.send_message)

    def send_message(self):
        message = self.textbox.text()
        self.talker.send_message(message)

def main():
    rclpy.init(args=sys.argv)
    app = QApplication(sys.argv)
    window = MainWindow()
    window.show()
    try:
        app.exec_()
    except KeyboardInterrupt:
        pass

if __name__ == '__main__':
    main()
