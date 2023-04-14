import sys
from PyQt5.QtCore import QTimer, QThread, pyqtSignal
from PyQt5.QtWidgets import QApplication, QMainWindow, QTextEdit
import rclpy
from rclpy.qos import qos_profile_sensor_data
from std_msgs.msg import String


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

    def run(self):
        try:
            rclpy.spin(self.node)
        except KeyboardInterrupt:
            pass
        self.node.destroy_node()
        rclpy.shutdown()


class MainWindow(QMainWindow):
    def __init__(self, topic):
        super().__init__()

        # Set the window title
        self.setWindowTitle("ROS 2 Subscriber")

        # Create a QTextEdit widget and set it as the central widget of the window
        self.text_edit = QTextEdit(self)
        self.setCentralWidget(self.text_edit)

        # Create a subscriber thread to the topic
        self.subscriber_thread = SubscriberThread(topic)
        self.subscriber_thread.msg_received.connect(self.update_gui)
        self.subscriber_thread.start()

    def update_gui(self, msg):
        # Update the text in the QTextEdit widget with the most recent message
        self.text_edit.append(msg)
        QApplication.processEvents()


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
