import sys
import os
import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit,QWidget
from PyQt5.QtCore import *
from PyQt5 import uic
from PyQt5.QtGui import QTextCursor, QTextCharFormat, QColor
import rclpy
from std_msgs.msg import String
from rclpy.qos import qos_profile_sensor_data

# add ItlApiCode module to sys.path
itl_api_code_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ItlApiCode")
sys.path.append(itl_api_code_path)
# import TrafficLightItem
from itlsimulator.ItlApiCode.TrafficLightItem import TrafficLightItem
from itlsimulator.ItlApiCode.TimedTrafficLightItem import *

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

        self.central_widget = QWidget()
        self.setCentralWidget(self.central_widget)
        ui_file_path = os.path.join(os.path.dirname(__file__), 'ui_files','itl_subscriber_window.ui')
        uic.loadUi(ui_file_path, self.central_widget)
        

        # Set the window title
        self.setWindowTitle("ITL ROS 2 :: Subscriber")
        self.central_widget.mainLabel.setText("Subscribed message:");

        #configure the ITL
        self.trafficItem= TimedTrafficLightItem("ITL")
        layout=self.central_widget.widgetITL.layout()
        layout.addWidget(self.trafficItem)

        # Create a subscriber thread to the topic
        self.subscriber_thread = SubscriberThread(topic)
        self.subscriber_thread.msg_received.connect(self.update_gui)
        self.subscriber_thread.start()

    def update_gui(self, msg):
        if len(msg)>=30 and len(msg)<=32:
            color="black"
        else:
            color="red"
        # Update the text in the QTextEdit widget with the most recent message
        text_cursor = self.central_widget.textEdit.textCursor()
        text_cursor.movePosition(QTextCursor.End)
        text_cursor.insertHtml(f"<span style=\"color:{color}\">{msg}</span><br>")
        self.central_widget.textEdit.setTextCursor(text_cursor)
        self.central_widget.textEdit.ensureCursorVisible()
        self.trafficItem.ItlNextColorState()
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
