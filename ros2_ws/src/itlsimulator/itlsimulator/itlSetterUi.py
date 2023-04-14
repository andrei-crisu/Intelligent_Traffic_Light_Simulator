import sys
import os
import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit,QWidget
from PyQt5 import uic
import rclpy
from std_msgs.msg import String



import datetime
import random
import string

class MsgCreator:
    def __init__(self):
        self.data=''

    def get_formatted_datetime(self, year, month, day, hour, minute, second):
        dt = datetime.datetime(year, month, day, hour, minute, second)
        formatted_dt = dt.strftime("%y%m%d%H%M%S")
        return formatted_dt

    def generate_random_string(self, length):
        letters = string.ascii_lowercase
        random_string = ''.join(random.choice(letters) for i in range(length))
        return random_string

    def get_formatted_current_datetime(self):
        now = datetime.datetime.now()
        formatted_dt = now.strftime("%y%m%d%H%M%S")
        return formatted_dt
    
    def type_selector(self):
        return 'I0F'
    
    def componenetFactory(self,dataString):
        msgString=''
        msgID=self.get_formatted_current_datetime()
        msgType=self.type_selector()
        msgDataLen=len(dataString)
        msgData=dataString
        msgSecurityString='$-0-$'
        msgString=msgID+'#'+msgType+"#"+str(msgDataLen)+"#"+msgData+"#"+msgSecurityString
        return msgString
    
    def getItlMessage(self,dataString='0xDEF'):
        msgString=''
        if dataString=='0xDEF':
            msgDataLen=random.randint(40,50)
            msgString=self.componenetFactory(self.generate_random_string(msgDataLen))
        else:
            msgString=self.componenetFactory(dataString)
        
        #here the data field is set 
        self.data=msgString
        return self.data



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
        local_msg=MsgCreator()
        message=local_msg.getItlMessage(message)
        self.talker.send_message(message)

    def send_random_message(self):
         local_msg=MsgCreator()
         message=local_msg.getItlMessage()
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
