import sys
import os
import PyQt5
from PyQt5.QtWidgets import QApplication, QMainWindow, QLineEdit,QWidget
from PyQt5 import uic
import rclpy
from std_msgs.msg import String

# add ItlApiCode module to sys.path
itl_api_code_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ItlApiCode")
sys.path.append(itl_api_code_path)
# import TrafficLightItem
from itlsimulator.ItlApiCode.TrafficLightItem import TrafficLightItem
from itlsimulator.ItlApiCode.TimedTrafficLightItem import *

#add ItlComProtocol module to sys.path
itl_api_code_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ItlComProtocol")
sys.path.append(itl_api_code_path)
#import 
from itlsimulator.ItlComProtocol.txMsgCreator import *
from itlsimulator.ItlComProtocol.comConstants import *


class TalkerNode:
    def __init__(self):
        self.node = rclpy.create_node('setterUiNode')
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
        ui_file_path = os.path.join(os.path.dirname(__file__), 'ui_files','itl_publisher_window.ui')
        uic.loadUi(ui_file_path, self.central_widget)
         # Set the window title
        self.setWindowTitle("ITL ROS 2 :: Publisher")
        self.central_widget.mainLabel.setText("Published message:");
        #configure the ITL
        self.trafficItem= TimedTrafficLightItem("ITL")
        layout=self.central_widget.widgetITL.layout()
        layout.addWidget(self.trafficItem)

        self.talker = TalkerNode()

        self.central_widget.sendMessageButton.clicked.connect(self.publishMessage)
        self.central_widget.clearMessageHistoryButton.clicked.connect(self.clearMessageHistory)
        self.central_widget.setGreenButton.clicked.connect(self.setItlOnGreen)
        self.central_widget.setYellowButton.clicked.connect(self.setItlOnYellow)
        self.central_widget.setRedButton.clicked.connect(self.setItlOnRed)
        self.central_widget.offButton.clicked.connect(self.itlShutDown)
        self.central_widget.autoButton.clicked.connect(self.itlAutoRun)
        self.central_widget.doNothingButton.clicked.connect(self.doNothing)

    def addMessageToHistoryBox(self,msg="default"):
        self.central_widget.msgHistoryText.append(msg)

    def clearMessageHistory(self):
        self.central_widget.msgHistoryText.clear()

    def setItlOnGreen(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_GREEN)
        self.addMessageToHistoryBox("Set State :: GREEN")
        self.sendCommand(ItlCmdMessage.GREEN_STATE)

    def setItlOnYellow(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_YELLOW)
        self.addMessageToHistoryBox("Set State :: YELLOW")
        self.sendCommand(ItlCmdMessage.YELLOW_STATE)
    
    def setItlOnRed(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_RED)
        self.addMessageToHistoryBox("Set State :: RED")
        self.sendCommand(ItlCmdMessage.RED_STATE)

    def itlShutDown(self):
        self.trafficItem.setItlOff()
        self.addMessageToHistoryBox("Set State :: OFF")
        self.central_widget.offButton.setEnabled(False)
        self.central_widget.autoButton.setEnabled(True)
        self.sendCommand(ItlCmdMessage.OFF_STATE)

    def itlAutoRun(self):
        self.trafficItem.timedStart(8,2,10)
        self.addMessageToHistoryBox("Set State :: AUTO")
        self.central_widget.offButton.setEnabled(True)
        self.central_widget.autoButton.setEnabled(False)
        self.sendCommand(ItlCmdMessage.AUTO_STATE)
    
    def doNothing(self):
        # this slot function is used by the doNothingButton
        self.addMessageToHistoryBox("Nothing !!!")
        
    def publishMessage(self):
        #get the raw message from the ui 
        info_message = self.central_widget.textEdit.toPlainText()
        if(len(info_message)==0):
            info_message='EMPTY_MESSAGE_NOTHING_IMPORTANT'
        #create an instance of the MsgCreator
        msg_creator=MsgCreator()
        #process the raw message to obtain a ready to send message
        message=msg_creator.buildItlMessage(info_message,ItlMessageTypes.MSGT_INFORMATION,
                                                ItlEncryptionOptions.STANDARD_ENCRYPTION_METHOD)
        #show message in the history
        self.addMessageToHistoryBox("----------------")
        self.addMessageToHistoryBox("RAW INFO:: "+info_message)
        self.addMessageToHistoryBox("FULL SENT MESSAGE:: "+message)
        #send message
        self.talker.send_message(message)

    def sendCommand(self,cmd_message):
        cmd_creator=MsgCreator()
        cmd_to_send=cmd_creator.buildItlMessage(cmd_message,ItlMessageTypes.MSGT_COMMAND,
                                                ItlEncryptionOptions.STANDARD_ENCRYPTION_METHOD)
        self.addMessageToHistoryBox("----------------")
        self.addMessageToHistoryBox("RAW COMMAND:: "+cmd_message)
        self.addMessageToHistoryBox("FULL SENT MESSAGE:: "+cmd_to_send)
        #send message
        self.talker.send_message(cmd_to_send)

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
