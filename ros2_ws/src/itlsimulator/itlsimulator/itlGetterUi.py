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

#add ItlComProtocol module to sys.path
itl_api_code_path = os.path.join(os.path.dirname(os.path.abspath(__file__)), "ItlComProtocol")
sys.path.append(itl_api_code_path)
#import 
from itlsimulator.ItlComProtocol.rxMsgExtract import *
from itlsimulator.ItlComProtocol.comConstants import *

class SubscriberThread(QThread):
    msg_received = pyqtSignal(str)

    def __init__(self, topic):
        super().__init__()
        self.node = rclpy.create_node('getterUiNode')
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
    COM_UPDATE_GUI_OFF=False
    COM_UPDATE_GUI_ON=True

    def __init__(self, topic):
        super().__init__()

        #define some class specific variables
        self.comStatusUpdateGui=self.COM_UPDATE_GUI_ON
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

        #connect ui controls to the corresponding functions + other ui configurations
        self.central_widget.offComButton.clicked.connect(self.deactivateMsgDisplay)
        self.central_widget.onComButton.clicked.connect(self.activateMsgDisplay)
        self.central_widget.clearScreenButton.clicked.connect(self.clearMsgScreen)
        self.central_widget.autoItlButton.clicked.connect(self.startTrafficLight)
        self.central_widget.offItlButton.clicked.connect(self.shutDownTrafficLight)

        # Create a subscriber thread to the topic
        self.subscriber_thread = SubscriberThread(topic)
        self.subscriber_thread.msg_received.connect(self.update_gui)
        self.subscriber_thread.start()


    def setItlOnGreen(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_GREEN)
        self.updateHistoryMessage("Set State :: GREEN")

    def setItlOnYellow(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_YELLOW)
        self.updateHistoryMessage("Set State :: YELLOW")
    
    def setItlOnRed(self):
        self.trafficItem.ItlSetCurrentColorState(ItlStates.STATE_RED)
        self.updateHistoryMessage("Set State :: RED")

    def itlShutDown(self):
        self.trafficItem.setItlOff()
        self.updateHistoryMessage("Set State :: OFF")
        self.central_widget.autoItlButton.setEnabled(True)
        self.central_widget.offItlButton.setEnabled(False)

    def itlAutoRun(self):
        self.trafficItem.timedStart(8,2,10)
        self.updateHistoryMessage("Set State :: AUTO")
        self.central_widget.autoItlButton.setEnabled(False)
        self.central_widget.offItlButton.setEnabled(True)

    def executeCommand(self,received_cmd):
        if received_cmd==ItlCmdMessage.RED_STATE:
            self.setItlOnRed()
        elif received_cmd==ItlCmdMessage.YELLOW_STATE:
            self.setItlOnYellow()
        elif received_cmd==ItlCmdMessage.GREEN_STATE:
            self.setItlOnGreen()
        elif received_cmd==ItlCmdMessage.AUTO_STATE:
            self.itlAutoRun()
        elif received_cmd==ItlCmdMessage.OFF_STATE:
            self.shutDownTrafficLight()
        else:
            self.updateHistoryMessage("ERR:RECEIVED UNKNOWN CMD: "+received_cmd,color="red")



    def consumeData(self,data_str,type_str):
        if type_str==ItlMessageTypes.MSGT_COMMAND:
            self.executeCommand(data_str)
        else:
            self.updateHistoryMessage("<br> SHOW INFO : "+data_str,color="green")
    
    def updateHistoryMessage(self,text_to_display,color="green"):
        text_cursor = self.central_widget.textEdit.textCursor()
        text_cursor.movePosition(QTextCursor.End)
        text_cursor.insertHtml(f"<span style=\"color:{color}\">{text_to_display}</span><br>")
        self.central_widget.textEdit.setTextCursor(text_cursor)
        self.central_widget.textEdit.ensureCursorVisible()

    #function to update gui based on the received message
    def update_gui(self, msg):
        if(self.comStatusUpdateGui==self.COM_UPDATE_GUI_ON):
            msgGetter=MsgExtract()
            extractionStatus=msgGetter.extractItlMessage(msg)
            if(extractionStatus==True):
                color="black"
                raw_data=msgGetter.data
                text_to_display=("----------------<br>"+
                            "FULL RECEIVED MESSAGE:: "+msg+" | "+
                            "RAW DATA:: "+raw_data)
                self.updateHistoryMessage(text_to_display,color)
                self.consumeData(raw_data,msgGetter.data_type)
                msgGetter.clearExtractor()
                
            else:
                text_to_display=("----------------<br>"+
                            "FULL RECEIVED MESSAGE:: "+msg)
                color="red"
                self.updateHistoryMessage(text_to_display,color)
            # Update the text in the QTextEdit widget with the most recent message
            
        QApplication.processEvents()

    def deactivateMsgDisplay(self):
        self.comStatusUpdateGui=self.COM_UPDATE_GUI_OFF
        self.central_widget.offComButton.setEnabled(False)
        self.central_widget.onComButton.setEnabled(True)
        self.central_widget.statusCheckBox.setChecked(False)
        self.central_widget.textEdit.append("DISPLAY MESSAGES=OFF\n")


    def activateMsgDisplay(self):
        self.comStatusUpdateGui=self.COM_UPDATE_GUI_ON
        self.central_widget.offComButton.setEnabled(True)
        self.central_widget.onComButton.setEnabled(False)
        self.central_widget.statusCheckBox.setChecked(True)
        self.central_widget.textEdit.append("DISPLAY MESSAGES=ON\n")


    def clearMsgScreen(self):
        self.central_widget.textEdit.clear()

    def startTrafficLight(self):
        self.trafficItem.timedStart(8,2,10)
        self.central_widget.textEdit.append("LOCAL OVERWRITE: ITL::auto cmd\n")
        self.central_widget.autoItlButton.setEnabled(False)
        self.central_widget.offItlButton.setEnabled(True)

    def shutDownTrafficLight(self):
        self.trafficItem.setItlOff()
        self.central_widget.textEdit.append("LOCAL OVERWRITE: ITL::off cmd\n")
        self.central_widget.autoItlButton.setEnabled(True)
        self.central_widget.offItlButton.setEnabled(False)



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
