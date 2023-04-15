
from PyQt5.QtWidgets import *
from PyQt5 import QtWidgets
from ItlApiCode.TrafficLightItem import TrafficLightItem
from ItlApiCode.TimedTrafficLightItem import *

class MainWindow(QWidget):
    def __init__(self):
        super().__init__()
        self.initUI()

    def initUI(self):
        # Create the traffic light widget
        self.trafficLight = TrafficLightItem("ITL1")
        self.trafficLight2 = TrafficLightItem("ITL2")
        self.trafficLight3 = TrafficLightItem("ITL3")
        self.trafficLight4 = TrafficLightItem("ITL4")
        self.timedTrafficLight = TimedTrafficLightItem("ITL5")

        # Create the buttons and specers
        self.nextStateButton = QPushButton("Next",self)
        self.nextStateButton.setSizePolicy(QtWidgets.QSizePolicy.Fixed,QtWidgets.QSizePolicy.Fixed)
        self.nextStateButton.setFixedSize(60,20)
        self.offStateButton = QPushButton("Off",self)
        self.offStateButton.setSizePolicy(QtWidgets.QSizePolicy.Fixed,QtWidgets.QSizePolicy.Fixed)
        self.offStateButton.setFixedSize(60,20)
        self.autoButton = QPushButton("Auto",self)
        self.autoButton.setSizePolicy(QtWidgets.QSizePolicy.Fixed,QtWidgets.QSizePolicy.Fixed)
        self.autoButton.setFixedSize(60,20)
        spacer = QtWidgets.QSpacerItem(40,20, QtWidgets.QSizePolicy.Expanding, QtWidgets.QSizePolicy.Fixed)

        # Add the buttons to a horizontal layout
        button_layout = QHBoxLayout()
        button_layout.addSpacerItem(spacer)
        button_layout.addWidget(self.nextStateButton)
        button_layout.addWidget(self.offStateButton)
        button_layout.addWidget(self.autoButton)


        # Add the traffic widget and button layout to a vertical layout
        main_layout = QVBoxLayout()
        trafficLightLayout=QHBoxLayout()
        trafficLightLayout.addWidget(self.trafficLight)
        trafficLightLayout.addWidget(self.trafficLight2)
        trafficLightLayout.addWidget(self.trafficLight3)
        trafficLightLayout.addWidget(self.trafficLight4)
        trafficLightLayout.addWidget(self.timedTrafficLight)
        main_layout.addLayout(trafficLightLayout)
        main_layout.addLayout(button_layout)

        # Set the main layout
        self.setLayout(main_layout)

        # Connect the button signals to their respective slots
        self.nextStateButton.clicked.connect(self.nextState)
        self.offStateButton.clicked.connect(self.turnOff)
        self.autoButton.clicked.connect(self.autoSet)

    def nextState(self):
        self.trafficLight.ItlNextColorState()
        self.trafficLight2.ItlNextColorState()
        self.trafficLight3.ItlNextColorState()
        self.trafficLight4.ItlNextColorState()

    def turnOff(self):
        self.trafficLight._ItlTurnOff()
        self.trafficLight2._ItlTurnOff()
        self.trafficLight3._ItlTurnOff()
        self.trafficLight4._ItlTurnOff()
        self.timedTrafficLight.setItlOff()
        self.autoButton.setEnabled(True)

    def autoSet(self):
        self.timedTrafficLight.timedStart(8,2,10)
        self.autoButton.setEnabled(False)

if __name__ == '__main__':
    import sys
    app = QApplication(sys.argv)
    window = MainWindow()
    window.resize(800,600)
    window.show()
    sys.exit(app.exec_())
