from PyQt5.QtCore import QTimer,Qt
from PyQt5.QtWidgets import QLabel, QVBoxLayout,QSizePolicy
from TrafficLightItem import TrafficLightItem
from ItlConstants import *

class TimedTrafficLightItem(TrafficLightItem):
    def __init__(self, name="ITL", parent=None):
        super().__init__(name, parent)

        # create countdown label widget
        self.countdown_label = QLabel(str(0))
        self.countdown_label.setAlignment(Qt.AlignCenter)
        self.countdown_label.setMaximumHeight(int(self.MAX_LABEL_HEIGHT/2.0))
        self.countdown_label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)

        # add countdown label widget to the layout
        countdown_layout = QVBoxLayout()
        countdown_layout.addWidget(self.countdown_label)
        self.layout().addLayout(countdown_layout)

        # set the initial countdown values
        self.__RED_COLOR_TIME=10
        self.__YELLOW_COLOR_TIME=3
        self.__GREEN_COLOR_TIME=10

        self.countdown_red=self.__RED_COLOR_TIME
        self.countdown_yellow=self.__YELLOW_COLOR_TIME
        self.countdown_green=self.__GREEN_COLOR_TIME

        # create a timer for the countdown
        self.countdown_timer = QTimer(self)
        self.countdown_timer.timeout.connect(self._updateCountdownLabel)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        font_size = int(0.04 * self.height())
        font = self.countdown_label.font()
        if font_size <=self.MAX_LABEL_HEIGHT:
            font.setPointSize(font_size)
        else:
            font.setPointSize(self.MAX_LABEL_HEIGHT)
        self.countdown_label.setFont(font)

    def timedStart(self, valueRed=10,valueYellow=3,valueGreen=10,startingState=ItlStates.STATE_GREEN):
        # start the countdown timer with the specified value
        self.__RED_COLOR_TIME=valueRed
        self.__YELLOW_COLOR_TIME=valueYellow
        self.__GREEN_COLOR_TIME=valueGreen

        self.countdown_red=self.__RED_COLOR_TIME
        self.countdown_yellow=self.__YELLOW_COLOR_TIME
        self.countdown_green=self.__GREEN_COLOR_TIME
        self.countdown_timer.start(1000) # update every 1 second
        self.ItlSetState(startingState)

    def _updateCountdownLabel(self):
        # decrement the countdown value and update the label
        state_val=super().getCurrentColorState()
        if state_val==ItlStates.STATE_GREEN:
            self.countdown_label.setText(str(self.countdown_green))
            # if the countdown value for green color reaches 0, next color will be activated
            if self.countdown_green <= 0:
                self.ItlNextColorState()
                self.countdown_green=self.__GREEN_COLOR_TIME
            else:
                self.countdown_green -= 1

        elif state_val==ItlStates.STATE_YELLOW:
            self.countdown_label.setText(str(self.countdown_yellow))
            # if the countdown value for yellow color reaches 0, next color will be activated
            if self.countdown_yellow <= 0:
                self.ItlNextColorState()
                self.countdown_yellow=self.__YELLOW_COLOR_TIME
            else:
                self.countdown_yellow-=1

        elif state_val==ItlStates.STATE_RED:
            self.countdown_label.setText(str(self.countdown_red))
            # if the countdown value for red color reaches 0, next color will be activated
            if self.countdown_red <= 0:
                self.ItlNextColorState()
                self.countdown_red=self.__RED_COLOR_TIME
            else:
                self.countdown_red-=1
        
        elif state_val==ItlStates.STATE_OFF:
            self.countdown_label.setText(str(0))


    def setItlOff(self):
        self._ItlTurnOff()
        self.countdown_timer.stop()
        self.countdown_red=self.__RED_COLOR_TIME
        self.countdown_yellow=self.__YELLOW_COLOR_TIME
        self.countdown_green=self.__GREEN_COLOR_TIME
        self._updateCountdownLabel()

    def ItlSetCurrentColorState(self,state):
        self.ItlSetState(state)
        self.countdown_timer.stop()
        self.countdown_label.setText(str(0))
