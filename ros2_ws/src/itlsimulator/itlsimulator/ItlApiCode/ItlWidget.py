from PyQt5.QtCore import QRectF
from PyQt5.QtGui import QPainter
from PyQt5.QtWidgets import QWidget
from PyQt5 import QtWidgets
from ItlConstants import *

class ItlWidget(QWidget):
    
    def __init__(self,textName,parent=None):
        super().__init__(parent)
        self.__textNameVal=textName
        self. __currentState=ItlStates.STATE_OFF

        #widget minimal values
        self.__WMIN=5
        self.__HMIN=15

        
        #set size policy for min and max
        self.setMinimumSize(self.__WMIN, self.__HMIN)
        self.setSizePolicy(QtWidgets.QSizePolicy.Expanding,QtWidgets.QSizePolicy.Expanding)

        #initialize the colors of the traffic light
        self._turnOff()

        #Get the size and position of the widget
        self.__widgetHeight=self.height()
        self.__widgetWidth=self.__widgetHeight*0.3333

        #center the rectangle in the middle
        self.__xPosition=self.width()*0.5-self.__widgetWidth*0.5
        self.__yPosition=0

        self.__circleDiameter=self.__widgetHeight*0.25
        self.__circleMargin=self.__widgetHeight*0.05

    def paintEvent(self, event):
        painter = QPainter(self)
        painter.setRenderHint(QPainter.Antialiasing)
        painter.setPen(Qt.NoPen)

        #Get the size of the widget
        self.__widgetHeight=self.height()
        self.__widgetWidth=self.__widgetHeight*0.3333
        #center it in the middle
        self.__xPosition=self.width()*0.5-self.__widgetWidth*0.5
        self.__yPosition=0

        self.__circleDiameter=self.__widgetHeight*0.25
        self.__circleMargin=self.__widgetHeight*0.05
        xCoordCircle=self.__xPosition+(self.__widgetWidth-self.__circleDiameter)/2.0
        yCoordRedCircle=self.__yPosition+self.__circleMargin
        yCoordYellowCircle=yCoordRedCircle+self.__circleDiameter+self.__circleMargin
        yCoordGreenCircle=yCoordYellowCircle+self.__circleDiameter+self.__circleMargin

        # Draw the dark background with rounded corners
        bg_rect = QRectF(self.__xPosition,self.__yPosition, self.__widgetWidth, self.__widgetHeight)
        bg_radius = self.__widgetHeight*0.09
        painter.setBrush(QColor("#171710"))
        painter.drawRoundedRect(bg_rect, bg_radius, bg_radius)

        # Draw the red light
        painter.setBrush(self.red)
        painter.drawEllipse(xCoordCircle, yCoordRedCircle, self.__circleDiameter, self.__circleDiameter)

        # Draw the yellow light
        painter.setBrush(self.yellow)
        painter.drawEllipse(xCoordCircle, yCoordYellowCircle, self.__circleDiameter, self.__circleDiameter)

        # Draw the green light
        painter.setBrush(self.green)
        painter.drawEllipse(xCoordCircle, yCoordGreenCircle, self.__circleDiameter, self.__circleDiameter)


    def __setBrightRed(self):
        self.__currentState=ItlStates.STATE_RED
        self.red = ItlColors.RED
        self.yellow = ItlColors.DARK_YELLOW
        self.green = ItlColors.DARK_GREEN
        self.update()

    def __setBrightYellow(self):
        self.__currentState = ItlStates.STATE_YELLOW
        self.red = ItlColors.DARK_RED
        self.yellow = ItlColors.YELLOW
        self.green = ItlColors.DARK_GREEN
        self.update()

    def __setBrightGreen(self):
        self.__currentState=ItlStates.STATE_GREEN
        self.red = ItlColors.DARK_RED
        self.yellow =ItlColors.DARK_YELLOW
        self.green = ItlColors.GREEN
        self.update()

    def _turnOff(self):
        self.__currentState=ItlStates.STATE_OFF
        self.red = ItlColors.DARK_RED
        self.yellow =ItlColors.DARK_YELLOW
        self.green = ItlColors.DARK_GREEN
        self.update()

    def _setCurrentState(self,state):

        self.__currentState=state
        if self.__currentState >ItlStates.STATE_GREEN:
            self.__currentState=ItlStates.STATE_RED
        
        if self.__currentState <ItlStates.STATE_OFF:
            self.__currentState=ItlStates.STATE_OFF

        if self.__currentState==ItlStates.STATE_OFF:
            self._turnOff()
        elif self.__currentState==ItlStates.STATE_RED:
            self.__setBrightRed()
        elif self.__currentState==ItlStates.STATE_YELLOW:
            self.__setBrightYellow()
        elif self.__currentState==ItlStates.STATE_GREEN:
            self.__setBrightGreen()

        return self.__currentState


    def _nextState(self):
        state=self.__currentState+1
        return_val=self._setCurrentState(state)
        return return_val
    
    def getCurrentState(self):
        return self.__currentState
            

