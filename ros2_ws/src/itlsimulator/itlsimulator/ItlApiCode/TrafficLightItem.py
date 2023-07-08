
from PyQt5.QtCore import Qt 
from PyQt5.QtWidgets import QWidget,QLabel,QVBoxLayout,QSizePolicy
from ItlWidget import ItlWidget


class TrafficLightItem(QWidget):

    #private constants
    HEIGHT_WIDTH_RATIO=3
    BASE_WIDTH=10
    BASE_HEIGHT=BASE_WIDTH*HEIGHT_WIDTH_RATIO
    MAX_RATIO=30

    #public constants
    MAX_LABEL_HEIGHT=30


    def __init__(self, name="ITL", parent=None):
        super().__init__(parent)
        
        # create traffic light widget
        self.__trafficLightInstace = ItlWidget(name)

        
        # create label widget
        self.label = QLabel(name)
        self.label.setAlignment(Qt.AlignCenter)
        self.MAX_LABEL_HEIGHT=int(0.07*self.height())
        self.label.setFixedHeight(self.MAX_LABEL_HEIGHT)
        self.label.setSizePolicy(QSizePolicy.Expanding, QSizePolicy.Expanding)
        
        # create vertical box layout
        layout = QVBoxLayout()
        layout.addWidget(self.__trafficLightInstace)
        layout.addWidget(self.label)
        layout.setSpacing(0)
        layout.setContentsMargins(0, 0, 0, 0)
        self.setLayout(layout)

        #set size policy
        self.setMinimumSize(self.BASE_WIDTH,self.BASE_HEIGHT)
        self.setMaximumSize(self.MAX_RATIO*self.BASE_WIDTH,self.MAX_RATIO*self.BASE_WIDTH)

    def resizeEvent(self, event):
        super().resizeEvent(event)
        font_size = int(0.04 * self.height())
        font = self.label.font()
        if font_size <=self.MAX_LABEL_HEIGHT:
            font.setPointSize(font_size)
        else:
            font.setPointSize(self.MAX_LABEL_HEIGHT)
        self.label.setFont(font)
    

    def ItlNextColorState(self):
        self.__trafficLightInstace._nextState()

    def ItlSetState(self,currentColorState):
        self.__trafficLightInstace._setCurrentState(currentColorState)

    def _ItlTurnOff(self):
        self.__trafficLightInstace._turnOff()

    def getCurrentColorState(self):
        return self.__trafficLightInstace.getCurrentState()