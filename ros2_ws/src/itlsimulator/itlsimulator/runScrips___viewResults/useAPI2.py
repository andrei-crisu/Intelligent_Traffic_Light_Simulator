import sys
from PyQt5.QtWidgets import QApplication, QWidget, QVBoxLayout,QHBoxLayout
from TrafficLightItem import TrafficLightItem
from PyQt5.QtCore import Qt


def main():
    # create application
    app = QApplication(sys.argv)

    # create main window
    window = QWidget()
    window.setWindowTitle('Traffic Light Example')
    window.setGeometry(100, 100, 200, 300)

    # create traffic light items
    item1 = TrafficLightItem('Intersection 1')
    item2 = TrafficLightItem('Intersection 2')

    # create vertical box layout for main window
    layout = QVBoxLayout()
    layout.setAlignment(Qt.AlignHCenter)
    layout.addWidget(item1)
    layout.addWidget(item2)
    layout.setSpacing(0)
    layout.setContentsMargins(0, 0, 0, 0)
    window.setLayout(layout)

    # show main window
    window.show()

    # run event loop
    sys.exit(app.exec_())


if __name__ == '__main__':
    main()
