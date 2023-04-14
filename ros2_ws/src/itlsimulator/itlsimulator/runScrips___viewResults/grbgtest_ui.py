import os
from PyQt5 import QtWidgets, uic

# get the absolute path of the directory containing this script
script_dir = os.path.dirname(os.path.abspath(__file__))

# load the UI file using a relative path
ui_file_path = os.path.join(script_dir, "ui_files/maindesign.ui")
Ui_MainWindow, QMainWindow = uic.loadUiType(ui_file_path)

# Create a new class for the main window
class MainWindow(QMainWindow, Ui_MainWindow):
    def __init__(self):
        super().__init__()
        self.setupUi(self)

# Create the QApplication and MainWindow objects, and start the application
if __name__ == "__main__":
    app = QtWidgets.QApplication([])
    window = MainWindow()
    window.show()
    app.exec_()
