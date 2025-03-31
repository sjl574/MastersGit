from PyQt5.QtCore import QSize, Qt
from PyQt5.QtWidgets import QApplication, QMainWindow, QPushButton


# Subclass QMainWindow to customize your application's main window
class MainWindow(QMainWindow):
    def __init__(self):
        super().__init__()

        self.setWindowTitle("My App")

        #Setup a test button
        self.testButton = QPushButton("Press Me!")
        #Attach function to test button
        self.testButton.clicked.connect(self.testButtonClicked)
        # Set the central widget of the Window.
        self.setCentralWidget(self.testButton)
        #store button state
        self.buttonState = False

    def testButtonClicked(self):
        self.buttonState = not self.buttonState
        if self.buttonState:
            self.testButton.setText("Active")
        else:
            self.testButton.setText("Not Active")
        print("Test Button Clicked \n")

app = QApplication([])

window = MainWindow()
window.show()

app.exec()