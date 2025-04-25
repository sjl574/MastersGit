from matplotlib.backends.backend_qt5agg import FigureCanvasQTAgg
from matplotlib.figure import Figure
from PyQt5 import  QtWidgets


class MplCanvas(FigureCanvasQTAgg):

    def __init__(self, width=5, height=4, dpi=100):
        fig = Figure(figsize=(width, height), dpi=dpi)
        self.axes = fig.add_subplot(111)
        super().__init__(fig)

class PlotWindow(QtWidgets.QDialog):
    def __init__(self, parent=None):
        super().__init__(parent)

    def plot(self, xAxis, yAxis):
        #Setup window
        self.setWindowTitle("Trajectory Visualization")
        self.setGeometry(100, 100, 600, 400)
        #Create layout
        self.layout = QtWidgets.QVBoxLayout(self)
        #Create plot
        self.canvas = MplCanvas(width=5, height=4, dpi=100)
        self.canvas.axes.plot(xAxis, yAxis)
        #Add plot to layout
        self.layout.addWidget(self.canvas)
        #show window
        self.exec()
