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
        self.setWindowTitle("Trajectory Visualization")
        self.setGeometry(100, 100, 600, 400)

        #Create plot
        sc = MplCanvas(width=5, height=4, dpi=100)
        sc.axes.plot([0,1,2,3,4], [10,1,20,3,40])

        #show window
        self.exec()
