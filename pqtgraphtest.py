import pyqtgraph as pg
from PyQt5.QtWidgets import QApplication  
import numpy as np

app = QApplication([])


#Creating a widget
plot_widget = pg.PlotWidget(title='Scatter plot')

#Show the widget
plot_widget.show()

#Set labels
plot_widget.setLabel('left','y')
plot_widget.setLabel('right','x')

#Set grid
plot_widget.showGrid(x=True,y=True)

#Initialize scatter plot item
scatter = pg.ScatterPlotItem(size=10,pen=None,symbol='o')
plot_widget.addItem(scatter)

def update_plot():
    x=np.random.rand(100)*10
    y=np.random.rand(100)*10
    scatter.setData(x=x,y=y)

timer=pg.QtCore.QTimer()
timer.timeout.connect(update_plot)
timer.start(1000)

app.exec_()


