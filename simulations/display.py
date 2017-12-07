import serial
from pyqtgraph.Qt import QtGui, QtCore
import numpy as np
import pyqtgraph as pg

#QtGui.QApplication.setGraphicsSystem('raster')
app = QtGui.QApplication([])
#mw = QtGui.QMainWindow()
#mw.resize(800,800)

win = pg.GraphicsWindow(title="Plot")
win.resize(1000,600)
win.setWindowTitle('pyqtgraph example: Plotting')

pg.setConfigOptions(antialias=False)

p = win.addPlot(title="Updating plot")
curve = p.plot(pen='y')
data = np.zeros(1000)

ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.close()
ser = serial.Serial('/dev/ttyACM0', timeout=1)

p.setYRange(0, 5000)

def update():
    global curve, data, p
    for i in range(data.size - 1):
        data[i] = data[i+1]
    data[data.size - 1] = ser.readline()
    curve.setData(data)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(35)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
