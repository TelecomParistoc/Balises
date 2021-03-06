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

p1 = win.addPlot(title="Updating plot")
p2 = win.addPlot(title="Updating plot")

curve1 = p1.plot(pen='y')
curve2 = p2.plot(pen='y')
data1 = np.zeros(1000)
data2 = np.zeros(1000)

ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.close()
ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.write('bbpos\r'.encode('utf-8'))
ser.readline()

p1.setYRange(0, 5000)
p2.setYRange(0, 5000)

def update():
    global curve1, curve2, data1, data2
    for i in range(data1.size - 1):
        data1[i] = data1[i+1]
        data2[i] = data2[i+1]
    lines = ser.readline().split(",")

    data1[data1.size - 1] = lines[5]
    curve1.setData(data1)
    data2[data2.size - 1] = lines[8]
    curve2.setData(data2)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(35)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
