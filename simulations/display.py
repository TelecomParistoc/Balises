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

# p1 = win.addPlot(title="Updating plot")
# p2 = win.addPlot(title="Updating plot")
# p3 = win.addPlot(title="Updating plot")
p4 = win.addPlot(title="Table")

n = 5
s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
pos = np.zeros(shape=(2,n))
pos[0][1] = 3000
pos[0][2] = 1500
pos[1][2] = 2000
spots = [{'pos': pos[:,i], 'data': 1} for i in range(n)] + [{'pos': [0,0], 'data': 1}]
s1.addPoints(spots)
p4.addItem(s1)
#
# curve1 = p1.plot(pen='y')
# curve2 = p2.plot(pen='y')
# curve3 = p3.plot(pen='y')
data1 = np.zeros(1000)
data2 = np.zeros(1000)
data3 = np.zeros(1000)

ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.close()
ser = serial.Serial('/dev/ttyACM0', timeout=1)

# p1.setYRange(0, 5000)
# p2.setYRange(0, 5000)
# p3.setYRange(0, 5000)

def update():
    # global curve1, curve2, curve3, data1, data2, data3
    global pos, s1
    # for i in range(data1.size - 1):
    #     data1[i] = data1[i+1]
    #     data2[i] = data2[i+1]
    #     data3[i] = data3[i+1]
    lines = ser.readline().split(",")
    #
    # data1[data1.size - 1] = lines[0]
    # curve1.setData(data1)
    # data2[data2.size - 1] = lines[1]
    # curve2.setData(data2)
    # data3[data3.size - 1] = lines[2]
    # curve3.setData(data3)
    pos[0][3] = lines[0]
    pos[1][3] = lines[1]
    pos[0][4] = lines[2]
    pos[1][4] = lines[3]
    spots = [{'pos': pos[:,i], 'data': 1} for i in range(n)] + [{'pos': [0,0], 'data': 1}]
    s1.clear()
    s1.addPoints(spots)

timer = QtCore.QTimer()
timer.timeout.connect(update)
timer.start(35)

## Start Qt event loop unless running in interactive mode or using pyside.
if __name__ == '__main__':
    import sys
    if (sys.flags.interactive != 1) or not hasattr(QtCore, 'PYQT_VERSION'):
        QtGui.QApplication.instance().exec_()
