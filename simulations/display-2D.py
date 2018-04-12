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

p4 = win.addPlot(title="Table")

# Number of points on the graph (3 anchors + the robots)
n = 6
s1 = pg.ScatterPlotItem(size=10, pen=pg.mkPen(None), brush=pg.mkBrush(255, 255, 255, 120))
pos = np.zeros(shape=(2,n))
pos[0][1] = 3000
pos[0][2] = 1500
pos[1][2] = 2000
lines = [1700,2800,940]
spots = [{'pos': pos[:,i], 'data': 1} for i in range(n)]
s1.addPoints(spots)
p4.addItem(s1)

ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.close()
ser = serial.Serial('/dev/ttyACM0', timeout=1)
ser.write('x\r'.encode('utf-8'))
ser.readline()
ser.write('bbpos\r'.encode('utf-8'))
ser.readline()
ser.readline()

def update():
    global curve1, curve2, data1, data2
    lines = ser.readline().split(",")
#    pos[0][3] = lines[3]
#    pos[1][3] = lines[4]
#    pos[0][4] = lines[6]
#    pos[1][4] = lines[7]
    pos[0][5] = lines[9]
    pos[1][5] = lines[10]

    spots = [{'pos': pos[:,i], 'data': 1, 'symbol': 0} for i in range(n-1)] + [{'pos': pos[:,n-1], 'data': 1, 'symbol': 1}]
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
