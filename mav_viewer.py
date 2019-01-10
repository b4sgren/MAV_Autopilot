import numpy as np
import pyqtgraph as pg
import pyqtgraph.opengl as gl
import pyqtgraph.Vector as Vector

class MAV_Viewer:
    def __init__(self):
        self.application = pg.QtGui.QApplication([])
        self.window = gl.GLViewWidget()
        self.window.setWindowTitle('Flight Simulator')
        self.window.setGeometry(0, 0, 750, 750)
        grid = gl.GLGridItem()
        grid.scale(20, 20, 20)
        self.window.addItem(grid)
        self.window.setCameraPosition(distance=200)
        self.window.setBackgroundColor('k')
        self.window.show()
        self.window.raise_()
        self.plot_initialize = False
        self.points, self.mesh_colors = self.getMAVPoints()

    def getMAVPoints(self):
        points = np.array([[1, 1, 0],
                           [1, -1, 0],
                           [-1, -1, 0],
                           [-1, 1, 0],
                           [1, 1, -2],
                           [1, -1, -2],
                           [-1, -1, -2],
                           [-1, 1, -2],
                           [1.5, 1.5, 0],
                           [1.5, -1.5, 0],
                           [-1.5, -1.5, 0],
                           [-1.5, 1.5, 0]]).T
        scale = 10
        points = scale * points

        red = np.array([1.0, 0.0, 0.0, 1])
        green = np.array([0.0, 1.0, 0.0, 1])
        blue = np.array([0.0, 0.0, 1.0, 1])
        yellow = np.array([1.0, 1.0, 0.0, 1])
        mesh_colors = np.empty((12, 3, 4), dtype=np.float32)
        mesh_colors[0] = yellow
        mesh_colors[1] = yellow
        mesh_colors[2] = blue
        mesh_colors[3] = blue
        mesh_colors[4] = blue
        mesh_colors[5] = blue
        mesh_colors[6] = blue
        mesh_colors[7] = blue
        mesh_colors[8] = blue
        mesh_colors[9] = blue
        mesh_colors[10] = green
        mesh_colors[11] = green

        return points, mesh_colors
if __name__ == "__main__":
    simulator = MAV_Viewer()
    pg.QtGui.QApplication.instance().exec_()
