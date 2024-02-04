import numpy as np
import scipy
class PathNode:
    def __init__(self,cell):
        self.gCost = 99999999999
        self.hCost = 0
        self.fCost = 0
        self.parent = None
        self.cell = cell
        self.position = self.cell.position


