import numpy as np
class Cell:


    def __init__(self,name,position):
        self.name = name
        self.position = position
        self.gCost = 99999999
        self.hCost = 0
        self.fCost = 0
        self.parent = None

    def DefineAdjacentCells(self,adjacentCells):
        self.adjacentCells = adjacentCells


    def CalculateHCost(self, endPosition):

        startVector = np.array(self.position)
        endVector = np.array(endPosition)

        self.hCost = np.linalg.norm(endVector - startVector)

        print(self.hCost)

    def CalculateFCost(self):
        self.fCost = self.gCost + self.hCost