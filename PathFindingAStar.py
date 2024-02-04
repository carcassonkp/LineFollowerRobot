from collections import deque

import numpy as np

from PathNode import PathNode
class PathFindingAStar:

    def __init__(self, cellsGraph):
        self.cellsGraph = cellsGraph



    def FindPath(self, startingCell, endCell):



        #startingCell = self.cellsGraph.GetCellInPosition(startingPos)
        #endCell = self.cellsGraph.GetCellInPosition(endPos)

        startingCell.gCost = 0
        startingCell.CalculateHCost(endCell.position)
        startingCell.CalculateFCost()

        if startingCell == endCell: return


        openCells = deque()
        closedCells = set()

        openCells.append(startingCell)

        while len(openCells) > 0:

            currentCell = min(openCells, key=lambda x: x.hCost)
            openCells.remove(currentCell)
            closedCells.add(currentCell)

            if currentCell == endCell:
                return  self.GetPath(startingCell, endCell);

            for adjacentCell in currentCell.adjacentCells:
                if adjacentCell not in closedCells:
                    newMovementCostToNeighbour = currentCell.gCost + self.GetDistanceBetweenCells(currentCell, adjacentCell);

                    if newMovementCostToNeighbour < adjacentCell.gCost or adjacentCell not in openCells:
                        adjacentCell.HCost = self.GetDistanceBetweenCells(adjacentCell, endCell)
                        adjacentCell.parent = currentCell
                        adjacentCell.GCost = newMovementCostToNeighbour
                        adjacentCell.CalculateFCost()

                        if adjacentCell not in openCells:
                            openCells.append(adjacentCell)

    def GetPath(self,startingCell, endCell):
        path = []
        currentNode = endCell

        while currentNode != startingCell:

            path.append(currentNode)
            currentNode = currentNode.parent

        path.reverse()

        return path

    def GetDistanceBetweenCells(self, originCell, endCell):

        origin = np.array(originCell.position)
        end = np.array(endCell.position)

        return np.linalg.norm(end - origin)

    def DEBUGPrintPath(self,STARTTESTNAME,ENDTESTNAME,TESTPATH ):
        print('PATH FROM ' + STARTTESTNAME + '--> ' + ENDTESTNAME + '\n')

        temp = ''
        for c in TESTPATH:
            temp2 = ' --> ' + c.name
            temp += temp2
        print(temp)
        print()