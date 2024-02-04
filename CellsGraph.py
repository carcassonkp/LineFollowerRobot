import random
from Cell import Cell
from controller import Supervisor

class CellsGraph:

    def __init__(self, supervisor):

        self.adjacentCellsNamesDict = {
            'C1': ['C7'],
            'C2': ['C3', 'C8'],
            'C3': ['C2', 'C4', 'C9'],
            'C4': ['C3', 'C5', 'C10'],
            'C5': ['C4', 'C11'],
            'C6': ['C12'],
            'C7': ['C1', 'C8', 'C15'],
            'C8': ['C2', 'C7', 'C9', 'C16'],
            'C9': ['C3', 'C8', 'C10', 'C17'],
            'C10': ['C4', 'C9', 'C11'],
            'C11': ['C5', 'C10'],
            'C12': ['C6', 'C13', 'C19'],
            'C13': ['C12', 'C14', 'C20'],
            'C14': ['C13'],
            'C15': ['C7', 'C21'],
            'C16': ['C8', 'C17', 'C22'],
            'C17': ['C9', 'C16', 'C23'],
            'C18': ['C19', 'C25'],
            'C19': ['C12', 'C18', 'C20'],
            'C20': ['C13', 'C19', 'C26'],
            'C21': ['C15', 'C22'],
            'C22': ['C16', 'C21', 'C23'],
            'C23': ['C17', 'C22', 'C24'],
            'C24': ['C23', 'C25'],
            'C25': ['C18', 'C24'],
            'C26': ['C20', 'C27'],
            'C27': ['C26']

        }


        # Name -> Cell Instance
        self.worldCellsDict = {}

        self.createCells(supervisor)

    def createCells(self,supervisor):

        maxCounter = 27

        #Creates instances while storing their positions positions
        for counter in range(1,maxCounter+1):
            newCellName = "C"+ str(counter)

            cellNode = supervisor.getFromDef(newCellName)
            cellPosition = cellNode.getField('translation').value

            newCell = Cell(newCellName,cellPosition)

            self.worldCellsDict[newCellName] = newCell

        #After creating the instances it connects them as adjacents
        for cellName in self.worldCellsDict.keys():

            adjacentCellsList = self.GetAdjacentCells(cellName)

            self.worldCellsDict[cellName].DefineAdjacentCells(adjacentCellsList)



        #DEBUG
        #print('TESTING CELL C16')
        #for adjacentCell in self.worldCellsDict['C16'].adjacentCells:
        #   tempX = str(adjacentCell.position[0])
        #   tempY = str(adjacentCell.position[1])
        #   tempZ = str(adjacentCell.position[2])
        #   print("NAME: " + adjacentCell.name + "\n" + " X -> " + tempX + " Y -> " + tempY + " Z -> " + tempZ + "\n\n")




    def GetAdjacentCells(self,cellName):

        tempAdjacentNamesList = self.adjacentCellsNamesDict[cellName]

        tempAdjacentCells = []
        for name in tempAdjacentNamesList:
            tempAdjacentCells.append(self.worldCellsDict[name])


        return tempAdjacentCells

    def GetCellInPosition(self,position):

        desiredCell = None

        for _,cell in self.worldCellsDict.items():
            if cell.position[0] == position[0] and cell.position[1] == position[1] and cell.position[2] == position[2]:
                desiredCell = cell
                break

        return desiredCell

    def GetRandomCell(self):
        _, randomCell = random.choice(list(self.worldCellsDict.items()))
        return randomCell