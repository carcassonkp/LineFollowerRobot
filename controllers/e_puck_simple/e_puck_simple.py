import cv2
import numpy as np
import math
from controller import Camera
from controller import Supervisor

from CellsGraph import CellsGraph
from PathFindingAStar import PathFindingAStar

#Declaring variables for global purposes

robot = None

cellsGraph = None
pathFinding = None
totalPath = []

currentCell = None

#Angles and distance errors
angleSafeRange = 0.05
distSafeArea = 0.1

#Wheel speed
left_wheel_speed = 0
right_wheel_speed = 0


max_speed = 6.28  # documentation vel
currentSpeed = 10

def run_robot():
    # Robot State
    robotCurrentState = 'ROTATING'
    timestep = int(robot.getBasicTimeStep())


    # Initialize motors
    left_motor = robot.getDevice("left wheel motor")
    left_motor.setPosition(float('inf'))
    left_motor.setVelocity(0.0)

    right_motor = robot.getDevice("right wheel motor")
    right_motor.setPosition(float('inf'))
    right_motor.setVelocity(0.0)


    # Initialize camera
    camera = robot.getDevice('camera')
    camera.enable(timestep)

    #Initialize GPS
    gps = robot.getDevice('gps')
    gps.enable(timestep)

    #Initialize Gyroscope
    #gyro = robot.getDevice('gyro')
    #gyro.enable(timestep)

    # Initialize IU
    inertialUnit = robot.getDevice('inertial unit')
    inertialUnit.enable(timestep)

    compass = robot.getDevice('compass')
    compass.enable(timestep)

    while robot.step(timestep) != -1:

        #Read Sensors
        gpsValues = gps.getValues()
        gpsValuesText = " {0:0.5f} , {1:0.5f}, {2:0.5f}".format(gpsValues[0],gpsValues[1],gpsValues[2])
        #print('POSITION:' + gpsValuesText)

        #roll,pitch,yaw = gyro.getRollPitchYaw()
        #print('roll: ' + str(roll) + ' pitch: ' + str(pitch) + ' yaw: ' + str(yaw))

        roll,pitch,yaw = inertialUnit.getRollPitchYaw()
        #print('roll: ' + str(roll) + ' pitch: ' + str(pitch) + ' yaw: ' + str(yaw))

        forward_vector = [math.cos(yaw), 0, -math.sin(yaw)]
        print('FORWARD VECTOR: {0}'.format(forward_vector))
        #forward_vector = compass.getValues()
        # Calculate the direction the robot is facing


        #print('roll: ' + str(roll) + ' pitch: ' + str(pitch) + ' yaw: ' + str(yaw))
        #print(forward_vector)

        #Process data

        if robotCurrentState == 'ROTATING':
            print('ROTATING TOWARDS: ' + currentCell.name)
            robotCurrentState = RotationBehaviour(forward_vector, left_motor,right_motor)
        else:
            right_motor.setVelocity(currentSpeed)
            left_motor.setVelocity(currentSpeed)

            robotCurrentState = MovementBehaviour(gpsValues, camera, left_motor,right_motor)

        print(robotCurrentState)

        #Actions


def MovementBehaviour(robotPosition,camera,right_motor,left_motor):

    global left_wheel_speed,right_wheel_speed

    nextCell = totalPath[0]
    euclideanVector = [nextCell.position[0] - robotPosition[0],
                          nextCell.position[1] - robotPosition[1],
                          nextCell.position[2] - robotPosition[2]]

    distance = np.linalg.norm(np.array(euclideanVector))
    print('DISTANCE TO NEXT POINT:' + str(distance))

    slowDistance = 0.1
    stopDistance = 0.05

    changeState = False

    if distance < stopDistance:
        global currentCell
        currentCell = totalPath.pop(0)
        changeState = True
    elif distance< slowDistance:
        right_motor.setVelocity(right_wheel_speed* distance)
        left_motor.setVelocity(left_wheel_speed* distance)
    else:
        FollowLineMovement(camera, left_motor,right_motor)

    if changeState == True:
        return 'ROTATING'
    else:
        return 'MOVING'





def FollowLineMovement(camera,left_motor,right_motor):

    # Camera Image Variables
    image_width = 640
    image_height = 240

    frame = np.frombuffer(camera.getImage(), dtype=np.uint8).reshape((camera.getHeight(), camera.getWidth(), 4))
    # save rgb
    # cv2.imwrite('processed_frame.jpg', frame)

    # Preprocessing
    gray = cv2.cvtColor(frame, cv2.COLOR_RGB2GRAY)
    # cv2.imwrite('processed_gray.jpg', gray)

    _, binary = cv2.threshold(gray, 150, 255, cv2.THRESH_BINARY)
    # cv2.imwrite('processed_binary.jpg', binary)

    # Canny edge detection
    edges = cv2.Canny(binary, 50, 150)
    # Hough line transform
    lines = cv2.HoughLinesP(edges, 1, np.pi / 180, threshold=50, minLineLength=50, maxLineGap=10)

    if lines is not None:
        center_x_total = 0
        average_center_x = 0
        center_y_total = 0
        count = 0

        # Iterate over all detected lines
        for line in lines:
            x1, y1, x2, y2 = line[0]

            # Calculate the center of the line segment
            center_x = (x1 + x2) // 2
            center_y = (y1 + y2) // 2

            center_x_total += center_x
            center_y_total += center_y
            count += 1

        if count > 0:
            # Calculate the average center point
            average_center_x = center_x_total // count
            average_center_y = center_y_total // count

            # Draw a circle at the average center point (for visualization)
        cv2.circle(frame, (average_center_x, average_center_y), 5, (0, 255, 0), -1)
    else:
        print('LINES NOT FOUND')

    cv2.imwrite('CENTER.jpg', frame)

    # CONTROL MOTORS
    # Find Error
    center_x = average_center_x  # Assuming you have obtained the average center x-coordinate
    desired_position = image_width / 2  # Assuming the desired position is the center of the image width
    ##print(f"Center X: {center_x}")
    error = center_x - desired_position
    # Set the proportional control gain
    Ke = 0.5
    Kp = 2

    # Calculate the individual wheel speeds

    global left_wheel_speed, right_wheel_speed
    left_wheel_speed = Kp - ((Ke * error) / desired_position)
    right_wheel_speed = Kp + ((Ke * error) / desired_position)
    left_motor.setVelocity(left_wheel_speed)
    right_motor.setVelocity(right_wheel_speed)

    if(left_wheel_speed > right_wheel_speed):
        print("AJUSTAR PARA A DIREITA")
    else:
        print('AJUSTAR PARA A ESQUERDA')
    ##print(f"left wheel: {left_wheel_speed}")
    ##print(f"right wheel: {right_wheel_speed}")

def RotationBehaviour(forwardVector,left_motor,right_motor):

    nextCell = totalPath[0]
    desiredOrientation = [nextCell.position[0]-currentCell.position[0], nextCell.position[1]-currentCell.position[1],
                          nextCell.position[2]-currentCell.position[2]]

    angle = angle_between(forwardVector,desiredOrientation)



    changeState = False


    if angle >= angleSafeRange:
        TurnRight(left_motor,right_motor)
        print('ROTATING RIGHT')
    elif angle <= -angleSafeRange:
        TurnLeft(left_motor,right_motor)
        print('ROTATING LEFT')
    else:
         changeState = True

    #print('TO: ' + nextCell.name)
    #print('ANGLE: ' + str(angle))

    if changeState == True:
        return 'MOVING'
    else:
        return 'ROTATING'


## https://stackoverflow.com/questions/2827393/angles-between-two-n-dimensional-vectors-in-python
def unit_vector(vector):
    """ Returns the unit vector of the vector.  """
    return vector / np.linalg.norm(vector)

def angle_between(v1, v2):
    """ Returns the angle in radians between vectors 'v1' and 'v2'::

            >>> angle_between((1, 0, 0), (0, 1, 0))
            1.5707963267948966
            >>> angle_between((1, 0, 0), (1, 0, 0))
            0.0
            >>> angle_between((1, 0, 0), (-1, 0, 0))
            3.141592653589793
    """
    v1_u = unit_vector(v1)
    v2_u = unit_vector(v2)
    #return np.arccos(np.clip(np.dot(v1_u, v2_u), -1.0, 1.0))
    return np.arctan2(np.cross(v2_u, v1_u), np.dot(v1_u, v2_u))[1]
##---------------------------------------------------------------------------------------

def TurnRight(left_motor,right_motor):
    right_motor.setVelocity(0.0)
    left_motor.setVelocity(0.2 * currentSpeed/2)

def TurnLeft( left_motor,right_motor):
    right_motor.setVelocity(0.2 *currentSpeed/2)
    left_motor.setVelocity(0.0)

def DefineInitialCells():

    randomStartCell = cellsGraph.GetRandomCell()

    randomPackageCell = None
    while True:
        randomPackageCell = cellsGraph.GetRandomCell()
        if (randomPackageCell != randomStartCell): break

    randomEndCell = None
    while True:
        randomEndCell = cellsGraph.GetRandomCell()
        if (randomEndCell != randomPackageCell): break

    return randomStartCell,randomPackageCell,randomEndCell

if __name__=='__main__':
    # Initialize Robot
    robot = Supervisor()

    #Initialize World Graph
    cellsGraph = CellsGraph(supervisor=robot)

    #Get random cells for path
    randomStartCell, randomPackageCell, randomEndCell = DefineInitialCells()
    print('Random cells: Start -> ' + str(randomStartCell.name) + ' Package -> ' +str(randomPackageCell.name) +
          ' End -> ' + str(randomEndCell.name) )

    #Get Paths Start-Package and Package-End
    pathFinding = PathFindingAStar(cellsGraph)

    ToPackagePath = pathFinding.FindPath(randomStartCell,randomPackageCell)
    pathFinding.DEBUGPrintPath(randomStartCell.name, randomPackageCell.name, ToPackagePath)

    ToEndPath= pathFinding.FindPath(randomPackageCell,randomEndCell)
    pathFinding.DEBUGPrintPath(randomPackageCell.name, randomEndCell.name, ToEndPath)

    totalPath = ToPackagePath + ToEndPath

    pathFinding.DEBUGPrintPath(randomStartCell.name, randomEndCell.name, totalPath)


    currentCell = randomStartCell




    TESTESTART = cellsGraph.worldCellsDict['C3']
    TESTEP  = cellsGraph.worldCellsDict['C9']
    TESTEND =  cellsGraph.worldCellsDict['C8']
    TESTEPATH1 = pathFinding.FindPath(TESTESTART,TESTEP)
    TESTPATH2 = pathFinding.FindPath(TESTEP,TESTEND)
    totalPath = TESTEPATH1 + TESTPATH2
    currentCell = TESTESTART


    #Start at random start cell position
    robotNode = robot.getFromDef('e-puck')
    translationField = robotNode.getField('translation')
    translationField.setSFVec3f(TESTESTART.position)


    run_robot()



