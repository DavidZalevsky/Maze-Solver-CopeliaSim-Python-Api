import sim
import math
import time
import numpy as np
import random

#-----------------------Connecting with copeliasim via API-----------------------

sim.simxFinish(-1)
clientID = sim.simxStart('127.0.0.1',19997,True,True,5000,5)

if clientID != -1:
    print("Connected with API server")
else:
    print("Not connected with API server")

#-----------------------Handlers-----------------------
#Movement
errorCode,left_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_leftMotor',sim.simx_opmode_oneshot_wait)
errorCode,right_motor_handle = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_rightMotor',sim.simx_opmode_oneshot_wait)

#Sensor
errorCode,proxmitysensor5 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor5',sim.simx_opmode_oneshot_wait) #Right front
errorCode,proxmitysensor4 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor4',sim.simx_opmode_oneshot_wait) #Left front
errorCode,proxmitysensor8 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor8',sim.simx_opmode_oneshot_wait) #Right panel
errorCode,proxmitysensor1 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor1',sim.simx_opmode_oneshot_wait) #Left panel
errorCode,proxmitysensor12 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor12',sim.simx_opmode_oneshot_wait) #Right Bottom
errorCode,proxmitysensor13 = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx_ultrasonicSensor13',sim.simx_opmode_oneshot_wait) #Left Bottom

#Object
errorCode,robot = sim.simxGetObjectHandle(clientID,'Pioneer_p3dx',sim.simx_opmode_oneshot_wait) #Robot

#-----------------------Sensor Initializing-----------------------
errorCode,detectionState5,detectedPoint5,detectedObjectHandle5,detectedSurfaceNormalVector5 = sim.simxReadProximitySensor(clientID,proxmitysensor5,sim.simx_opmode_streaming)
errorCode,detectionState4,detectedPoint4,detectedObjectHandle4,detectedSurfaceNormalVector4 = sim.simxReadProximitySensor(clientID,proxmitysensor4,sim.simx_opmode_streaming)
errorCode,detectionState1,detectedPoint1,detectedObjectHandle1,detectedSurfaceNormalVector1 = sim.simxReadProximitySensor(clientID,proxmitysensor1,sim.simx_opmode_streaming)
errorCode,detectionState8,detectedPoint8,detectedObjectHandle8,detectedSurfaceNormalVector8 = sim.simxReadProximitySensor(clientID,proxmitysensor8,sim.simx_opmode_streaming)
errorCode,detectionState12,detectedPoint12,detectedObjectHandle12,detectedSurfaceNormalVector12 = sim.simxReadProximitySensor(clientID,proxmitysensor12,sim.simx_opmode_streaming)
errorCode,detectionState13,detectedPoint13,detectedObjectHandle13,detectedSurfaceNormalVector13 = sim.simxReadProximitySensor(clientID,proxmitysensor13,sim.simx_opmode_streaming)

#-----------------------Robot Movement-----------------------
def robot_forward():
    sim.simxSetJointTargetVelocity(clientID,left_motor_handle,3,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID,right_motor_handle,3,sim.simx_opmode_blocking)

def robot_stop():
    sim.simxSetJointTargetVelocity(clientID,left_motor_handle,0,sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID,right_motor_handle,0,sim.simx_opmode_blocking)

def robot_rotate_right():
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, 0.5, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, -0.5, sim.simx_opmode_blocking)

def robot_rotate_left():
    sim.simxSetJointTargetVelocity(clientID, left_motor_handle, -0.1, sim.simx_opmode_blocking)
    sim.simxSetJointTargetVelocity(clientID, right_motor_handle, 0.1, sim.simx_opmode_blocking)

def robot_rotate_angle(rotate_angle):
    robot_stop()
    angle_initial = robot_angle_calculation()  # Read actual robot angle orientation

    print("Angle initial original: ",int(angle_initial))

    #Processing data so robot will rotate to global multiples of 90 degree

    angle_initial_quarter = int(angle_initial / 90) * 90  # Fixed angle value
    if (angle_initial - angle_initial_quarter) < 45:
        angle_initial = angle_initial_quarter
    else:
        angle_initial = angle_initial_quarter + 90


    print("Angle initial: ", int(angle_initial))

    if rotate_angle > 0: #right turn
        while(True):
            robot_rotate_right()
            angle_now = robot_angle_calculation()
            print("Angle now: ",int(angle_now))

            if angle_initial == 270:    #jump near 270-360
                if rotate_angle == 90:
                    if angle_now < 225:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break
                elif rotate_angle == 180:
                    if angle_now > 90 and angle_now < 225:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if angle_initial == 360:  # jump near 360-90
                if rotate_angle == 90:
                    if angle_now < 315 and angle_now > 90:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break
                elif rotate_angle == 180:
                    if angle_now > 180 and angle_now < 315:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if angle_initial == 180:  # jump near 360/0
                if rotate_angle == 180:
                    if angle_now < 135:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if ((angle_now-angle_initial) >= rotate_angle) and angle_initial != 270 and angle_initial != 360:
                robot_forward()
                print("Robot angle actual: ", robot_angle_calculation())
                break

    else: #left turn
        rotate_angle = abs(rotate_angle)
        while (True):
            robot_rotate_left()
            angle_now = robot_angle_calculation()
            print("Angle now: ", int(angle_now))

            if angle_initial == 90:    #jump near 90-0
                if rotate_angle == 90:
                    if angle_now > 135:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break
                elif rotate_angle == 180:
                    if angle_now > 135 and angle_now < 270:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if angle_initial == 0:  # jump near 0-270
                if rotate_angle == 90:
                    if angle_now < 270 and angle_now > 45:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break
                elif rotate_angle == 180:
                    if angle_now > 45 and angle_now < 180:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if angle_initial == 180:  # jump near 0/360
                if rotate_angle == 180:
                    if angle_now > 225:
                        robot_forward()
                        print("Robot angle actual: ", robot_angle_calculation())
                        break

            if (angle_initial - angle_now) >= rotate_angle and angle_initial != 0 and angle_initial != 90:
                robot_forward()
                print("Robot angle actual: ", robot_angle_calculation())
                break

def robot_path_riding(path_to_ride):
    # example input path_to_ride = [(0,1), (1,1), (2,1)]

    for i in range (0, len(path_to_ride)):
        robot_matrix_movement(path_to_ride[i])
        robot_labirynth_matrix_position_discrete()

#-----------------------Robot sensors-----------------------
def robot_proximity_front(): #2 front sensors
    global always_right_status

    errorCode, detectionState5, detectedPoint5, detectedObjectHandle5, detectedSurfaceNormalVector5 = sim.simxReadProximitySensor(clientID, proxmitysensor5, sim.simx_opmode_buffer)
    errorCode, detectionState4, detectedPoint4, detectedObjectHandle4, detectedSurfaceNormalVector4 = sim.simxReadProximitySensor(clientID, proxmitysensor4, sim.simx_opmode_buffer)

    try:
        if always_right == True:
            if 0.1 < (detectedPoint4[2] or detectedPoint5[2]) < 0.3:  # Sensor range 1 (object is far) -> 0.1 (object is very close to sensor)
                print("Detected objent in |FRONT|")
                return (True)
            else:
                print("No object in |FRONT|")
                return (False)
    except:
        if detectionState4 or detectionState5 == True:  # Detected obj in alteast 1 of 2 sensors
            print("Detected object in |FRONT|")
            return (True)
        else:
            print("No object in |FRONT|")
            return (False)

def robot_proximity_right(): #right sensor
    errorCode, detectionState8, detectedPoint8, detectedObjectHandle8, detectedSurfaceNormalVector8 = sim.simxReadProximitySensor(clientID, proxmitysensor8, sim.simx_opmode_buffer)

    if detectionState8 == True:
        #print("Detected object in |RIGHT|")
        return(True)
    else:
        #print("No object in |RIGHT|")
        return(False)


def robot_proximity_left(): #left sensor
    errorCode, detectionState1, detectedPoint1, detectedObjectHandle1, detectedSurfaceNormalVector1 = sim.simxReadProximitySensor(clientID, proxmitysensor1, sim.simx_opmode_buffer)
    if detectionState1 == True:
        #print("Detected object in |LEFT|")
        return(True)
    else:
        #print("No object in |LEFT|")
        return(False)

def robot_proximity_back(): #2 back sensors
    errorCode, detectionState12, detectedPoint12, detectedObjectHandle12, detectedSurfaceNormalVector12 = sim.simxReadProximitySensor(clientID, proxmitysensor12, sim.simx_opmode_buffer)
    errorCode, detectionState13, detectedPoint13, detectedObjectHandle13, detectedSurfaceNormalVector13 = sim.simxReadProximitySensor(clientID, proxmitysensor13, sim.simx_opmode_buffer)

    if detectionState12 or detectionState13 == True:    #Sensor range 1 (object is far) -> 0.1 (object is very close to sensor)
        #print("Detected objent in |BACK|")
        return(True)
    else:
        #print("No object in |BACK|")
        return(False)

def robot_all_sensors():
    return([robot_proximity_front(),robot_proximity_right(),robot_proximity_back(),robot_proximity_left()])

# -----------------------Calculations-----------------------
def robot_angle_calculation():
    NewMax = 0 #new range
    NewMin = 360

    OldMin = 0 #old range shifted
    OldMax = 2 * math.pi

    returnCode, OldValue = sim.simxGetObjectOrientation(clientID, robot, -1, sim.simx_opmode_blocking)

    #print(OldValue)

    OldValue[2]  = OldValue[2]  + math.pi #added pi so value from initial range (-pi | 0 | pi) is always positive
    NewValue = (((OldValue[2]  - OldMin) * (NewMax - NewMin)) / (OldMax - OldMin)) + NewMin

    return(NewValue)

def robot_position():
    temp = sim.simxGetObjectPosition(clientID,robot,-1,sim.simx_opmode_blocking)
    return(temp[1][0],temp[1][1])   #global x,y robot position

def robot_labirynth_matrix_position_discrete():
    robot_labirynth_position = []
    x,y = robot_position()

    x_axis = np.arange(start=-3, stop=5, step=1)
    y_axis = np.arange(start=-3, stop=5, step=1)

    for i in range(8):
        if x < x_axis[i]:
            robot_labirynth_position.append(i)
            break

    for i in range(8):
        if y < y_axis[i]:
            robot_labirynth_position.append(i)
            break

    return(robot_labirynth_position) #labirynth square x,y positions

def robot_matrix_movement(movement_direction_position_discrete):
    x_robot_discrete,y_robot_discrete = robot_labirynth_matrix_position_discrete()

    if x_robot_discrete > movement_direction_position_discrete[0]:
        movement = "LEFT"
    if x_robot_discrete < movement_direction_position_discrete[0]:
        movement = "RIGHT"
    if y_robot_discrete < movement_direction_position_discrete[1]:
        movement = "UP"
    if y_robot_discrete > movement_direction_position_discrete[1]:
        movement = "DOWN"

    print("before", robot_angle_calculation())

    #rotating towards movement square target
    if movement == "DOWN":
        robot_rotate_right()
        while (True):
            robot_actual_angle = robot_angle_calculation()
            if robot_actual_angle > 268 and robot_actual_angle < 272:
                robot_stop()
                break

    if movement == "UP":
        robot_rotate_right()
        while (True):
            robot_actual_angle = robot_angle_calculation()
            if robot_actual_angle > 88 and robot_actual_angle < 92:
                robot_stop()
                break

    if movement == "LEFT":
        robot_rotate_right()
        while (True):
            robot_actual_angle = robot_angle_calculation()
            print(robot_actual_angle)
            if robot_actual_angle > 358 or robot_actual_angle < 2:
                robot_stop()
                break

    if movement == "RIGHT":
        robot_rotate_right()
        while (True):
            robot_actual_angle = robot_angle_calculation()
            if robot_actual_angle > 178 and robot_actual_angle < 182:
                robot_stop()
                break

    print("after", robot_angle_calculation())

    #riding to the middle of the square
    position_middle = square_center_position_matrix[movement_direction_position_discrete[0]][movement_direction_position_discrete[1]] #square center
    robot_forward()

    while(True):
        position = robot_position() #actual robot position

        if movement == "RIGHT":
            if position[0] > position_middle[0]:
                break

        if movement == "LEFT":
            if position[0] < position_middle[0]:
                break

        if movement == "UP":
            if position[1] > position_middle[1]:
                break

        if movement == "DOWN":
            if position[1] < position_middle[1]:
                break

    #ending ride in the middle of the square
    robot_stop()


def labirynth_square_center_positions():
    global square_center_position_matrix
    square_center_position_matrix = [[0 for x in range(8)] for y in range(8)]
    x_position = -3.5
    for x_axis_index in range(8):
        y_position = -3.5
        for y_axis_index in range(8):
            square_center_position_matrix[x_axis_index][y_axis_index] = x_position, y_position,1.3865e-01
            y_position = y_position + 1
        x_position = x_position + 1

def simulation_start():
    sim.simxStartSimulation(clientID,sim.simx_opmode_oneshot)   #Starts simulation
    robot_stop()
    # variable initializing
    labirynth_square_center_positions()

# -----------------------Algorithms----------------------

def algoritm_always_right():
    global always_right_status
    always_right_status = True

    robot_stop()
    robot_forward()
    while (True):
        if robot_proximity_front() == True:  # wall ahead
            if robot_proximity_right() == False:  # no wall on the right
                robot_rotate_angle(90)
            elif robot_proximity_left() == False:
                robot_rotate_angle(-90)
            else:
                robot_rotate_angle(180)

        if robot_proximity_right() == False:  # no wall on right
            time.sleep(0.75)  # DEPENDS ON ROBOT SPEED!!! For Ola suggested 1.5
            robot_rotate_angle(90)
            while (True):
                if robot_proximity_right() == True:
                    break


def algorithm_rrt():
    # Initializing
    main_branch_array = [[7, 0]]

    while (True):
        new_tree_element, movement_option = create_new_tree_orb(
            main_branch_array)  # New tree element is aray of new tree orb position (x,y maze discrite), movement option is number of possible robot movement

        if movement_option == 1:  # Single path
            main_branch_array.append(new_tree_element[0])  # Adding new tree element to main branch

        if movement_option >= 2:  # Multi path
            first_branch = main_branch_array[-1]  # Creating first child branch
            first_branch = [first_branch, new_tree_element[0]]

            second_branch = main_branch_array[-1]  # Creating second child branch
            second_branch = [second_branch, new_tree_element[1]]

            if movement_option == 3:
                third_branch = main_branch_array[-1]
                third_branch = [third_branch, new_tree_element[2]]
                child_branch_array = [first_branch, second_branch, third_branch]

            if movement_option == 2:
                child_branch_array = [first_branch, second_branch]  # Array of all child branch

            while (True):
                chosen_child_branch_array = random.choice(child_branch_array)  # Choosing which branch we wanna expand
                # chosen_child_branch_array = child_branch_array[0]

                new_tree_element_child, movement_option_child = create_new_tree_orb(
                    chosen_child_branch_array)  # Checking movement option in new branch

                if movement_option_child == 1:  # Single path
                    chosen_child_branch_array.append(
                        new_tree_element_child[0])  # Adding new tree element to main branch

                if movement_option_child == 0:  # Dead end, child dies
                    child_branch_array.remove(chosen_child_branch_array)
                    child_branch_array[0].pop(0)
                    main_branch_array = main_branch_array + child_branch_array[0]
                    break

                # print(child_branch_array)
                # time.sleep(1)

        if [0, 7] in main_branch_array:
            sim.simxSetObjectPosition(clientID, robot, -1, square_center_position_matrix[7][0],
                                      sim.simx_opmode_oneshot)  # Teleporting back robot
            # sim.simxSetObjectOrientation(clientID,robot,-1,(8.7721e+01,-8.7390e+01,8.7715e+01),sim.simx_opmode_oneshot)
            robot_stop()
            main_branch_array.remove([7, 0])
            break

    print(main_branch_array)
    robot_path_riding(main_branch_array)

# -----------------------RRT Methods-----------------------

def rrt_wall_scanning(location): #Scanning in 4 direction
    x,y = location
    scanning_position = square_center_position_matrix[x][y]
    sim.simxSetObjectPosition(clientID, robot, -1, scanning_position, sim.simx_opmode_oneshot) #Teleporting robot
    #sim.simxSetObjectOrientation(clientID, robot, -1, (8.7721e+01, 0, 8.7715e+01), sim.simx_opmode_oneshot) #Stick orientation
    robot_stop()
    return(robot_all_sensors())

def create_new_tree_orb(main_branch_array): #Creating new tree orb
    movement_option = 0
    new_tree_element = []
    sensors_status = (rrt_wall_scanning(main_branch_array[-1])) #Array of 4 sensor status True/False
    for sensor_direction in range(4):  # Checking each 4 sensor direction
        if sensors_status[sensor_direction] == False:  # Free space detected
            x = main_branch_array[-1][0]  # Previous main branch x position
            y = main_branch_array[-1][1]  # Previous main branch y position

            if sensor_direction == 0:  # front sensor
                if ([x, y + 1] in main_branch_array) == False:
                    new_tree_element.append([x, y + 1])
                    movement_option = movement_option + 1

            if sensor_direction == 1:  # right sensor
                if ([x + 1, y] in main_branch_array) == False:
                    new_tree_element.append([x + 1, y])
                    movement_option = movement_option + 1

            if sensor_direction == 2:  # bottom sensor
                if ([x, y - 1] in main_branch_array) == False:
                    new_tree_element.append([x, y - 1])
                    movement_option = movement_option + 1

            if sensor_direction == 3:  # left sensor
                if ([x - 1, y] in main_branch_array) == False:
                    new_tree_element.append([x - 1, y])
                    movement_option = movement_option + 1

    return(new_tree_element, movement_option)

# -----------------------Robot command library-----------------------
# --- Movement ---
#robot_forward()
#robot_stop()
#robot_rotate_angle(x) #where x is: -180,-90,90,180 degree

# --- Sensor ---
#robot_proximity_front() #return true or false  ###True if wall detected otherwise false
#robot_proximity_right() #return true or false
#robot_proximity_left() #return true or false
#robot_proximity_back() #return true or false
#robot_all_sensors() #return status of: front,right,down,left sensor

# --- Simulation ---
#robot_angle_calculation() #global robot orientation
#robot_position() #global robot x,y position
#robot_labirynth_matrix_position_discrete() #return robot in matrix position #bottom left 0 | 0

# --- Global Variable ---
#square_center_position_matrix matrix of all square center positions, bottom left -3.5 | -3.5


# --- RRT Methods ---
#rrt_wall_scanning(x,y) #return wall proximity in x,y discrete maze, clockwise front,right,bottom,left true and false statments
#robot_path_riding(maze_matrix_discrete) #start robot movement via maze matrix trace

# --- Avaible algorithms
#algoritm_always_right() #choose one of the algorithm
#algorithm_rrt()

# -----------------------Main program-----------------------
simulation_start()

algorithm_rrt()
