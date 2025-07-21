# coding=utf-8
#! /usr/bin/env python3
# import rospy
# import actionlib
import numpy as np  
import threading
from dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile
from time import sleep
import numpy as np
import re
import time
# import roslib; roslib.load_manifest('robotiq_2f_gripper_control')
# from robotiq_2f_gripper_control.msg import _Robotiq2FGripper_robot_output  as outputMsg


globalLockValue = threading.Lock()


def genCommand(char, command):
    """Update the command according to the character entered by the user."""  
    print(char)  
        
    if char == 'a':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 1
        command.rGTO = 1
        command.rSP  = 255
        command.rFR  = 150

    if char == 'r':
        command = outputMsg.Robotiq2FGripper_robot_output();
        command.rACT = 0

    if char == 'c':
        command.rPR = 255

    if char == 'o':
        command.rPR = 0   

    #If the command entered is a int, assign this value to rPRA
    try: 
        command.rPR = int(char)
        if command.rPR > 255:
            command.rPR = 255
        if command.rPR < 0:
            command.rPR = 0
    except ValueError:
        pass                    
        
    if char == 'f':
        command.rSP += 25
        if command.rSP > 255:
            command.rSP = 255
            
    if char == 'l':
        command.rSP -= 25
        if command.rSP < 0:
            command.rSP = 0

            
    if char == 'i':
        command.rFR += 25
        if command.rFR > 255:
            command.rFR = 255
            
    if char == 'd':
        command.rFR -= 25
        if command.rFR < 0:
            command.rFR = 0

    return command
        

def askForCommand(command):
    """Ask the user for a command to send to the gripper."""    

    currentCommand  = 'Simple 2F Gripper Controller\n-----\nCurrent command:'
    currentCommand += '  rACT = '  + str(command.rACT)
    currentCommand += ', rGTO = '  + str(command.rGTO)
    currentCommand += ', rATR = '  + str(command.rATR)
    currentCommand += ', rPR = '   + str(command.rPR )
    currentCommand += ', rSP = '   + str(command.rSP )
    currentCommand += ', rFR = '   + str(command.rFR )


    print (currentCommand)

    strAskForCommand  = '-----\nAvailable commands\n\n'
    strAskForCommand += 'r: Reset\n'
    strAskForCommand += 'a: Activate\n'
    strAskForCommand += 'c: Close\n'
    strAskForCommand += 'o: Open\n'
    strAskForCommand += '(0-255): Go to that position\n'
    strAskForCommand += 'f: Faster\n'
    strAskForCommand += 'l: Slower\n'
    strAskForCommand += 'i: Increase force\n'
    strAskForCommand += 'd: Decrease force\n'
    
    strAskForCommand += '-->'

    return input(strAskForCommand)

def RunPoint(move: DobotApiMove, point_list: list):
    move.JointMovJ(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5])
def MovePoint(move: DobotApiMove, point_list: list):
    move.MovJ(point_list[0], point_list[1], point_list[2],
              point_list[3], point_list[4], point_list[5])  

def WaitArrive(point_list):
    while True:
        is_arrive = True
        globalLockValue.acquire()
        if current_actual is not None:
            for index in range(4):
                if (abs(current_actual[index] - point_list[index]) > 1):
                    is_arrive = False
            if is_arrive:
                globalLockValue.release()
                return
        globalLockValue.release()
        sleep(0.001)

def ConnectRobot():
    try:
        ip = "192.168.5.1"
        dashboardPort = 29999
        movePort = 30003
        feedPort = 30004
        print("正在建立连接...")
        dashboard = DobotApiDashboard(ip, dashboardPort)
        move = DobotApiMove(ip, movePort)
        feed = DobotApi(ip, feedPort)
        print(">.<连接成功>!<")
        return dashboard, move, feed
    except Exception as e:
        print(":(连接失败:(")
        raise e
def GetFeed(feed: DobotApi):
    global current_actual
    global algorithm_queue
    global enableStatus_robot
    global robotErrorState
    hasRead = 0
    while True:
        data = bytes()
        while hasRead < 1440:
            temp = feed.socket_dobot.recv(1440 - hasRead)
            if len(temp) > 0:
                hasRead += len(temp)
                data += temp
        hasRead = 0
        feedInfo = np.frombuffer(data, dtype=MyType)
        if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
            globalLockValue.acquire()
            # Refresh Properties
            current_actual = feedInfo["tool_vector_actual"][0]
            algorithm_queue = feedInfo['run_queued_cmd'][0]
            enableStatus_robot = feedInfo['enable_status'][0]
            robotErrorState = feedInfo['error_status'][0]
            globalLockValue.release()
        sleep(0.001)


def ClearRobotError(dashboard: DobotApiDashboard):
    global robotErrorState
    dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
    while True:
        globalLockValue.acquire()
        if robotErrorState:
            numbers = re.findall(r'-?\d+', dashboard.GetErrorID())
            numbers = [int(num) for num in numbers]
            if (numbers[0] == 0):
                if (len(numbers) > 1):
                    for i in numbers[1:]:
                        alarmState = False
                        if i == -2:
                            print("机器告警 机器碰撞 ", i)
                            alarmState = True
                        if alarmState:
                            continue
                        for item in dataController:
                            if i == item["id"]:
                                print("机器告警 Controller errorid", i,
                                      item["zh_CN"]["description"])
                                alarmState = True
                                break
                        if alarmState:
                            continue
                        for item in dataServo:
                            if i == item["id"]:
                                print("机器告警 Servo errorid", i,
                                      item["zh_CN"]["description"])
                                break

                    choose = input("输入1, 将清除错误, 机器继续运行: ")
                    if int(choose) == 1:
                        dashboard.ClearError()
                        
                        dashboard.Continue()

        else:
            if int(enableStatus_robot) == 1 and int(algorithm_queue) == 0:
                dashboard.Continue()
        globalLockValue.release()


def reset():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 0
    pub.publish(command)
    rospy.sleep(0.1)

def close():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rPR = 255
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    pub.publish(command)
    print(command)
    rospy.sleep(0.1)

def open():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    command.rPR = 0   
    pub.publish(command)
    print(command)
    rospy.sleep(0.1)    

def activate():
    command = outputMsg.Robotiq2FGripper_robot_output();
    command.rACT = 1
    command.rGTO = 1
    command.rSP  = 255
    command.rFR  = 150
    pub.publish(command)
    rospy.sleep(0.1)

if __name__ == "__main__":

    # rospy.init_node('Robotiq2FGripperSimpleController222')
    
    # pub = rospy.Publisher('Robotiq2FGripperRobotOutput', outputMsg.Robotiq2FGripper_robot_output,queue_size = 10)
    # time.sleep(1)

    # command = outputMsg.Robotiq2FGripper_robot_output();
    # if not rospy.is_shutdown():
        # reset()
        # time.sleep(1)
        # activate()
        time.sleep(2)
        print('activate')
        dashboard, move, feed = ConnectRobot()
        feed_thread = threading.Thread(target=GetFeed, args=(feed,))
        feed_thread.daemon = True
        feed_thread.start()
        print("使bu能...")
        dashboard.DisableRobot()
        time.sleep(2)
        print("开始使能...")
        dashboard.EnableRobot()
        time.sleep(2)
        print("robot_activate")

        point1 = [173,-9,-89,-78,4.7,93.6]
        # p1=[-388.7,293.9,400,-93.6,-82.5,-7.8]
        # MovePoint(move,p1)
        RunPoint(move,point1)
        time.sleep(2)
        print('ready!')
        # close()
        time.sleep(1)
        point2 = [173.3,-18.5,-91.91,-85,3.9,107.8]
        RunPoint(move,point2)
        time.sleep(2)
        # open()
        time.sleep(1)
        RunPoint(move,point1)
        # MovePoint(move,p1)
        time.sleep(2)
        # close()
        time.sleep(2)
        point3 = [166,-1.6,-100,-76,21.7,96.6]    
        # p3 = [-211,457.9,367.3,-93,-82,-31]    
        RunPoint(move,point3)
        # MovePoint(move,p3)

        # open()
        # RunPoint(move,point1)
        










        
     

