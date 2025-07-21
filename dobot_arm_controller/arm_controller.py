import re
import sys
import json
import time
import threading
import numpy as np
from time import sleep
from loguru import logger

from dobot_arm_controller.dobot_api import DobotApiDashboard, DobotApi, DobotApiMove, MyType, alarmAlarmJsonFile

class DobotArmController:
    """
    机器人控制类，用于管理与Dobot机器人的连接和操作。

    属性:
        ip_address (str): 机器人的IP地址。
        dashboardPort (int): 用于仪表板API的端口。
        movePort (int): 用于移动API的端口。
        feedPort (int): 用于反馈API的端口。
        dashboard (DobotApiDashboard): 仪表板API实例。
        move (DobotApiMove): 移动API实例。
        feed (DobotApi): 反馈API实例。
        current_actual (list): 当前实际坐标。
        algorithm_queue (int): 算法队列状态。
        enableStatus_robot (int): 机器人使能状态。
        robotErrorState (bool): 机器人错误状态。
        globalLockValue (threading.Lock): 线程锁。
    """

    def __init__(self, ip_address='192.168.5.1'):
        """
        初始化DobotArmController实例。

        参数:
            ip_address (str): 机器人的IP地址。
        """
        self.ip_address = ip_address
        self.dashboardPort = 29999
        self.movePort = 30003
        self.feedPort = 30004

        self.dashboard = None
        self.move = None
        self.feed = None

        self.current_actual = [-1]
        self.actual_quaternion = [-1]
        self.algorithm_queue = -1
        self.enableStatus_robot = -1
        self.robotErrorState = False
        self.robot_mode = 1
        # self.actual_transform = math3d.Transform()

        self.globalLockValue = threading.Lock()
        self.connect()

        with open('config/pick_place.json', 'r', encoding='utf-8') as file:
            self.pick_place_json = json.load(file)

    def connect(self):
        """
        建立与机器人的连接。
        """
        try:
            logger.info("正在建立连接...")
            self.dashboard = DobotApiDashboard(self.ip_address, self.dashboardPort)
            self.move = DobotApiMove(self.ip_address, self.movePort)
            self.feed = DobotApi(self.ip_address, self.feedPort)
            logger.info(">.<连接成功>!<")

            # 坐标系初始化
            self.set_tool_user_index()

            # 夹爪IO清零
            # self.dashboard.ToolDO(1, False)
            # self.dashboard.ToolDO(2, False)

            # 开启监控线程
            self.feed_thread = threading.Thread(target=self.get_feed)
            self.feed_thread.daemon = True
            self.feed_thread.start()

            self.error_thread = threading.Thread(target=self.clear_robot_error)
            self.error_thread.daemon = True
            self.error_thread.start()
        except Exception as e:
            logger.info(":(连接失败:(")
            raise e

    def set_payload_mass(self, mass: float):
        """设置末端质量

        Args:
            mass (float): 末端质量
        """

        self.dashboard.PayLoad(weight=mass, inertia=1.5)

    def set_tool_user_index(self, tool_index=0, user_index=0):
        self.dashboard.Tool(tool_index)
        self.dashboard.User(user_index)

    def set_gripper(self, cmd: str):
        if cmd=='open':
            self.dashboard.ToolDO(1, True)
            time.sleep(0.1)
            self.dashboard.ToolDO(1, False)
        elif cmd == 'close':
            self.dashboard.ToolDO(2, True)
            time.sleep(0.1)
            self.dashboard.ToolDO(2, False)
        else:
            logger.error(f'unknown gripper command: {cmd}')

        time.sleep(3)

        return True

    def run_point(self, point_list):
        """
        运行至指定点

        参数:
            point_list (list): 目标点的坐标[x, y, z, r, p, y]，单位：m，radians
        """

        self.dashboard.ResetRobot()
        logger.debug(np.degrees(point_list[3]))
        result = self.move.MovJ(point_list[0]*1000.0, point_list[1]*1000.0, point_list[2]*1000.0,
                       np.degrees(point_list[3]), np.degrees(point_list[4]), np.degrees(point_list[5]))
        return result

    def get_tool_transform(self, tool_name: str):
        tool_trans_array = self.pick_place_json['coordinate_system']['tool'][tool_name]['trans']
        trans = self.get_transform(tool_trans_array)

        return trans
    
    def get_user_transform(self, user_name: str):
        user_trans_array = self.pick_place_json['coordinate_system']['user'][user_name]['trans']
        trans = self.get_transform(user_trans_array)

        return trans
    
    def joint_movj(self, joint_angles: list):
        # 执行MoveJ动作
        self.dashboard.ResetRobot()
        self.move.JointMovJ(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint_angles[5])
        # 等待执行完成（阻塞）
        return True

    def get_feed(self):
        """
        获取反馈信息，并更新全局变量。
        """

        hasRead = 0
        while True:
            data = bytes()
            while hasRead < 1440:
                temp = self.feed.socket_dobot.recv(1440 - hasRead)
                if len(temp) > 0:
                    hasRead += len(temp)
                    data += temp
            hasRead = 0
            feedInfo = np.frombuffer(data, dtype=MyType)
            # print(feedInfo)
            if hex((feedInfo['test_value'][0])) == '0x123456789abcdef':
                self.globalLockValue.acquire()
                self.current_actual = feedInfo["tool_vector_actual"][0]
                self.actual_quaternion = feedInfo["actual_quaternion"][0]
                self.algorithm_queue = feedInfo['run_queued_cmd'][0]
                self.enableStatus_robot = feedInfo['enable_status'][0]
                self.robotErrorState = feedInfo['error_status'][0]
                self.actual_joint_angle = feedInfo['q_actual']
                self.robot_mode = feedInfo['robot_mode']
                # orient = math3d.quaternion.UnitQuaternion(self.actual_quaternion[0], self.actual_quaternion[1], self.actual_quaternion[2], self.actual_quaternion[3])
                # orient = math3d.Orientation.new_euler((np.radians(self.current_actual[5]), np.radians(self.current_actual[4]), np.radians(self.current_actual[3])), "ZYX")
                pos = [self.current_actual[0]/1000.0, self.current_actual[1]/1000.0, self.current_actual[2]/1000.0]
                self.globalLockValue.release()
            sleep(0.001)
 
    def wait_arrive(self, point_list):
        """
        等待到达指定点。

        参数:
            point_list (list): 目标点的坐标列表。
        """

        # 将目标值转换成和机械臂一致的格式（毫米，角度制）
        a_point_list = [point_list[0]*1000.0, point_list[1]*1000.0, point_list[2]*1000.0,
                       np.degrees(point_list[3]), np.degrees(point_list[4]), np.degrees(point_list[5])]

        while True:
            is_arrive = True
            self.globalLockValue.acquire()
            if self.current_actual is not None:
                for index in range(4):
                    if (abs(self.current_actual[index] - a_point_list[index]) > 1):
                        is_arrive = False
                if is_arrive:
                    self.globalLockValue.release()
                    return
            self.globalLockValue.release()
            sleep(0.001)

    def clear_robot_error(self):
        """
        清除机器人的错误。
        """

        dataController, dataServo = alarmAlarmJsonFile()    # 读取控制器和伺服告警码
        while True:
            self.globalLockValue.acquire()
            if self.robotErrorState:
                numbers = re.findall(r'-?\d+', self.dashboard.GetErrorID())
                numbers = [int(num) for num in numbers]
                if (numbers[0] == 0):
                    if (len(numbers) > 1):
                        for i in numbers[1:]:
                            alarmState = False
                            if i == -2:
                                logger.info(f"机器告警 机器碰撞：{i}")
                                alarmState = True
                            if alarmState:
                                continue
                            for item in dataController:
                                if i == item["id"]:
                                    logger.info(f'机器告警 Controller error id: {i}, description: {item["zh_CN"]["description"]}')
                                    alarmState = True
                                    break
                            if alarmState:
                                continue
                            for item in dataServo:
                                if i == item["id"]:
                                    logger.info(f'机器告警 Servo error id: {i}, description: {item["zh_CN"]["description"]}')
                                    break

                        # choose = input("输入1, 将清除错误, 机器继续运行: ")
                        # if int(choose) == 1:
                        #     self.dashboard.ClearError()
                        #     sleep(0.01)
                        #     self.dashboard.Continue()

            else:
                if int(self.enableStatus_robot) == 1 and int(self.algorithm_queue) == 0:
                    self.dashboard.Continue()
            self.globalLockValue.release()
            sleep(5)

if __name__ == '__main__':
    logger.remove()  # 移除默认的日志配置
    logger.add(sys.stderr, level="INFO")  # 只记录 'INFO' 及以上级别的日志

    # 设定机械臂端口
    robot = DobotArmController("192.168.5.1")

    logger.info("开始使能...")
    # robot.dashboard.EnableRobot()
    logger.info("完成使能:)")

    robot.move.MovL(-198.4362, -146.4971, 407.6227, 179.5018, -1.0094, -89.2702)
    robot.move.MovL(-198.4362, -146.4971, 377.6227, 179.5018, -1.0094, -89.2702)
    
