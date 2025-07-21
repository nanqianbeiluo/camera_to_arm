from tkinter import NO
from .libs.dobot_api import DobotApiDashboard, DobotApiFeedBack, DobotApiDashMove
from pymodbus.client.sync import ModbusSerialClient
from time import sleep
import re
import socket

# 常量定义
SLAVE_ID_US = 2
SLAVE_ID_GRIPPER = 1
JOINTS_LIMIT = [0.1] * 6  # 关节运动的误差限制
POINT_LIMIT = [0.1] * 6   # 直线运动的误差限制
GRIPPER_AXIS_LIMIT = 1.0  # 夹爪轴的误差限制
GRIPPER_FORCE_LIMIT = 0.1 # 夹爪力的误差限制
ACC = 100                 # 加速度
VEL = 3                   # 速度
DOBOT_IP = '192.168.5.1'  # Dobot的IP地址
DOBOT_PORT = 29999        # Dobot的端口
DOBOT_MOVEPORT = 30004    # Dobot的运动端口
DOBOT_FEEDPORT = 30005    # Dobot的反馈端口


class arm:
    def __init__(self, simulation_mode=False) -> None:
        # 初始化Dobot和Modbus客户端
        self.dashboard = None
        self.__dobot_feed = None
        self.is_connected = False
        self.__dobot = None
        self.__mb = None
        self.simulation_mode = simulation_mode
        
        # 初始化模拟状态
        self._simulated_joints = [0, -45, 135, 0, -90, 0]
        self._simulated_pose = [300, 0, 200, 0, 0, 0]
        self._simulated_gripper_pos = 0.0
        
        if not simulation_mode:
            try:
                self.__dobot = DobotApiDashboard(DOBOT_IP, DOBOT_PORT)
                self.__dobot.ClearError()  # 清除Dobot的错误状态
                # 创建带反馈功能的连接用于获取实时位姿
                self.__dobot_feed = DobotApiDashMove(DOBOT_IP, DOBOT_FEEDPORT)
                self.dashboard = self.__dobot
                self.is_connected = True
                print(f"机械臂连接成功: {DOBOT_IP}:{DOBOT_PORT}")
            except Exception as e:
                print(f"机械臂连接失败: {e}")
                print("自动切换到模拟模式")
                self.simulation_mode = True
                self.is_connected = False
                # 确保dobot对象被正确清理
                if self.__dobot:
                    try:
                        self.__dobot.close()
                    except:
                        pass
                    self.__dobot = None
                self.__dobot_feed = None
                self.dashboard = None
        else:
            print("启动模拟模式 - 机械臂功能将被模拟")
            self.__dobot_feed = None
            # 在模拟模式下创建一个模拟的dashboard对象
            self.dashboard = self
        
        # self.__mb = ModbusSerialClient(port=mb_serial, baudrate=115200)
        # self.__mb.write_coil(1401, True, slave=SLAVE_ID_GRIPPER)  # 初始化Modbus线圈
        #self.home1()  # 执行默认动作
        # self._move_gripper(0, GRIPPER_AXIS_LIMIT)

    def connect(self):
        try:
            self.socket_dobot = socket.socket()
            self.socket_dobot.connect((DOBOT_IP, DOBOT_PORT))
            self.dashboard = DobotApiDashboard(DOBOT_IP,DOBOT_PORT)
            self.__dobot_feed = DobotApiDashMove(DOBOT_IP, DOBOT_FEEDPORT)
            self.is_connected = True
            print("机械臂已连接")
        except Exception as e:
            print(f"机械臂连接失败: {e}")
            self.is_connected = False
    
    def check_robot_status(self):
        """检查机械臂状态"""
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                # 获取机器人状态
                status = self.__dobot.RobotMode()
                print(f"机器人状态: {status}")
                return status
            except Exception as e:
                print(f"获取机器人状态失败: {e}")
                return None
        else:
            print("模拟模式 - 机器人状态正常")
            return "SIMULATION_MODE"
    
    def clear_error_and_enable(self):
        """清除错误并使能机器人"""
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                # 清除错误
                clear_result = self.__dobot.ClearError()
                print(f"清除错误结果: {clear_result}")
                sleep(0.5)
                
                # 使能机器人
                enable_result = self.__dobot.EnableRobot()
                print(f"使能机器人结果: {enable_result}")
                sleep(1.0)
                
                # 检查状态
                status = self.check_robot_status()
                return status
            except Exception as e:
                print(f"清除错误和使能失败: {e}")
                return None
        else:
            print("模拟模式 - 清除错误和使能成功")
            return "SIMULATION_MODE"

    def get_angle(self):
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                angle_str = self.__dobot.GetAngle()
                match = re.search(r"\{(.*?)\}", angle_str)
                if match:
                    angle_values = match.group(1).split(',')
                    # 过滤空字符串并转换为浮点数
                    return [float(x.strip()) for x in angle_values if x.strip()]
                else:
                    print(f"无法解析角度数据: {angle_str}")
                    return self._simulated_joints.copy()
            except Exception as e:
                print(f"获取角度时发生错误: {e}")
                return self._simulated_joints.copy()
        else:
            # 模拟模式：返回模拟的关节角度
            print(f"模拟关节角度: {self._simulated_joints}")
            return self._simulated_joints.copy()

    def get_point(self):
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                pose_str = self.__dobot.GetPose()
                match = re.search(r"\{(.*?)\}", pose_str)
                if match:
                    pose_values = match.group(1).split(',')
                    # 过滤空字符串并转换为浮点数
                    temp = [float(x.strip()) for x in pose_values if x.strip()]
                    if len(temp) >= 6:
                        temp[3] = (temp[3] + 720) % 360
                        temp[4] = (temp[4] + 720) % 360
                        temp[5] = (temp[5] + 720) % 360
                        print(temp)
                        return temp
                    else:
                        print(f"位置数据不完整: {temp}")
                        return self._simulated_pose.copy()
                else:
                    print(f"无法解析位置数据: {pose_str}")
                    return self._simulated_pose.copy()
            except Exception as e:
                print(f"获取位置时发生错误: {e}")
                return self._simulated_pose.copy()
        else:
            # 模拟模式：返回模拟的位置
            temp = self._simulated_pose.copy()
            print(f"模拟位置: {temp}")
            return temp

    def move_j(self, joints, limits):
        # 关节运动 - 接收关节角度参数
        assert len(limits) == len(joints)
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                # 检查并清除错误状态
                self.__dobot.ClearError()
                # 确保机器人使能
                self.__dobot.EnableRobot()
                sleep(0.1)  # 等待使能完成
                
                # 执行关节运动 - joints参数应该是关节角度
                result = self.__dobot.MovJ(*joints, 1, a=ACC, v=VEL * 3)
                print(f"MovJ执行结果: {result}")
                
                # 等待运动完成
                self._wait_for_result(self.get_angle, joints, limits)
                return True
            except Exception as e:
                print(f"关节运动执行错误: {e}")
                return False
        else:
            # 模拟模式：直接更新模拟关节角度
            print(f"模拟关节运动: {self._simulated_joints} -> {joints}")
            self._simulated_joints = list(joints)
            sleep(0.5)  # 模拟运动时间
            return True
    
    def move_j_cartesian(self, point, limits):
        # 笛卡尔坐标运动（使用MovJ但输入笛卡尔坐标）- 这是一个兼容性方法
        # 注意：这实际上应该使用move_l，但为了向后兼容保留此方法
        print("警告：move_j_cartesian已弃用，请使用move_l进行笛卡尔坐标运动")
        return self.move_l(point, limits)
    
    def move_l(self, point, limits):
        # 直线运动
        assert len(limits) == len(point)
        point[3] = (point[3] + 720) % 360
        point[4] = (point[4] + 720) % 360
        point[5] = (point[5] + 720) % 360
        if self.is_connected and self.__dobot and not self.simulation_mode:
            try:
                # 检查并清除错误状态
                self.__dobot.ClearError()
                # 确保机器人使能
                self.__dobot.EnableRobot()
                sleep(0.1)  # 等待使能完成
                
                # 执行直线运动
                result = self.__dobot.MovL(*point, 0, a=ACC, v=VEL)
                print(f"MovL执行结果: {result}")
                
                # 等待运动完成
                self._wait_for_result(self.get_point, point, limits)
                return True
            except Exception as e:
                print(f"直线运动执行错误: {e}")
                return False
        else:
            # 模拟模式：直接更新模拟位置
            print(f"模拟直线运动: {self._simulated_pose} -> {point}")
            self._simulated_pose = list(point)
            sleep(0.5)  # 模拟运动时间
            return True

    def EnableRobot(self, enable: bool = True):
        # 使能/禁用机器人
        if self.is_connected and self.__dobot and not self.simulation_mode:
            self.__dobot.EnableRobot()
        else:
            # 模拟模式：直接更新模拟状态
            print(f"模拟使能/禁用机器人: {enable}")
            self._simulated_enabled = enable
    
    def StartDrag(self):
        # 开始拖拽模式
        if self.is_connected and self.__dobot and not self.simulation_mode:
            self.__dobot.StartDrag()
        else:
            # 模拟模式：模拟拖拽模式
            print("模拟开始拖拽模式")
            self._simulated_drag_mode = True

    def CP(self, ratio):
        # 设置平滑过渡比例 - 支持两种不同的API
        if self.is_connected and self.__dobot and not self.simulation_mode:
            # 使用Dobot API设置平滑过渡比例
            self.__dobot.CP(ratio)
        else:
            # 模拟模式：记录平滑过渡比例设置
            print(f"模拟设置平滑过渡比例: {ratio}")
            self._simulated_cp_ratio = ratio

    def JointMovJ(self, j1, j2, j3, j4, j5, j6, *dynParams):
        # 关节运动 - 使用新版API
        joints = [j1, j2, j3, j4, j5, j6]
        if self.is_connected and self.__dobot and not self.simulation_mode:
            # 使用新版Dobot API进行关节运动
            self.__dobot.RelJointMovJ(j1, j2, j3, j4, j5, j6, *dynParams)
            # 等待运动完成
            self._wait_for_result(self.get_angle, joints, JOINTS_LIMIT)
        else:
            # 模拟模式：直接更新模拟关节角度
            print(f"模拟关节运动: {self._simulated_joints} -> {joints}")
            self._simulated_joints = list(joints)
            sleep(0.5)  # 模拟运动时间
        print(self.get_angle())
        sleep(0.5)

    def _wait_for_result(self, get_current_func, target, limits):
        # 等待运动完成
        while True:
            done = True
            for index, (t, c, l) in enumerate(zip(target, get_current_func(), limits)):
                if abs(t - c) > l:
                    # print(t, c, l)
                    done = False
                    break
            if done:
                break
            sleep(0.001)

    def _us_get_dis(self):
        # 获取超声波传感器的距离
        return self.__mb.read_holding_registers(513, 1, slave=SLAVE_ID_US).registers[0]

    def _move_gripper(self, position: float, limit: float):
        # 控制夹爪运动
        if self.is_connected and self.__mb and not self.simulation_mode:
            self.__mb.write_registers(4900, [0, 0, 0, int(position * 1000)], slave=SLAVE_ID_GRIPPER)
            while abs(self._gripper_pose() - position) > limit:
                sleep(0.001)
        else:
            # 模拟模式：模拟夹爪运动
            print(f"模拟夹爪运动到位置: {position}")
            self._simulated_gripper_pos = position
            sleep(0.2)  # 模拟运动时间

    def _gripper_pose(self):
        # 获取夹爪当前位置
        if self.is_connected and self.__mb and not self.simulation_mode:
            curr = self.__mb.read_input_registers(0, 2, slave=SLAVE_ID_GRIPPER).registers[1]
            return curr / 1000 if curr < 32767 else (65535 - curr) / 1000
        else:
            # 模拟模式：返回模拟的夹爪位置
            return getattr(self, '_simulated_gripper_pos', 0.0)

    def home1(self):
        # 默认动作：初始化位置
        self.move_j((-90, -45, 135, 0, -90, 0), JOINTS_LIMIT)

    def home2(self):
        # 默认动作：初始化位置
        self.move_j((0, -45, 135, 0, -90, 0), JOINTS_LIMIT) #机械臂初始位置，正面门口
    
    @property
    def current_actual(self):
        """
        获取机械臂当前实际位姿
        返回格式: [x, y, z, rx, ry, rz] (TCP位置和姿态)
        """
        if self.is_connected and self.__dobot_feed and not self.simulation_mode:
            try:
                # 获取实时反馈数据
                feed_data = self.__dobot_feed.getFeedData()
                if feed_data and hasattr(feed_data, 'toolVectorActual') and len(feed_data.toolVectorActual) >= 6:
                    # toolVectorActual包含[x, y, z, rx, ry, rz]
                    current_pose = list(feed_data.toolVectorActual)
                    print(f"当前机械臂位姿: {current_pose}")
                    return current_pose
                else:
                    print("无法获取有效的位姿数据")
                    return None
            except Exception as e:
                print(f"获取机械臂位姿时发生错误: {e}")
                return None
        else:
            # 模拟模式：返回模拟的位姿
            print(f"模拟当前位姿: {self._simulated_pose}")
            return self._simulated_pose.copy()


    def open_gripper(self):
        # 打开夹爪
        self._move_gripper(0, GRIPPER_AXIS_LIMIT)

    def close_gripper(self):
        # 关闭夹爪
        self._move_gripper(21, GRIPPER_AXIS_LIMIT)
    
    def gripper_move(self):
        # 关闭夹爪
        self._move_gripper(30, GRIPPER_AXIS_LIMIT)

