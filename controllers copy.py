from dobot.arm import arm
# from pylibrm.RMAxis import Axis_V6
import motormaster
from pyModbusTCP.client import ModbusClient
from pymodbus.client.sync import ModbusSerialClient
from pymodbus.constants import Endian
from pymodbus.payload import BinaryPayloadDecoder
import time
import json
import struct
import threading
import math
import numpy as np
from typing import List, Tuple, Dict, Any, Optional
from rotavapor_controller import RotavaporController
import requests
import ctypes
import inspect
from camera.camera import get_camera
from realsense_calibration import load_hand_eye_calibration
import os
from datetime import datetime
import cv2
import socket
from queue import Queue, Empty

class BenchContorller:

    task_new = False
    order_info = {"key_name":"", "task_id":"","params":"", "status":"", "detail":""}
    order_status = ""
    bench_status = "idle"
    t_status = threading.Thread
    t_task = threading.Thread

    rotavapor_controller = RotavaporController()

    def __init__(self):

        self.motor_contorller = ModbusClient(host="127.0.0.1", port=502, unit_id=1, auto_open=True)
        # self.motor_contorller = ModbusClient("/dev/ttyUSB1", port=502, unit_id=1, auto_open=True)
        # self.slide_controller = ModbusSerialClient(method="rtu", port="/dev/slide_com", baudrate=115200, timeout=1)
        # self.slide_controller = ModbusSerialClient(method="rtu", port="\\\\.\\COM1", baudrate=115200, timeout=1)
        # # self.slide_controller = ModbusSerialClient(method="rtu", port="/dev/ttyUSB1", baudrate=115200, timeout=1)
        # self.slide_controller.connect()
        # print("client succeed")
        # self.grip_init()
        # self.oscillator_pusher_init()
        # self.grip_rotate_init()
        # self.grip_recv_position()
        self.grip_axis = motormaster.create_axis_modbus_rtu('\\\\.\\COM5', 115200, 7)
        # self.grip_axis = Axis_V6.create_modbus_tcp('192.168.0.233', 502, 1)
        # self.ex = Exception("time out") 
        # # self.lid_operater_init()
        # with open('bench.json', 'r', encoding='utf-8') as file:
        #     self.bench_json = json.load(file)
        # self.all_zero()
        # time.sleep(3.0)
        # self.url = 'http://192.168.13.31:3001/api/location/minus'

        # 首先尝试连接实体机械臂
        self.arm_controller = arm(simulation_mode=False)
        
        if self.arm_controller.is_connected:
            try:
                self.arm_controller.EnableRobot(True)
                print("机械臂连接成功并已使能")
            except Exception as e:
                print(f"机械臂使能失败: {e}")
        else:
            print("机械臂连接失败，系统将在模拟模式下运行")
        
        # 加载手眼标定结果
        self.hand_eye_matrix = None
        self.camera_matrix = None
        self.dist_coeffs = None
        self.load_calibration_data()
        
        # TCP客户端配置
        self.tcp_server_host = "127.0.0.1"
        self.tcp_server_port = 7930
        self.coordinate_queue = Queue()  # 用于存储接收到的坐标数据
        self.tcp_client_running = False
        self.tcp_client_thread = None
        self.tcp_client_socket = None
        self.parse_error_count = 0  # 解析错误计数器
        self.max_parse_errors = 5   # 最大连续解析错误次数
        
        # 启动TCP客户端
        self.start_tcp_client()
    
    def load_calibration_data(self):
        """加载手眼标定数据"""
        try:
            self.hand_eye_matrix, self.camera_matrix, self.dist_coeffs = load_hand_eye_calibration()
            if self.hand_eye_matrix is not None:
                print("手眼标定数据加载成功")
            else:
                print("警告：未找到手眼标定数据，将使用默认坐标转换")
        except Exception as e:
            print(f"加载手眼标定数据失败: {e}")
            self.hand_eye_matrix = None
            self.camera_matrix = None
            self.dist_coeffs = None

    def check_arm_connection(self):
        """检查机械臂连接状态"""
        return self.arm_controller.is_connected
    
    def start_tcp_client(self):
        """启动TCP客户端连接到外部服务器"""
        if not self.tcp_client_running:
            self.tcp_client_running = True
            self.tcp_client_thread = threading.Thread(target=self._tcp_client_worker, daemon=True)
            self.tcp_client_thread.start()
            print(f"TCP客户端已启动，连接到 {self.tcp_server_host}:{self.tcp_server_port}")
    
    def stop_tcp_client(self):
        """停止TCP客户端"""
        self.tcp_client_running = False
        if self.tcp_client_socket:
            try:
                self.tcp_client_socket.close()
            except:
                pass
            self.tcp_client_socket = None
        if self.tcp_client_thread and self.tcp_client_thread.is_alive():
            self.tcp_client_thread.join(timeout=2.0)
        print("TCP客户端已停止")
    
    def _tcp_client_worker(self):
        """TCP客户端工作线程"""
        while self.tcp_client_running:
            try:
                # 创建socket连接
                self.tcp_client_socket = socket.socket(socket.AF_INET, socket.SOCK_STREAM)
                self.tcp_client_socket.settimeout(5.0)  # 连接超时
                
                print(f"正在连接到TCP服务器 {self.tcp_server_host}:{self.tcp_server_port}")
                self.tcp_client_socket.connect((self.tcp_server_host, self.tcp_server_port))
                print(f"成功连接到TCP服务器 {self.tcp_server_host}:{self.tcp_server_port}")
                
                # 重置解析错误计数器
                self.parse_error_count = 0
                
                # 连接成功后发送start命令
                try:
                    message = "start\n"
                    self.tcp_client_socket.send(message.encode('utf-8'))
                    print("连接成功后已发送start命令")
                except Exception as e:
                    print(f"发送初始start命令失败: {e}")
                
                # 设置接收超时
                self.tcp_client_socket.settimeout(1.0)
                
                # 持续接收数据
                print("开始监听TCP数据...")
                while self.tcp_client_running:
                    try:
                        data = self.tcp_client_socket.recv(1024).decode('utf-8').strip()
                        if not data:
                            print("服务器关闭连接")
                            break
                        
                        print(f"接收到原始数据: {data}")
                        
                        # 解析分号分隔的坐标数据格式: -18.598;-83.695;-4.513
                        try:
                            coordinates = data.split(';')
                            if len(coordinates) == 3:
                                x = float(coordinates[0])
                                y = float(coordinates[1])
                                z = float(coordinates[2])
                                
                                coordinate_data = {
                                    "x": x,
                                    "y": y,
                                    "z": z,
                                    "timestamp": time.time()
                                }
                                
                                # 将坐标数据放入队列
                                self.coordinate_queue.put(coordinate_data)
                                # 重置解析错误计数器
                                self.parse_error_count = 0
                                print(f"坐标数据已加入队列: X={x:.3f}, Y={y:.3f}, Z={z:.3f}, 队列大小: {self.coordinate_queue.qsize()}")
                                
                            else:
                                self.parse_error_count += 1
                                if self.parse_error_count <= self.max_parse_errors:
                                    print(f"坐标数据格式错误，期望3个值，收到{len(coordinates)}个值: {data}")
                                elif self.parse_error_count == self.max_parse_errors + 1:
                                    print(f"连续解析错误超过{self.max_parse_errors}次，后续错误将不再显示")
                                
                        except ValueError as e:
                            self.parse_error_count += 1
                            if self.parse_error_count <= self.max_parse_errors:
                                print(f"解析坐标数据失败: {e}, 数据: {data}")
                            elif self.parse_error_count == self.max_parse_errors + 1:
                                print(f"连续解析错误超过{self.max_parse_errors}次，后续错误将不再显示")
                            
                    except socket.timeout:
                        continue  # 接收超时，继续循环
                    except Exception as e:
                        print(f"接收数据时发生错误: {e}")
                        break
                        
            except Exception as e:
                print(f"TCP客户端连接失败: {e}")
                
            # 关闭当前连接
            if self.tcp_client_socket:
                try:
                    self.tcp_client_socket.close()
                except:
                    pass
                self.tcp_client_socket = None
            
            # 如果还在运行状态，等待一段时间后重新连接
            if self.tcp_client_running:
                print("等待5秒后重新连接...")
                time.sleep(5)
                
        print("TCP客户端工作线程已退出")
    

    
    def send_start_command(self, command="start", clear_queue=True):
        """
        向TCP服务器发送指定的命令字符串
        
        Args:
            command: 要发送的命令，可以是"start", "start1", "start2"等，默认为"start"
            clear_queue: 是否清空队列中的旧数据，默认为True
        
        Returns:
            dict: 包含执行结果的字典
        """
        try:
            if self.tcp_client_socket and self.tcp_client_running:
                message = f"{command}\n"
                bytes_sent = self.tcp_client_socket.send(message.encode('utf-8'))
                print(f"已向TCP服务器发送{command}命令，发送字节数: {bytes_sent}, 内容: {repr(message)}")
                print(f"TCP连接状态 - Socket: {self.tcp_client_socket}, 运行状态: {self.tcp_client_running}")
                print(f"服务器地址: {self.tcp_server_host}:{self.tcp_server_port}")
                
                cleared_count = 0
                if clear_queue:
                    # 清空当前队列中的旧数据
                    while not self.coordinate_queue.empty():
                        try:
                            self.coordinate_queue.get_nowait()
                            cleared_count += 1
                        except Empty:
                            break
                    print(f"已清空坐标队列中的{cleared_count}条旧数据")
                else:
                    print("保留队列中的现有数据")
                
                return {
                    "success": True,
                    "message": f"{command}命令发送成功",
                    "command_sent": command,
                    "cleared_data_count": cleared_count
                }
            else:
                return {
                    "success": False,
                    "message": "TCP客户端未连接，无法发送命令",
                    "error_code": "TCP_NOT_CONNECTED"
                }
        except Exception as e:
            print(f"发送{command}命令失败: {e}")
            return {
                "success": False,
                "message": f"发送{command}命令失败: {str(e)}",
                "error_code": "SEND_COMMAND_ERROR"
            }
    
    def get_tcp_status(self):
        """
        获取TCP客户端连接状态
        
        Returns:
            dict: TCP连接状态信息
        """
        try:
            queue_size = self.coordinate_queue.qsize()
            
            # 获取队列中最新的几条数据（不移除）
            recent_data = []
            temp_queue = Queue()
            try:
                # 临时取出所有数据
                all_data = []
                while not self.coordinate_queue.empty():
                    try:
                        data = self.coordinate_queue.get_nowait()
                        all_data.append(data)
                    except Empty:
                        break
                
                # 保留最新的3条数据用于显示
                recent_data = all_data[-3:] if len(all_data) >= 3 else all_data
                
                # 将所有数据放回队列
                for data in all_data:
                    self.coordinate_queue.put(data)
                    
            except Exception:
                pass
            
            return {
                "success": True,
                "tcp_running": self.tcp_client_running,
                "socket_connected": self.tcp_client_socket is not None,
                "server_host": self.tcp_server_host,
                "server_port": self.tcp_server_port,
                "queue_size": queue_size,
                "parse_error_count": self.parse_error_count,
                "thread_alive": self.tcp_client_thread.is_alive() if self.tcp_client_thread else False,
                "recent_coordinates": recent_data,
                "last_update": time.time()
            }
        except Exception as e:
            return {
                "success": False,
                "message": f"获取TCP状态失败: {str(e)}",
                "error_code": "GET_STATUS_ERROR"
            }
    
    def get_latest_coordinates(self, timeout=1.0):
        """
        从队列中获取最新的坐标数据
        
        Args:
            timeout: 等待超时时间（秒）
            
        Returns:
            坐标数据字典或None
        """
        try:
            print(f"正在等待坐标数据，队列当前大小: {self.coordinate_queue.qsize()}")
            coordinate_data = self.coordinate_queue.get(timeout=timeout)
            print(f"成功获取到坐标数据: X={coordinate_data['x']:.3f}, Y={coordinate_data['y']:.3f}, Z={coordinate_data['z']:.3f}")
            return coordinate_data
        except Empty:
            print(f"在{timeout}秒内未从队列中获取到坐标数据")
            print(f"当前TCP连接状态: 运行={self.tcp_client_running}, Socket连接={self.tcp_client_socket is not None}")
            return None
    
    def execute_pick_sequence_from_queue(self, approach_height: float = 700.0, pick_height: float = 446.0, timeout: float = 10.0, recognition_scheme: str = "start1") -> Dict[str, Any]:
        """
        从队列中获取坐标数据并执行抓取序列（整合版本）
        
        Args:
            approach_height: 接近高度（毫米），在目标上方的安全高度
            pick_height: 抓取高度（毫米），实际抓取时的Z坐标
            timeout: 等待坐标数据的超时时间（秒）
            recognition_scheme: 识别方案，可以是"start1", "start2"等，默认为"start1"
        
        Returns:
            执行结果字典
        """
        try:
            # 检查机械臂连接状态
            if not self.check_arm_connection():
                return {
                    "success": False,
                    "message": "机械臂未连接",
                    "error_code": "ARM_NOT_CONNECTED"
                }
            
            # 根据识别方案设置不同的参数
            if recognition_scheme == "start1":
                gripper_open_pos = 33
                gripper_close_pos = 35
                arm_rotation = 270
                origin_point = [-0.0268, -363.0517, 772.0800, 180, 0, 270]
                default_pick_height = 446.0
                target_z_default = 500
            elif recognition_scheme == "start2":
                gripper_open_pos = 16.8
                gripper_close_pos = 25.5
                arm_rotation = 90
                origin_point = [93.0233, -370.0210, 772.0800, 180, 0, 90]
                default_pick_height = 525.0
                target_z_default = 600
            else:
                return {
                    "success": False,
                    "message": f"不支持的识别方案: {recognition_scheme}",
                    "error_code": "UNSUPPORTED_SCHEME"
                }
            
            # 如果使用默认pick_height，则根据识别方案调整
            if pick_height == 0:  # 默认值
                pick_height = default_pick_height
            
            # 步骤0: 移动到初始点位（原点坐标）
            print(f"步骤0: 移动到初始点位 ({origin_point[0]:.3f}, {origin_point[1]:.3f}, {origin_point[2]:.3f})")
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(origin_point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "移动到初始点位失败",
                    "error_code": "INITIAL_MOVE_FAILED"
                }
            time.sleep(0.5)
            
            # 抓取开始时发送指定的识别方案命令
            print(f"抓取开始，发送识别方案: {recognition_scheme}")
            start_result = self.send_start_command(command=recognition_scheme, clear_queue=True)
            if not start_result["success"]:
                print(f"发送{recognition_scheme}命令失败: {start_result['message']}")
                return {
                    "success": False,
                    "message": f"发送{recognition_scheme}命令失败",
                    "error_code": "START_COMMAND_FAILED"
                }
            else:
                print(f"{recognition_scheme}命令发送成功，等待服务器响应...")
                # 给服务器一些时间来响应命令并发送数据
                time.sleep(0.5)
            
            # 从队列中获取坐标数据
            print(f"等待坐标数据，超时时间: {timeout}秒")
            
            coordinate_data = self.get_latest_coordinates(timeout=timeout)
            
            if coordinate_data is None:
                return {
                    "success": False,
                    "message": f"在{timeout}秒内未接收到坐标数据",
                    "error_code": "NO_COORDINATE_DATA"
                }
            
            # 提取坐标数据
            target_x = coordinate_data["x"]
            target_y = coordinate_data["y"]
            target_z = coordinate_data.get("z", target_z_default)  # 如果z为None，使用默认值
            
            print(f"开始执行抓取序列，接收到的坐标: ({target_x:.3f}, {target_y:.3f})，使用识别方案: {recognition_scheme}")
            
            # 步骤1: 打开夹爪
            print("步骤1: 打开夹爪")
            gripper_result = self.gripper_control_move(position=gripper_open_pos)
            if not gripper_result["success"]:
                return gripper_result
            time.sleep(1.0)
            
            # 步骤2: 移动到目标上方的安全位置
            print(f"步骤2: 移动到安全接近位置 ({target_x:.3f}, {target_y:.3f}, {approach_height:.2f})")
            point = [target_x, target_y, approach_height, 180, 0, arm_rotation]  # 使用接收到的X,Y坐标
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "移动到接近位置失败",
                    "error_code": "APPROACH_MOVE_FAILED"
                }
            time.sleep(1.0)
            
            # 步骤3: 下降到抓取位置（可以使用接收到的Z坐标或指定的pick_height）
            actual_pick_height = min(pick_height, target_z) if target_z is not None and target_z > 0 else pick_height
            print(f"步骤3: 下降到抓取位置 ({target_x:.3f}, {target_y:.3f}, {actual_pick_height:.2f})")
            point = [target_x, target_y, actual_pick_height, 180, 0, arm_rotation]
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "下降到抓取位置失败",
                    "error_code": "PICK_MOVE_FAILED"
                }
            time.sleep(0.5)
            
            # 步骤4: 关闭夹爪进行抓取
            print("步骤4: 关闭夹爪进行抓取")
            gripper_result = self.gripper_control_move(position=gripper_close_pos)
            if not gripper_result["success"]:
                return gripper_result
            time.sleep(1.0)
            
            # 步骤5: 提升到安全高度
            print(f"步骤5: 提升到安全高度 ({target_x:.3f}, {target_y:.3f}, {approach_height:.2f})")
            point = [target_x, target_y, approach_height, 180, 0, arm_rotation]
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "提升到安全高度失败",
                    "error_code": "LIFT_MOVE_FAILED"
                }
            time.sleep(0.5)

            # 步骤6: 回到原点
            print(f"步骤6: 回到原点")
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(origin_point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "回到原点失败",
                    "error_code": "RETURN_ORIGIN_FAILED"
                }
            time.sleep(0.5)
            
            print("抓取序列执行完成")
            
            # 抓取流程完成后清空队列数据
            cleared_count = 0
            while not self.coordinate_queue.empty():
                try:
                    self.coordinate_queue.get_nowait()
                    cleared_count += 1
                except Empty:
                    break
            print(f"抓取完成后已清空队列中的{cleared_count}条数据")
            
            return {
                "success": True,
                "message": "抓取序列执行成功",
                "recognition_scheme": recognition_scheme,
                "received_coordinates": {
                    "x": target_x,
                    "y": target_y,
                    "z": target_z
                },
                "executed_positions": {
                    "approach_height": approach_height,
                    "pick_height": actual_pick_height
                }
            }
            
        except Exception as e:
            print(f"执行抓取序列时发生错误: {e}")
            return {
                "success": False,
                "message": f"执行过程中发生异常: {str(e)}",
                "error_code": "EXECUTION_ERROR"
            }
    

    def put_back(self, approach_height: float = 700.0, pick_height: float = 446.0, timeout: float = 10.0, recognition_scheme: str = "start1") -> Dict[str, Any]:
        """
        从队列中获取坐标数据并执行抓取序列
        
        Args:
            approach_height: 接近高度（毫米），在目标上方的安全高度
            pick_height: 抓取高度（毫米），实际抓取时的Z坐标
            timeout: 等待坐标数据的超时时间（秒）
            recognition_scheme: 识别方案，可以是"start", "start1", "start2"等，默认为"start1"
        
        Returns:
            执行结果字典
        """
        try:
            # 检查机械臂连接状态
            if not self.check_arm_connection():
                return {
                    "success": False,
                    "message": "机械臂未连接",
                    "error_code": "ARM_NOT_CONNECTED"
                }
            
            # 抓取开始时发送指定的识别方案命令
            print(f"抓取开始，发送识别方案: {recognition_scheme}")
            start_result = self.send_start_command(command=recognition_scheme, clear_queue=True)
            if not start_result["success"]:
                print(f"发送{recognition_scheme}命令失败: {start_result['message']}")
                return {
                    "success": False,
                    "message": f"发送{recognition_scheme}命令失败",
                    "error_code": "START_COMMAND_FAILED"
                }
            else:
                print(f"{recognition_scheme}命令发送成功，等待服务器响应...")
                # 给服务器一些时间来响应命令并发送数据
                time.sleep(0.5)
            
            # 从队列中获取坐标数据
            print(f"等待坐标数据，超时时间: {timeout}秒")
            
            coordinate_data = self.get_latest_coordinates(timeout=timeout)
            
            if coordinate_data is None:
                return {
                    "success": False,
                    "message": f"在{timeout}秒内未接收到坐标数据",
                    "error_code": "NO_COORDINATE_DATA"
                }
            
            # 提取坐标数据
            target_x = coordinate_data["x"]
            target_y = coordinate_data["y"]
            target_z = 500
            
            print(f"开始执行放回序列，接收到的坐标: ({target_x:.3f}, {target_y:.3f})，使用识别方案: {recognition_scheme}")
                       
            # 步骤1: 移动到目标上方的安全位置
            print(f"步骤1: 移动到安全接近位置 ({target_x:.3f}, {target_y:.3f}, {approach_height:.2f})")
            point = [target_x, target_y, approach_height, 180, 0, 270]  # 使用接收到的X,Y坐标
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "移动到接近位置失败",
                    "error_code": "APPROACH_MOVE_FAILED"
                }
            time.sleep(1.0)
            
            # 步骤2: 下降到抓取位置（可以使用接收到的Z坐标或指定的pick_height）
            actual_pick_height = min(pick_height, target_z) if target_z > 0 else pick_height
            print(f"步骤2: 下降到抓取位置 ({target_x:.3f}, {target_y:.3f}, {actual_pick_height:.2f})")
            point = [target_x, target_y, actual_pick_height, 180, 0, 270]
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "下降到抓取位置失败",
                    "error_code": "PICK_MOVE_FAILED"
                }
            time.sleep(0.5)
            
            # 步骤3: 打开夹爪进行放置
            print("步骤3: 打开夹爪进行放置")
            gripper_result = self.gripper_control_move(position=33)
            if not gripper_result["success"]:
                return gripper_result
            time.sleep(1.0)
            
            # 步骤4: 提升到安全高度
            print(f"步骤4: 提升到安全高度 ({target_x:.3f}, {target_y:.3f}, {approach_height:.2f})")
            point = [target_x, target_y, approach_height, 180, 0, 270]
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "提升到安全高度失败",
                    "error_code": "LIFT_MOVE_FAILED"
                }
            time.sleep(0.5)

            # 步骤5: 回到原点
            print(f"步骤5: 回到原点")
            point = [-0.0268, -363.0517, 772.0800, 180, 0, 270]
            # point = [-20.0003, -390.0005, 763.1149, 180, 0, 270]
            limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]
            move_result = self.arm_controller.move_l(point, limits)
            if not move_result:
                return {
                    "success": False,
                    "message": "回到原点失败失败",
                    "error_code": "LIFT_MOVE_FAILED"
                }
            time.sleep(0.5)
            
            print("抓取序列执行完成")
            
            # 抓取流程完成后清空队列数据
            cleared_count = 0
            while not self.coordinate_queue.empty():
                try:
                    self.coordinate_queue.get_nowait()
                    cleared_count += 1
                except Empty:
                    break
            print(f"放回完成后已清空队列中的{cleared_count}条数据")
            
            return {
                "success": True,
                "message": "放回序列执行成功",
                "recognition_scheme": recognition_scheme,
                "received_coordinates": {
                    "x": target_x,
                    "y": target_y,
                    "z": target_z
                },
                "executed_positions": {
                    "approach_height": approach_height,
                    "pick_height": actual_pick_height
                }
            }
            
        except Exception as e:
            print(f"执行抓取序列时发生错误: {e}")
            return {
                "success": False,
                "message": f"执行过程中发生异常: {str(e)}",
                "error_code": "EXECUTION_ERROR"
            }
    
    def control_gripper(self, action: str):
        """
        控制夹爪开合
        
        Args:
            action: 动作类型，"open" 或 "close"
        
        Returns:
            dict: 包含执行结果的字典
        """
        try:
            if action not in ["open", "close"]:
                return {
                    "success": False,
                    "message": "无效的夹爪动作，请使用 'open' 或 'close'",
                    "error_code": "INVALID_ACTION"
                }
            
            print(f"执行夹爪动作: {action}")
            
            if action == "open":
                self.gripper_open()
                return {
                    "success": True,
                    "message": "夹爪打开成功",
                    "action": "open"
                }
            else:  # action == "close"
                self.gripper_close()
                return {
                    "success": True,
                    "message": "夹爪关闭成功",
                    "action": "close"
                }
                
        except Exception as e:
            print(f"控制夹爪时发生错误: {e}")
            return {
                "success": False,
                "message": f"夹爪控制异常: {str(e)}",
                "error_code": "GRIPPER_CONTROL_ERROR"
            }
    
    def capture_and_save_image(self, save_directory: str = "images") -> Dict[str, Any]:
        """
        单独控制相机拍照并按日期保存图片到指定目录
        
        Args:
            save_directory: 保存图片的目录，默认为"images"
        
        Returns:
            dict: 包含执行结果的字典
        """
        try:
            # 获取相机实例
            camera = get_camera()
            
            if not camera.is_connected:
                return {
                    "success": False,
                    "message": "相机未连接",
                    "error_code": "CAMERA_NOT_CONNECTED"
                }
            
            # 确保保存目录存在
            if not os.path.exists(save_directory):
                os.makedirs(save_directory)
            
            # 生成带时间戳的文件名
            timestamp = datetime.now().strftime("%Y%m%d_%H%M%S")
            filename = f"capture_{timestamp}.png"
            filepath = os.path.join(save_directory, filename)
            
            print(f"开始拍照，保存路径: {filepath}")
            
            # 采集图像
            image = camera.capture_image()
            if image is None:
                return {
                    "success": False,
                    "message": "图像采集失败",
                    "error_code": "IMAGE_CAPTURE_FAILED"
                }
            
            # 保存图像
            success = cv2.imwrite(filepath, image)
            if not success:
                return {
                    "success": False,
                    "message": "图像保存失败",
                    "error_code": "IMAGE_SAVE_FAILED"
                }
            
            print(f"图像保存成功: {filepath}")
            
            return {
                "success": True,
                "message": "图像拍照和保存成功",
                "filepath": filepath,
                "filename": filename,
                "timestamp": timestamp,
                "image_size": {
                    "width": image.shape[1],
                    "height": image.shape[0],
                    "channels": image.shape[2] if len(image.shape) > 2 else 1
                }
            }
            
        except Exception as e:
            print(f"拍照保存过程中发生错误: {e}")
            return {
                "success": False,
                "message": f"拍照保存异常: {str(e)}",
                "error_code": "CAPTURE_SAVE_ERROR"
            }

    def image_to_world_1(u, v):
        """将坩埚图像坐标转换为世界坐标"""
        # 标定矩阵参数
        h11, h12, h13 = 0.10313439, 0.0017332305, -111.84082
        h21, h22, h23 = 0.002168329, -0.1026563, 513.4729
        
        # 计算特征点世界坐标
        feature_x = h11 * u + h12 * v + h13
        feature_y = h21 * u + h22 * v + h23
        
        # 应用校准后的偏移量
        obj_x = feature_x + 50.9267 + 15
        obj_y = feature_y - 606.9468 + 15
        
        return obj_x, obj_y
        
    def control_arm_movement(self, x: float, y: float, z: float, rx: float = 0, ry: float = 0, rz: float = 0, movement_type: str = "joint") -> Dict[str, Any]:
        """
        单独控制机械臂x,y,z移动
        
        Args:
            x: X轴坐标（毫米）
            y: Y轴坐标（毫米）
            z: Z轴坐标（毫米）
            rx: X轴旋转角度（度），默认0
            ry: Y轴旋转角度（度），默认0
            rz: Z轴旋转角度（度），默认0
            movement_type: 运动类型，"joint"（关节运动）或"linear"（直线运动），默认"joint"
        
        Returns:
            dict: 包含执行结果的字典
        """
        try:
            # 确保所有参数都是数值类型，防止字符串类型导致的错误
            try:
                x = float(x)
                y = float(y)
                z = float(z)
                rx = float(rx)
                ry = float(ry)
                rz = float(rz)
            except (ValueError, TypeError) as e:
                return {
                    "success": False,
                    "message": f"参数类型错误，所有坐标参数必须是数值类型: {str(e)}",
                    "error_code": "INVALID_PARAMETER_TYPE"
                }
            
            # 检查机械臂连接状态
            if not self.check_arm_connection():
                # 尝试重新连接
                print("机械臂未连接，尝试重新连接...")
                try:
                    self.arm_controller.connect()
                    if not self.check_arm_connection():
                        return {
                            "success": False,
                            "message": "机械臂连接失败，请检查机械臂电源和网络连接",
                            "error_code": "ARM_NOT_CONNECTED"
                        }
                except Exception as conn_e:
                    return {
                        "success": False,
                        "message": f"机械臂重连失败: {str(conn_e)}",
                        "error_code": "ARM_RECONNECT_FAILED"
                    }
            
            # 检查机械臂状态并清除错误
            print("检查机械臂状态...")
            robot_status = self.arm_controller.check_robot_status()
            if robot_status is None or "Connection refused" in str(robot_status):
                print("机械臂状态异常，尝试清除错误并重新使能...")
                clear_status = self.arm_controller.clear_error_and_enable()
                if clear_status is None:
                    return {
                        "success": False,
                        "message": "机械臂状态异常且无法恢复，请检查机械臂连接和状态",
                        "error_code": "ARM_ERROR_STATE"
                    }
            
            # 验证运动类型
            if movement_type not in ["joint", "linear"]:
                return {
                    "success": False,
                    "message": "无效的运动类型，请使用 'joint' 或 'linear'",
                    "error_code": "INVALID_MOVEMENT_TYPE"
                }
            
            print(f"控制机械臂移动到位置: X={x}, Y={y}, Z={z}, RX={rx}, RY={ry}, RZ={rz}，运动类型: {movement_type}")
            
            # 执行运动
            if movement_type == "joint":
                # 对于关节运动，需要先将笛卡尔坐标转换为关节角度
                # # 这里暂时使用move_l进行笛卡尔坐标运动，因为输入的是xyz坐标而不是关节角度
                # print("警告：输入的是笛卡尔坐标，使用直线运动模式")
                point = [x, y, z, rx, ry, rz]  # 位置坐标
                limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 误差限制
                move_result = self.arm_controller.move_l(point, limits)
            else:  # linear
                # move_l需要位置列表和误差限制列表
                point = [x, y, z, rx, ry, rz]  # 位置坐标
                limits = [0.1, 0.1, 0.1, 0.1, 0.1, 0.1]  # 误差限制
                move_result = self.arm_controller.move_l(point, limits)
            
            if move_result:
                return {
                    "success": True,
                    "message": f"机械臂{movement_type}运动成功",
                    "target_position": {
                        "x": x,
                        "y": y,
                        "z": z,
                        "rx": rx,
                        "ry": ry,
                        "rz": rz
                    },
                    "movement_type": movement_type
                }
            else:
                return {
                    "success": False,
                    "message": "机械臂移动失败",
                    "error_code": "ARM_MOVE_FAILED"
                }
                
        except Exception as e:
            print(f"控制机械臂移动时发生错误: {e}")
            return {
                "success": False,
                "message": f"机械臂移动异常: {str(e)}",
                "error_code": "ARM_MOVEMENT_ERROR"
            }

    # kill thread
    def _async_raise(self, tid, exctype):
        """raises the exception, performs cleanup if needed"""
        tid = ctypes.c_long(tid)
        if not inspect.isclass(exctype):
            exctype = type(exctype)
        res = ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, ctypes.py_object(exctype))
        if res == 0:
            raise ValueError("invalid thread id")
        elif res != 1:
            # """if it returns a number greater than one, you're in trouble,
            # and you should call it again with exc=NULL to revert the effect"""
            ctypes.pythonapi.PyThreadState_SetAsyncExc(tid, None)
            raise SystemError("PyThreadState_SetAsyncExc failed")

    def oscillator_pusher_init(self):
        if self.motor_contorller.write_single_register(5, 1):
            print("write ok")
        else:
            print("write error")     
        time.sleep(0.5)  

    def oscillator_pusher_on(self):
        if self.motor_contorller.write_single_register(13, 1):
            print("write ok")
        else:
            print("write error")       
        time.sleep(1.0)  

    def magnetic_rotator_on(self):
        self.motor_contorller.write_single_register(1021, 360)
        self.motor_contorller.write_single_register(1022, 10)
        self.motor_contorller.write_single_register(24, 1)
        # time_keep = 0
        # while(time_keep <= 19):
        #     regs = self.motor_contorller.read_holding_registers(23, 1)
        #     time_keep = regs[0]
        #     if regs:
        #         print(regs)
        #     else:
        #         print("read error")
        #     time.sleep(0.2)
        # statue = 0
        # while(statue != 3):
        #     regs = self.motor_contorller.read_holding_registers(24, 1)
        #     statue = regs[0]
        #     if regs:
        #         print(regs)
        #     else:
        #         print("read error")
        #     time.sleep(0.2)

    def crucible_shaker_on(self):
        self.motor_contorller.write_single_register(1080, 10)
        self.motor_contorller.write_single_register(82, 1)
        statue = 0
        while(statue != 3):
            regs = self.motor_contorller.read_holding_registers(82, 1)
            statue = regs[0]
            if regs:
                print(regs)
            else:
                print("read error")
            time.sleep(0.2)

    def funnel_shaker_on(self):
        self.motor_contorller.write_single_register(1083, 5)
        self.motor_contorller.write_single_register(85, 1)
        statue = 0
        while(statue != 3):
            regs = self.motor_contorller.read_holding_registers(85, 1)
            statue = regs[0]
            if regs:
                print(regs)
            else:
                print("read error")
            time.sleep(0.2)

    def powder_shaker_on(self):
        self.motor_contorller.write_single_register(1086, 10)
        self.motor_contorller.write_single_register(88, 1)
        statue = 0
        while(statue != 3):
            regs = self.motor_contorller.read_holding_registers(88, 1)
            statue = regs[0]
            if regs:
                print(regs)
            else:
                print("read error")
            time.sleep(0.2)

    def grip_rotate_init(self):
        self.motor_contorller.write_single_register(60, 1)
        time.sleep(0.5)

    def grip_recv_position(self):
        self.motor_contorller.write_single_register(61, 1)
        statue = 0
        time_wait = 0.0
        timeout = 30.0  # 30秒超时
        while(statue != 3):
            regs = self.motor_contorller.read_holding_registers(61, 1)
            if regs:
                statue = regs[0]
                print(f"grip_recv_position status: {regs}")
                if statue == 3:
                    print("grip_recv_position completed successfully")
                    break
            else:
                print("grip_recv_position read error")
                break  # 避免无限循环
            
            time_wait += 0.2
            if time_wait >= timeout:
                print(f"grip_recv_position timeout after {timeout} seconds, current status: {statue}")
                break
            time.sleep(0.2)

    def grip_pull_position(self):
        self.motor_contorller.write_single_register(62, 1)
        statue = 0
        time_wait = 0.0
        timeout = 30.0  # 30秒超时
        while(statue != 3):
            regs = self.motor_contorller.read_holding_registers(62, 1)
            if regs:
                statue = regs[0]
                print(f"grip_pull_position status: {regs}")
                if statue == 3:
                    print("grip_pull_position completed successfully")
                    break
            else:
                print("grip_pull_position read error")
                break  # 避免无限循环
            
            time_wait += 0.2
            if time_wait >= timeout:
                print(f"grip_pull_position timeout after {timeout} seconds, current status: {statue}")
                break
            time.sleep(0.2)

    def grip_init(self):
        self.motor_contorller.write_single_register(40, 1)

    # def grip_open(self):
    #     self.motor_contorller.write_single_register(41, 1)
    #     statue = 0
    #     time_wait = 0.0
    #     timeout = 30.0  # 30秒超时
    #     while(statue != 3):
    #         regs = self.motor_contorller.read_holding_registers(41, 1)
    #         if regs:
    #             statue = regs[0]
    #             print(f"grip_open status: {regs}")
    #             if statue == 3:
    #                 print("grip_open completed successfully")
    #                 break
    #         else:
    #             print("grip_open read error")
    #             break  # 避免无限循环
            
    #         time_wait += 0.2
    #         if time_wait >= timeout:
    #             print(f"grip_open timeout after {timeout} seconds, current status: {statue}")
    #             break
    #         time.sleep(0.2)

    def gripper_open(self, open_position=0.1):
        """
        夹爪张开函数
        
        参数:
        open_position: 张开位置 (mm)
        velocity: 运动速度 (mm/s)
        acceleration: 加速度 (mm/s²)
        """
        try:
            self.grip_axis.config_motion(100, 3000, 3000)
            # 使用绝对位置运动控制夹爪张开
            self.grip_axis.move_to(open_position)
            
            # 等待运动完成，添加Modbus通信异常处理
            timeout = 3.0  # 10秒超时
            start_time = time.time()
            
            while True:
                if time.time() - start_time > timeout:
                    print("夹爪张开超时")
                    return False
                    
                try:
                    # 安全地检查夹爪是否到达目标位置
                    position = self.grip_axis.position()
                    print(f"夹爪张开到：{position}mm")
                    if position >= open_position:
                        break
                except Exception as modbus_error:
                    print(f"读取夹爪状态时发生Modbus通信错误: {modbus_error}")
                    # 继续尝试，不立即退出
                    time.sleep(0.05)
                    continue
                    
                time.sleep(0.01)
            
            print(f"夹爪已张开到位置: {open_position}mm")
            return True
        except Exception as e:
            print(f"夹爪张开失败: {e}")
            return False

    # def grip_close(self):
    #     self.motor_contorller.write_single_register(1044, 1)
    #     self.motor_contorller.write_single_register(42, 1)
    #     # statue = 0
    #     # while(statue != 3):
    #     #     regs = self.motor_contorller.read_holding_registers(42, 1)
    #     #     statue = regs[0]
    #     #     if regs:
    #     #         print(regs)
    #     #         print("-----------------------")
    #     #     else:
    #     #         print("read error")
    #     #     time.sleep(0.2)
    #     statue = 2
    #     time_wait = 0.0
    #     timeout = 30.0  # 30秒超时
    #     while(statue != 0 and statue != 3):
    #         regs = self.motor_contorller.read_holding_registers(42, 1)
    #         if regs:
    #             statue = regs[0]
    #             print(f"grip_close status: {regs}")
    #             print("-----------------------")
    #             if statue == 0 or statue == 3:
    #                 print("grip_close completed successfully")
    #                 break
    #         else:
    #             print("grip_close read error")
    #             break  # 避免无限循环
            
    #         time_wait += 0.2
    #         if time_wait >= timeout:
    #             print(f"grip_close timeout after {timeout} seconds, current status: {statue}")
    #             break
    #         time.sleep(0.2)

    def gripper_close(self, close_position=40, timeout=5):
        """
        夹爪闭合函数（带力控制）
        
        参数:
        close_position: 闭合位置 (mm)
        velocity: 运动速度 (mm/s)
        force_limit: 力限制 (0-1之间的百分比，如0.15表示15%)
        timeout: 超时时间 (秒)
        """
        try:
            # 使用推压运动控制夹爪闭合，带力控制
            self.grip_axis.config_motion(100, 3000, 3000)
            self.grip_axis.move_to(close_position)

            position = self.grip_axis.position()
            print(position)
            
            # 等待运动完成或夹持到物体
            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    print("夹爪闭合超时")
                    return False
                
                try:
                    # 安全地检查夹爪状态，处理可能的Modbus通信异常
                    is_moving = self.grip_axis.is_moving()
                    is_reached = self.grip_axis.is_reached()
                    
                    if not is_moving or is_reached:
                        break
                        
                except Exception as modbus_error:
                    print(f"读取夹爪状态时发生Modbus通信错误: {modbus_error}")
                    # 继续尝试，不立即退出
                    time.sleep(0.05)  # 稍微延长等待时间
                    continue
                    
                time.sleep(0.01)
            
            # 再次安全地检查最终状态
            try:
                if self.grip_axis.is_moving():
                    print(f"夹爪已闭合到位置: {close_position}mm")
            except Exception as status_error:
                print(f"读取最终夹爪状态时发生错误: {status_error}")
                print(f"夹爪闭合操作已完成到位置: {close_position}mm")
                
            return True
            
        except Exception as e:
            print(f"夹爪闭合失败: {e}")
            return False

    def gripper_control_move(self, position, timeout=5):
        try:
            self.grip_axis.move_to(position)

            start_time = time.time()
            while True:
                if time.time() - start_time > timeout:
                    print("夹爪操作超时")
                    return False
                
                try:
                    # 安全地检查夹爪状态，处理可能的Modbus通信异常
                    is_moving = self.grip_axis.is_moving()
                    is_reached = self.grip_axis.is_reached()

                    position_1 = self.grip_axis.position()
                    print(position_1)
                    
                    if not is_moving or is_reached:
                        break
                        
                except Exception as modbus_error:
                    print(f"读取夹爪状态时发生Modbus通信错误: {modbus_error}")
                    # 继续尝试，不立即退出
                    time.sleep(0.05)  # 稍微延长等待时间
                    continue
                    
                time.sleep(0.01)
            
            # 再次安全地检查最终状态
            try:
                if self.grip_axis.is_reached():
                    print(f"夹爪已移动到位置: {position}mm")
            except Exception as status_error:
                print(f"读取最终夹爪状态时发生错误: {status_error}")
                print(f"夹爪操作已完成到位置: {position_1}mm")
                
            return {
                    "success": True,
                    "message": "夹爪移动成功"
                }
            
        except Exception as e:
            print(f"夹爪操作失败: {e}")
            return {
                    "success": False,
                    "message": "夹爪移动失败"
                }


    def all_zero(self):
        response = self.slide_controller.write_coils(13, 1)
        self.WaitSlideFinished("z", 0)
        self.WaitSlideFinished("x", 0)
        self.WaitSlideFinished("y", 0)

    def slide_total_mode(self):
        response = self.slide_controller.write_coils(0, 1)

    def slide_move_y(self, dis):
        response = self.slide_controller.write_register(6, dis)
        self.WaitSlideFinished("y", dis)

    def slide_move_x(self, dis):
        response = self.slide_controller.write_register(9, dis)
        self.WaitSlideFinished("x", dis)

    def slide_move_z(self, dis):
        response = self.slide_controller.write_register(12, dis)
        self.WaitSlideFinished("z", dis)

    def caculate_position(self, row, col, start_x=1430, start_y=1210, spacing=186):
        x = start_x + col * spacing
        y = start_y - row * spacing
        return x,y
    
    def pipette_tip_mounting(self, num):
        # caculate position use num
        dights = [int(char) for char in num]
        x,y = self.caculate_position(dights[0], dights[1])
        # print(x)
        # print(y)
        self.slide_total_mode()
        self.slide_move_x(x)
        self.slide_move_y(y)
        self.slide_move_z(1760)
        self.slide_move_z(0)
        self.slide_move_x(0)

    def slide_move_grasp(self):
        # self.slide_move_x(1405)
        self.slide_total_mode()
        self.slide_move_y(1210)
        # time.sleep(4.0)        

    def  pipette_tip_mount(self, num):     
        bench_controller.pipette_tip_mounting(num)
        bench_controller.slide_move_grasp()

    def slide_evaporator_move(self, position):
        response = self.slide_controller.write_register(17, 100)
        response = self.slide_controller.write_register(18, 500)
        response = self.slide_controller.write_register(19, position)

    def liquid_operater(self, speed, aspirate_time, cmd):
        if cmd == "aspirate":
            response = self.slide_controller.write_register(20, speed, slave=1)
            response = self.slide_controller.write_register(22, aspirate_time, slave=1)
            response = self.slide_controller.write_coils(3, 1, slave=1)            
        elif cmd == "spit":
            response = self.slide_controller.write_coils(4, 1)

    def push_pipette_tip(self):
        response = self.slide_controller.write_coils(5, 1)

    def crucible_lid_operater(self, cmd):
        if cmd == "open":
            response = self.slide_controller.write_coils(1, 1)            
        elif cmd == "close":
            response = self.slide_controller.write_coils(2, 1)

    def tube_lid_operater(self):
        self.slide_controller.write_registers(2,self.WriteFloat(0.0), slave=2)

    def lid_operater_init(self):
      # 夹持速度
      self.WriteCmd(4,20.0)
      # 夹持电流
      self.WriteCmd(6,0.5)
      # 旋转速度
      self.WriteCmd(14,540.0)      
      # 旋转电流
      self.WriteCmd(16,2.3)

      self.WriteCmd(2,20.0)
      self.WaitGripperOpen()
      self.WriteCmd(10,0.0)
      time.sleep(2.0)

    def TubeLidClose(self):
        self.WriteCmd(10,0.0)      
        # self.WaitTubeLidClose()
        time.sleep(3.0)
        self.WriteCmd(2,20.0)
        # self.WaitGripperOpen()
        time.sleep(1.2)
        self.WriteCmd(10,0.0)
        time.sleep(1.0)
        # self.WaitLidAngle(0.0)     

    def WriteCmd(self, position, cmd):
        # self.master.execute(1, cst.WRITE_MULTIPLE_REGISTERS, position, output_value=self.WriteFloat(cmd))
        self.slide_controller.write_registers(position,self.WriteFloat(cmd), slave=2)

    def WaitGripperClose(self):
        closed = False 
        time_wait = 0.0
        while not closed:
        # red = self.master.execute(1, cst.READ_HOLDING_REGISTERS, 65, 1)
            red = self.slide_controller.read_holding_registers(65, 1, slave=2).registers[0]
            print(red)
            if red == 2:
                print("closed")
                closed = True
            time_wait += 0.1
            if time_wait >= 5.0:
                raise self.ex    
            time.sleep(0.1)
        return closed
    
    def WaitGripperOpen(self):
        opened = False
        time_wait = 0.0
        while not opened:
            red = self.ReadRegisters(self.slide_controller.read_holding_registers(66, 2, slave=2).registers)
            print(red)
            if red > 19.9:
                print("opened")
                opened = True
            time_wait += 0.1
            if time_wait >= 30.0:
                raise self.ex    
            time.sleep(0.1)
        return opened
    
    def WaitTubeGripperClose(self):
        closed = False 
        time_wait = 0.0
        while not closed:
            red = self.ReadRegisters(self.slide_controller.read_holding_registers(66, 2, slave=2).registers)
            if abs(red) < 0.1:
                print("closed")
                closed = True
            time_wait += 0.1
            if time_wait >= 30.0:
                raise self.ex    
            time.sleep(0.1)
        return closed

    def WaitTubeLidClose(self):
        closed = False
        time_wait = 0.0
        while not closed:
            red = self.ReadRegisters(self.slide_controller.read_holding_registers(78, 2, slave=2).registers)
            print(red)
            if abs(red) > 0.25:
                closed = True
            time_wait += 0.1
            if time_wait >= 30.0:
                raise self.ex    
            time.sleep(0.1)
        return closed
    def WaitLidAngle(self,angle):
        opened = False
        time_wait = 0.0
        while not opened:
            red = self.ReadRegisters(self.slide_controller.read_holding_registers(0x004A, 2, slave=2).registers)
            print(red)
            if abs(red - angle) < 1.0:
                opened = True
            time_wait += 0.1
            if time_wait >= 30.0:
                raise self.ex    
            time.sleep(0.1)
        return opened

    def WaitSlideFinished(self, axis, dis):
        moving = True
        while moving:
            if axis == "y":
                red = self.slide_controller.read_input_registers(10, 1, slave=1)
                red2 = self.slide_controller.read_input_registers(11, 1, slave=1) 
                dis_now = red[0] + red2[0]
                dis_now = red.registers[0] + red2.registers[0]
            elif axis == "x":
                red = self.slide_controller.read_input_registers(12, 1, slave=1)
                dis_now = red[0]
                dis_now = red.registers[0]
            elif axis == "z":
                red = self.slide_controller.read_input_registers(13, 1, slave=1)
                dis_now = red[0]
                dis_now = red.registers[0]

            # if dis_now == dis:
            #     print("%i %i", dis_now, dis)
            #     moving = False
            dis_diff = dis_now - dis
            if dis_diff >=-5 and dis_diff <=5:
                print("%s %i %i", "waiting", dis_now, dis)
                moving = False
            # print(dis_now)
            # print("wait")
            time.sleep(0.3)

    def ReadRegisters(self, value):
        decoder = BinaryPayloadDecoder.fromRegisters(value, Endian.BIG, wordorder=Endian.BIG)
        result = decoder.decode_32bit_float()
        return result
    
    def ReadRegistersInt(self, value):
        decoder = BinaryPayloadDecoder.fromRegisters(value, Endian.BIG, wordorder=Endian.BIG)
        result = decoder.decode_16bit_uint()
        return result
    
    def WriteFloat(self, value, reverse=True):
        y_bytes = struct.pack('!f', value)
        # y_hex = bytes.hex(y_bytes)
        y_hex = ''.join(['%02x' % i for i in y_bytes])
        n, m = y_hex[:-4], y_hex[-4:]
        n, m = int(n, 16), int(m, 16)
        if reverse:
            v = [n, m]
        else:
            v = [m, n]
        return v

    def ReadFloat(self, *args, reverse=True):
        for n, m in args:
            n, m = '%04x' % n, '%04x' % m
        if reverse:
            v = n + m
        else:
            v = m + n
        y_bytes = bytes.fromhex(v)
        y = struct.unpack('!f', y_bytes)[0]
        y = round(y, 6)
        return y

    def gripper_control_push(self, position):
        try:
            self.grip_axis.push(80, 40, position)
            reached = False
            timeout = 10.0  # 10秒超时
            start_time = time.time()
            
            while not reached:
                if time.time() - start_time > timeout:
                    print("夹爪推压操作超时")
                    return False
                    
                try:
                    reached = self.grip_axis.is_reached()
                    print(reached)
                except Exception as modbus_error:
                    print(f"读取夹爪状态时发生Modbus通信错误: {modbus_error}")
                    time.sleep(0.05)
                    continue
                    
                time.sleep(0.05)
            return True
        except Exception as e:
            print(f"夹爪推压操作失败: {e}")
            return False

    # def gripper_control_move(self, position):
    #     try:
    #         self.grip_axis.move_to(position)
    #         moving = True
    #         timeout = 10.0  # 10秒超时
    #         start_time = time.time()
            
    #         while moving:
    #             if time.time() - start_time > timeout:
    #                 print("夹爪移动操作超时")
    #                 return False
                    
    #             try:
    #                 moving = self.grip_axis.is_moving()
    #                 print(moving)
    #             except Exception as modbus_error:
    #                 print(f"读取夹爪状态时发生Modbus通信错误: {modbus_error}")
    #                 time.sleep(0.05)
    #                 continue
                    
    #             time.sleep(0.05)
    #         return True
    #     except Exception as e:
    #         print(f"夹爪移动操作失败: {e}")
    #         return False

    def arm_movel(self, position):
        self.arm_controller.move_l(*self.bench_json[position])

    def arm_movej(self, position):
        self.arm_controller.move_j(*self.bench_json[position])

    def set_arm_param(self):
        bench_controller.gripper_control_move(10)
        bench_controller.arm_controller.CP(100)

    def put_tube2recv(self, num):
        
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["home_left"])

        bench_controller.gripper_control_move(15)
        location_num = "location" + str(num)
        pose = bench_controller.bench_json["tube_racks"][location_num]
        pose_up = pose.copy()
        pose_up[2] += 150.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_push(34)
        bench_controller.arm_controller.move_l(*pose_up)
        
        pose = bench_controller.bench_json["tube_transfer"]
        print(pose)
        pose_up = pose.copy()
        pose_up[2] += 250.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)

        bench_controller.gripper_control_move(0)
        pose_up[2] -= 200.0
        bench_controller.arm_controller.move_l(*pose_up)

        pose = bench_controller.bench_json["tube_transfer2"]
        pose_up = pose.copy()
        pose_up[2] += 130.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)

        bench_controller.gripper_control_push(26)

        bench_controller.arm_controller.move_l(*pose_up)
        pose = bench_controller.bench_json["tube_recv"]
        pose_up = pose.copy()
        pose_up[2] += 120.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_move(7)
        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] += 120.0
        bench_controller.arm_controller.move_l(*pose_up)

    def cap_tube_mini(sel, num):
        bench_controller.gripper_control_move(30)
        location_num = "location" + str(num)

        pose = bench_controller.bench_json["tube_mini_racks"][location_num]
        pose_up = pose.copy()
        pose_up[2] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)

        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_push(40)

        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[0] -= (pose_up[0] - 403.0 + 100.0)
        bench_controller.arm_controller.move_l(*pose_up)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_before"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap"])

        # bench_controller.WriteCmd(2,20)
        bench_controller.WriteCmd(2,0.0)
        # bench_controller.WaitGripperClose()
        time.sleep(1.0)
        bench_controller.WriteCmd(10,1100.0)
        # bench_controller.WaitLidAngle(1100.0)
        time.sleep(2.5)
        # sys.exit()
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_down2"])

    def aspirate_liquid(self):
        pose = bench_controller.bench_json["tube_mini_recv_down"]
        pose_down_throw = pose.copy()
        # pose_down_throw2 = pose.copy()
        pose_down_throw[2] -= 70.0
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_recv_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_recv"])
        bench_controller.liquid_operater(255, 2000, "spit")
        time.sleep(2.0)
        bench_controller.liquid_operater(255, 1000, "aspirate")
        time.sleep(3.0)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_recv_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap"])
        bench_controller.TubeLidClose()
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_cap_down"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_mini_recv_down"])
        bench_controller.arm_controller.move_l(*pose_down_throw)
        bench_controller.gripper_control_move(26)

        pose_down_throw[0] -= 30.0
        bench_controller.arm_controller.move_l(*pose_down_throw)
        pose_down_throw[0] += 30.0
        bench_controller.arm_controller.move_l(*pose_down_throw)
        pose_down_throw[2] += 170.0
        bench_controller.arm_controller.move_l(*pose_down_throw)
        pose_down_throw[1] += 120.0
        pose_down_throw[0] -= 90.0
        bench_controller.arm_controller.move_l(*pose_down_throw)

    def add_liquid(self):
        bench_controller.slide_move_x(680)
        bench_controller.slide_move_z(1500)
        bench_controller.liquid_operater(255, 2000, "spit")
        bench_controller.slide_move_z(0)
        bench_controller.slide_move_x(0)
        bench_controller.slide_move_z(1800)
        bench_controller.push_pipette_tip()
        bench_controller.slide_move_z(0)

    def put_tube2evaporator(self, num):
        bench_controller.gripper_control_move(0)
        joint_angles = bench_controller.arm_controller.get_angle()
        joint6 = joint_angles[5]-180.0
        bench_controller.arm_controller.JointMovJ(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint6)

        pose = bench_controller.bench_json["tube_recv"]
        pose_up = pose.copy()
        pose_up[2] += 120.0
        bench_controller.arm_controller.move_l(*pose_up)

        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_push(26)

        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] += 120.0
        bench_controller.arm_controller.move_l(*pose_up)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_evaporator_before"])

        location_num = "location" + str(num)
        pose = bench_controller.bench_json["tube_evaporator_racks"][location_num]
        pose_up = pose.copy()
        pose_up[2] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)

        bench_controller.gripper_control_move(7)

        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_evaporator_before"])

        joint_angles = bench_controller.arm_controller.get_angle()
        joint1 = joint_angles[0]+45.0
        joint6 = joint_angles[5]+90.0
        bench_controller.arm_controller.JointMovJ(joint1, joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint6)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["home_left"])

    def put_crucible(self, num):
        list, num = num.split("-")

        location_num = "location" + str(int(num))
        if list == "A":
            pose = bench_controller.bench_json["crucible_racks2"][location_num]
        elif list == "B":
            pose = bench_controller.bench_json["crucible_racks1"][location_num]
        pose_up = pose.copy()
        pose_up[2] += 40.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.gripper_control_move(30)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_move(37)
        bench_controller.arm_controller.move_l(*pose_up)

        pose = bench_controller.bench_json["crucible_cap"]
        pose_up = pose.copy()
        pose_up[1] += 100.0
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.crucible_lid_operater("open")
        time.sleep(3.0)
        bench_controller.arm_controller.move_l(*pose_up)
        print("crucible_lid_operater open")

        pose = bench_controller.bench_json["crucible_recv"]
        pose_up = pose.copy()
        pose_up[1] += 100.0
        pose_up[2] += 20.0
        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] -= 100.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_move(30)
        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)

    def put_funnel(self):

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_before"])
        bench_controller.gripper_control_move(13)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_grasp"])
        bench_controller.gripper_control_push(40)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_before"])

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_recv_before"])       
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_recv_up"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_recv"])   
        bench_controller.gripper_control_move(13)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["funnel_recv_before"])   

    def add_tube_powder(self, num):
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["home2"])
        joint_angles = bench_controller.arm_controller.get_angle()
        joint6 = joint_angles[5]-180.0
        bench_controller.arm_controller.JointMovJ(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint6)
        # sys.exit()
        # bench_controller.slide_evaporator_move(0)
        bench_controller.gripper_control_move(7)
        
        # bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_evaporator_before"])
        location_num = "location" + str(num)
        pose = bench_controller.bench_json["tube_evaporator_racks"][location_num]
        pose_up = pose.copy()
        pose_up[2] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)

        bench_controller.gripper_control_push(26)

        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["tube_evaporator_before"])

        #-------------------------------------mix and add------------------------------------
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["oscillator_pusher_before1"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["oscillator_pusher_before2"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["oscillator_pusher"])
        bench_controller.oscillator_pusher_on()
        time.sleep(2.0)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["oscillator_pusher_before2"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["oscillator_pusher_before1"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["magnetic_rotator_before"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["magnetic_rotator"])
        # sys.exit()
        
        bench_controller.magnetic_rotator_on()
        bench_controller.arm_controller.move.Circle3(15, *bench_controller.bench_json["magnetic_rotator2"],*bench_controller.bench_json["magnetic_rotator3"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["magnetic_rotator_after"])

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull_up"])
        bench_controller.grip_open()
        
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull"])
        bench_controller.grip_close()
        bench_controller.gripper_control_move(13)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull_up"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["home"])

        bench_controller.grip_pull_position()

        bench_controller.powder_shaker_on()
        bench_controller.grip_recv_position()
        bench_controller.funnel_shaker_on()
        bench_controller.crucible_shaker_on()

    def put_tube_back(self, num):
        bench_controller.gripper_control_move(10)

        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull_up"])
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull"])
        bench_controller.gripper_control_push(26)
        bench_controller.grip_open()
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["grip_pull_up"])

        pose = bench_controller.bench_json["tube_transfer2"]
        pose_up = pose.copy()
        pose_up[2] += 130.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_move(10)
        bench_controller.arm_controller.move_l(*pose_up)

        pose = bench_controller.bench_json["tube_transfer"]
        print(pose)
        pose_up = pose.copy()
        pose_up[2] += 50.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_push(30)
        pose_up[2] += 200.0
        bench_controller.arm_controller.move_l(*pose_up)

        location_num = "location" + str(num)
        pose = bench_controller.bench_json["tube_racks"][location_num]
        pose_up = pose.copy()
        # pose_up2 = pose.copy()
        pose_up[2] += 150.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_move(19)
        bench_controller.arm_controller.move_l(*pose_up)

    def put_funnel_back(self):

        self.gripper_control_move(10)

        joint_angles = self.arm_controller.get_angle()
        joint6 = joint_angles[5]+90.0
        self.arm_controller.JointMovJ(joint_angles[0], joint_angles[1], joint_angles[2], joint_angles[3], joint_angles[4], joint6)
        # sys.exit()
        self.arm_controller.move_l(*self.bench_json["funnel_recv_up"])
        self.arm_controller.move_l(*self.bench_json["funnel_recv"])
        self.gripper_control_push(33)

        self.arm_controller.move_l(*self.bench_json["funnel_recv_up"])
        pose = self.bench_json["funnel_recv_before"]
        pose_funnel_recv_before = pose.copy()
        self.arm_controller.move_l(*pose_funnel_recv_before)
        pose_funnel_recv_before[0] += 150 
        pose_funnel_recv_before[1] += 50 
        pose_funnel_recv_before[2] += 200 
        self.arm_controller.move_l(*pose_funnel_recv_before)
        self.arm_controller.move_j(*self.bench_json["funnel_pull_out"])
        pose2 = self.bench_json["funnel_pull_out2"]
        pose2[2] -= 30
        self.arm_controller.move_j(*pose2)
        pose2[2] += 30
        self.arm_controller.move_j(*pose2)
        pose2[2] -= 30
        self.arm_controller.move_j(*pose2)
        pose2[2] += 30
        self.arm_controller.move_j(*pose2)

        self.arm_controller.move_j(*self.bench_json["funnel_pull_out"])

        self.arm_controller.move_l(*pose_funnel_recv_before)

        self.arm_controller.move_l(*self.bench_json["funnel_up"])
        self.arm_controller.move_l(*self.bench_json["funnel_before"])
        # self.arm_controller.move_l(*self.bench_json["funnel_grasp"])
        pose = self.bench_json["funnel_grasp"]
        pose_before = pose.copy()
        pose_before[1] += 40
        pose_before[2] -= 7
        self.arm_controller.move_l(*pose_before)
        pose_before[2] += 7
        self.arm_controller.move_l(*pose_before)
        pose_before[1] -= 40
        self.arm_controller.move_l(*pose_before)
        self.gripper_control_move(13)

        self.arm_controller.move_l(*self.bench_json["funnel_before"])
        self.arm_controller.move_l(*self.bench_json["funnel_up"])

    def put_crucible_back(self, num):
        list, num = num.split("-")
        num = int(num)
        bench_controller.gripper_control_move(30)
        pose = bench_controller.bench_json["crucible_recv"]
        pose_up = pose.copy()
        pose_up[1] += 100.0
        pose_up[2] += 20.0
        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] -= 100.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.gripper_control_push(37)
        bench_controller.arm_controller.move_l(*pose_up)
        pose_up[1] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)

        pose = bench_controller.bench_json["crucible_cap"]
        pose_up = pose.copy()
        pose_up[1] += 100.0
        bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*pose)
        bench_controller.crucible_lid_operater("close")
        time.sleep(3.0)
        bench_controller.arm_controller.move_l(*pose_up)
        print("crucible_lid_operater open")

        location_num = "location" + str(num)
        if num <=5:
            pose = bench_controller.bench_json["crucible_racks"][location_num]
            pose_up = pose.copy()
            pose_up[2] += 120.0
            bench_controller.arm_controller.move_l(*pose_up)
            bench_controller.arm_controller.move_l(*pose)
            bench_controller.gripper_control_move(30)
            bench_controller.arm_controller.move_l(*pose_up)
        elif num == 6:
            if list == "A":
                pose = bench_controller.bench_json["crucible_racks2"][location_num]
            elif list == "B":
                pose = bench_controller.bench_json["crucible_racks1"][location_num]
            
            pose_up = pose.copy()
            pose_up[2] += 40.0
            bench_controller.arm_controller.move_l(*pose_up)
            bench_controller.arm_controller.move_l(*pose)
            bench_controller.gripper_control_move(30)
            bench_controller.arm_controller.move_l(*pose_up)

            for i in range(1,6):
                num = 6-i
                location_num = "location" + str(num)
                pose = bench_controller.bench_json["crucible_racks"][location_num]
                pose_up = pose.copy()
                pose_up[2] += 60.0
                bench_controller.arm_controller.move_l(*pose_up)
                bench_controller.arm_controller.move_l(*pose)
                bench_controller.gripper_control_push(37)
                bench_controller.arm_controller.move_l(*pose_up)

                if list == "A":
                    pose = bench_controller.bench_json["crucible_racks2"][location_num]
                elif list == "B":
                    pose = bench_controller.bench_json["crucible_racks1"][location_num]
                
                pose_up = pose.copy()
                pose_up[2] += 50.0
                bench_controller.arm_controller.move_l(*pose_up)
                bench_controller.arm_controller.move_l(*pose)
                bench_controller.gripper_control_move(30)
                bench_controller.arm_controller.move_l(*pose_up)
        bench_controller.arm_controller.move_l(*bench_controller.bench_json["home_left"])

    def evaporate(self):
        print("evaporate")
        # time.sleep(20.0)
        self.slide_evaporator_move(4700)
        time.sleep(3.0)
        time.sleep(10.0)
        self.rotavapor_controller.pub_cmd("start")
        print("start")
        time.sleep(15.0)
        self.rotavapor_controller.pub_cmd("find_position_start")
        print("find_position_start")
        time.sleep(3.0)
        running = True
        while running:
            #TODO: read position
            red = self.slide_controller.read_input_registers(0, 2, slave=11)
            red2 = self.slide_controller.read_input_registers(0, 2, slave=10)
            red = int(str(red.registers[1]), 16)/10000.0
            red2 = int(str(red2.registers[1]), 16)/10000.0
            print(red)
            print(red2)

            if red >= 32.5 and red <= 33.5 and red2 >= 29.5 and red2 <= 30.5:
                self.rotavapor_controller.pub_cmd("stop")
                running = False
        self.slide_evaporator_move(0)
        time.sleep(10.0)
        self.change_status("finish")

    def move_crucible2slide(self, tray_name, cmd):
        self.slide_total_mode()
        if cmd == "move_out":
            self.slide_move_y(10900)
            self.change_status("finish")
            return
        elif cmd == "move_back":
            self.slide_move_y(0)
            self.change_status("finish")
            return
        print('func')
        # if "tray_name" == "eva-tube":
        #     self.slide_move_y(10900)
        if cmd == "pick":
            print('pick')
            if tray_name == "cru-a":
                self.slide_move_y(5200)
                self.gripper_control_move(0)
                pose = self.bench_json["tray"]["cru_a"]
                pose2 = self.bench_json["tray"]["cru_a_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up2)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose)
                self.gripper_control_push(25)
                self.arm_controller.move_l(*pose_up)
            elif tray_name == "cru-b":
                self.slide_move_y(5200)
                self.gripper_control_move(0)
                pose = self.bench_json["tray"]["cru_b"]
                pose2 = self.bench_json["tray"]["cru_b_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up2)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose)
                self.gripper_control_push(25)
                self.arm_controller.move_l(*pose_up)
            elif tray_name == "tube":
                print('tube')
                self.slide_move_y(5200)
                self.gripper_control_move(0)
                pose = self.bench_json["tray"]["tube"]
                pose2 = self.bench_json["tray"]["tube_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up2)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose)
                self.gripper_control_push(25)
                self.arm_controller.move_l(*pose_up)

            elif tray_name == "pipette":
                self.slide_move_y(6650) #TODO:7200
                #pick up
                self.gripper_control_move(0)
                pose = self.bench_json["tray"]["pipette"]
                pose2 = self.bench_json["tray"]["pipette_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up2)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose)
                self.gripper_control_push(25)
                self.arm_controller.move_l(*pose_up)
                self.slide_move_y(5200)
        # place 
            pose = self.bench_json["tray"]["eva_tube"]
            pose2 = self.bench_json["tray"]["eva_tube_before"]
            pose_up2 = pose2.copy()
            pose_up2[2] += 100.0
            pose_up = pose.copy()
            pose_up[2] += 100.0
            self.arm_controller.move_l(*pose_up)
            self.arm_controller.move_l(*pose)
            self.gripper_control_move(0)
            self.arm_controller.move_l(*pose2)
            self.arm_controller.move_l(*pose_up2)
            # move_out
            self.slide_move_y(10900)

        elif cmd == "place":
            pose = self.bench_json["tray"]["eva_tube"]
            pose2 = self.bench_json["tray"]["eva_tube_before"]
            pose_up2 = pose2.copy()
            pose_up2[2] += 100.0
            pose_up = pose.copy()
            pose_up[2] += 100.0
            self.arm_controller.move_l(*pose_up2)
            self.arm_controller.move_l(*pose2)
            self.gripper_control_move(0)
            self.arm_controller.move_l(*pose)
            self.arm_controller.move_l(*pose_up)
            self.gripper_control_push(25)

            if tray_name == "cru-a":
                self.slide_move_y(5200)
                pose = self.bench_json["tray"]["cru_a"]
                pose2 = self.bench_json["tray"]["cru_a_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up)
                self.arm_controller.move_l(*pose)
                self.gripper_control_move(0)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose_up2)
            elif tray_name == "cru-b":
                self.slide_move_y(5200)
                pose = self.bench_json["tray"]["cru_b"]
                pose2 = self.bench_json["tray"]["cru_b_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up)
                self.arm_controller.move_l(*pose)
                self.gripper_control_move(0)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose_up2)
            elif tray_name == "tube":
                self.slide_move_y(5200)
                pose = self.bench_json["tray"]["tube"]
                pose2 = self.bench_json["tray"]["tube_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up)
                self.arm_controller.move_l(*pose)
                self.gripper_control_move(0)
                self.arm_controller.move_l(*pose2)
                self.arm_controller.move_l(*pose_up2)
            elif tray_name == "pipette":
                self.slide_move_y(6650) #TODO:7200
                #pick up
                pose = self.bench_json["tray"]["pipette"]
                pose2 = self.bench_json["tray"]["pipette_before"]
                pose_up = pose.copy()
                pose_up[2] += 100.0
                pose_up2 = pose2.copy()
                pose_up2[2] += 100.0
                self.arm_controller.move_l(*pose_up)
                self.arm_controller.move_l(*pose)
                self.gripper_control_move(0)
                self.arm_controller.move_l(*pose2)
                self.gripper_control_push(25)
                self.arm_controller.move_l(*pose_up2)
                self.slide_move_y(5200)
            self.slide_move_y(0)

            # move_back
            
        self.change_status("finish")

    def add_tube2evaporator(self, pip_num, tube_num):
        print("add_tube2evaporator")
        print("%s %s", pip_num, tube_num)
        # time.sleep(20.0)
        print("mount")
        self.pipette_tip_mount(pip_num)
        # data = {"location_code":"CS-01", "value": "1"}
        # response = requests.post(self.url, json=data) 
        print("put_tube2recv")
        self.put_tube2recv(tube_num)
        print("cap_tube_mini")
        self.cap_tube_mini(tube_num)
        print("aspirate_liquid")
        self.aspirate_liquid()
        self.add_liquid()
        self.put_tube2evaporator(tube_num)
        self.change_status("finish")

    def make_sample(self, crucible_num, tube_num):    
        print("make_sample")
        print("%s %s", crucible_num, tube_num)
        # time.sleep(20.0) 
        self.put_crucible(crucible_num)
        self.put_funnel()
        self.add_tube_powder(tube_num)
        self.put_tube_back(tube_num)
        self.put_funnel_back()
        self.put_crucible_back(crucible_num)
        self.change_status("finish")

    def change_status(self, step):

        if step == "finish":            
            self.order_info["status"] = "success"
            self.order_info["detail"] = "success"
            self.bench_status = "idle"        
        elif step == "run":
            self.order_info["status"] = "running"
            self.order_info["detail"] = "running"
            self.bench_status = "running"
        elif step == "idle":            
            self.order_info["status"] = "default"
            self.order_info["detail"] = "idle"
            self.bench_status = "idle"    
        elif step == "fail":            
            self.order_info["status"] = "failed"
            self.order_info["detail"] = "failed"
            self.bench_status = "failed"  

    def task_thread(self):
        bench_controller.set_arm_param()
        # bench_controller.add_tube2evaporator("00", 1)
        # sys.exit()
        # bench_controller.add_tube2evaporator("01", 2)
        # bench_controller.add_tube2evaporator("02", 3)
        # bench_controller.add_tube2evaporator("03", 4)
        # bench_controller.add_tube2evaporator("04", 5)
        # bench_controller.add_tube2evaporator("05", 6)
        # bench_controller.add_tube2evaporator("06", 7)
        # bench_controller.add_tube2evaporator("07", 8)
        # bench_controller.add_tube2evaporator("10", 9)
        # bench_controller.add_tube2evaporator("11", 10)
        # bench_controller.add_tube2evaporator("12", 11)
        # bench_controller.add_tube2evaporator("13", 12)
        # bench_controller.evaporate()
        bench_controller.slide_move_grasp()
        bench_controller.make_sample("B-01", 12)
        bench_controller.make_sample("B-02", 11)
        bench_controller.make_sample("B-03", 10)
        bench_controller.make_sample("B-04", 9)
        bench_controller.make_sample("B-05", 8)
        bench_controller.make_sample("B-06", 7)
        bench_controller.make_sample("A-01", 6)
        bench_controller.make_sample("A-02", 5)
        bench_controller.make_sample("A-03", 4)
        bench_controller.make_sample("A-04", 3)
        bench_controller.make_sample("A-05", 2)
        bench_controller.make_sample("A-06", 1)

if __name__ == '__main__':

    bench_controller = BenchContorller()

    time.sleep(3.0)
    bench_controller.move_crucible2slide("cru-a", "pick")
    sys.exit()


    time.sleep(2.0)
    bench_controller.slide_total_mode()
    bench_controller.slide_move_y(1210)
    sys.exit()

    # --------------------------------------------task------------------------------------------------
    bench_controller.set_arm_param()
    # bench_controller.add_tube2evaporator("00", 1)
    # bench_controller.add_tube2evaporator("01", 2)
    # bench_controller.add_tube2evaporator("02", 3)
    # bench_controller.add_tube2evaporator("03", 4)
    # bench_controller.add_tube2evaporator("04", 5)
    # bench_controller.add_tube2evaporator("05", 6)
    # bench_controller.add_tube2evaporator("06", 7)
    # bench_controller.add_tube2evaporator("07", 8)
    # bench_controller.add_tube2evaporator("10", 9)
    # bench_controller.add_tube2evaporator("11", 10)
    # bench_controller.add_tube2evaporator("12", 11)
    # bench_controller.add_tube2evaporator("13", 12)
    # bench_controller.evaporate()
    bench_controller.slide_move_grasp()
    # bench_controller.make_sample("B-01", 12)
    # bench_controller.make_sample("B-02", 11)
    # bench_controller.make_sample("B-03", 10)
    # bench_controller.make_sample("B-04", 9)
    bench_controller.make_sample("B-05", 8)
    bench_controller.make_sample("B-06", 7)
    bench_controller.make_sample("A-01", 6)
    bench_controller.make_sample("A-02", 5)
    bench_controller.make_sample("A-03", 4)
    bench_controller.make_sample("A-04", 3)
    bench_controller.make_sample("A-05", 2)
    bench_controller.make_sample("A-06", 1)


