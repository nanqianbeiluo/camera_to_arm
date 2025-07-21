#!/usr/bin/env python3
# -*- coding: utf-8 -*-

from numpy import tri
import yaml
from flask import Flask, request
from flask_restful import Api
import json
from controllers import BenchContorller
import uuid
import time
from datetime import datetime
import threading
import logging
import json

app = Flask(__name__)
api = Api(app)

bc = BenchContorller()

@app.route('/api/order', methods=['POST'])
def bench_order_request():
    """订单请求
    Returns:
        string: 请求结果
    """ 
    print(request.json)
    
    # if bc._task_new:
    #     return_data = {"code": 1, "message": "order denied, busy"}
    #     return return_data, 401
    # if bc._repair_state:
    #     return_data = {"code": 1, "message": "order denied, repair"}
    #     return return_data, 401
        #    {"pip_num":"00","dada":"01","aaa":"01"}
    # 执行新任务
    bc._task_new = True
    json_data = request.json
    order_info = {"key_name":"", "task_id":"","params":""}

    bc.order_info["key_name"] = json_data['key_name']
    bc.order_info["task_id"] = json_data['task_id']
    bc.order_info["params"] = json_data['params']

    params =  bc.order_info["params"]
    # 异步线程开启任务执行
    return_data = {"code": 0, "message": "order accepted"}
    bc.change_status("run")
    time.sleep(0.1)
    if bc.order_info["key_name"] == "tube2evaporator":
        bc.t_task = threading.Thread(target=bc.add_tube2evaporator,args=(params["pip_num"], params["tube_num"]))
    elif bc.order_info["key_name"] == "evaporate":
        bc.t_task = threading.Thread(target=bc.evaporate,args=())
    elif bc.order_info["key_name"] == "make_sample":
        bc.t_task = threading.Thread(target=bc.make_sample,args=(params["crucible_num"], params["tube_num"]))
    elif bc.order_info["key_name"] == "crucible2slide":
        bc.t_task = threading.Thread(target=bc.move_crucible2slide,args=(params["cmd"], params["tray_name"]))
    # elif bc.order_info["key_name"] == "auto_pick_circles":
    #     safe_height = params.get("safe_height", 700.0)
    #     pick_height = params.get("pick_height", 500)
    #     max_circles = params.get("max_circles", 1)
    #     bc.t_task = threading.Thread(target=bc.auto_pick_circles, args=(safe_height, pick_height, max_circles))
    # elif bc.order_info["key_name"] == "pick_sequence":
    #     target_x = params["target_x"]
    #     target_y = params["target_y"]
    #     safe_height = params.get("safe_height", 700.0)
    #     pick_height = params.get("pick_height", 500)
    #     bc.t_task = threading.Thread(target=bc.execute_pick_sequence, args=(target_x, target_y, safe_height, pick_height))
    elif bc.order_info["key_name"] == "pick_sequence_from_queue":
        approach_height = params.get("approach_height", 700.0)
        pick_height = params.get("pick_height", 500)
        timeout = params.get("timeout", 10.0)
        recognition_scheme = params.get("recognition_scheme", "start1")
        bc.t_task = threading.Thread(target=bc.execute_pick_sequence_from_queue, args=(approach_height, pick_height, timeout, recognition_scheme))
        
    bc.t_task.start()
    return_data = json.dumps(return_data)
    return return_data, 200

@app.route('/api/status', methods=['GET'])
def bench_status_request():
    """订单请求
    Returns:
        string: 请求结果
    """
    json_data = {"status":bc.bench_status, "task":{"task_id": bc.order_info["task_id"],"order": bc.order_info["key_name"],"status": bc.order_info["status"],"detail": bc.order_info["detail"]}}

    # status = mc.AmrStatus()
    return_json = json.dumps(json_data)
    # TODO：何时返回错误代码，完全无法获取状态时，如果只能获取部分状态，则返回部分设备状态以及错误情况
    return return_json, 200

@app.route('/api/params', methods=['POST'])
def ench_post_params_request():
    # if mc.t_status != None:
    #     mc._async_raise(mc.t_status.ident, SystemExit)
    json_data = {}
    return json_data

@app.route('/api/params', methods=['GET'])
def bench_get_params_request():
    # if mc.t_status != None:
    #     mc._async_raise(mc.t_status.ident, SystemExit)
    json_data = {}
    return json_data

@app.route('/api/operate', methods=['POST'])
def bench_operate_request():
    if not mc.arm_status[1] and not (mc.arm_status[0] or mc.arm_status[2] or mc.arm_status[3] or mc.arm_status[4] or mc.arm_status[5]):
        return_data = {"code": 1, "message": "cmd reject, can not start", "extra": "arm in none status"}
        return return_data, 401
    #TODO: 开始机器人，使能，开始程序
    
    try:
        pass
        # mc.ModbusStart()
        # mode = mc.GetArmMode()
    except Exception as e:
        return_data = {"code": 0, "message": "start failed", "extra": e}
        return return_data, 401
    
    finish_code = 0
    if mode == "7":
        json_data = {"code": 1, "message": "succeed", "extra": mode}
        finish_code = 200
    else:
        json_data = {"code": 0, "message": "failed", "extra": mode}
        finish_code = 401
    return json_data, finish_code

@app.route('/api/move_gripper', methods=['POST'])
def move_gripper_request():
    """控制夹爪移动
    
    请求参数:
        position: 目标位置 (mm)
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json
        if not json_data or 'position' not in json_data:
            return {
                "success": False,
                "message": "缺少必需参数 'position'",
                "error_code": "MISSING_PARAMETER"
            }, 400
        
        position = json_data['position']
        
        # 调用控制器方法
        result = bc.gripper_control_move(position=position)
        
        if result["success"]:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/control_gripper', methods=['POST'])
def control_gripper_request():
    """控制夹爪开合
    
    请求参数:
        action: 动作类型，"open" 或 "close"
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json
        if not json_data or 'action' not in json_data:
            return {
                "success": False,
                "message": "缺少必需参数 'action'",
                "error_code": "MISSING_PARAMETER"
            }, 400
        
        action = json_data['action']
        
        # 调用控制器方法
        result = bc.control_gripper(action=action)
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/camera_status', methods=['GET'])
def camera_status_request():
    """获取相机状态
    
    Returns:
        JSON: 相机连接状态信息
    """
    try:
        from camera.camera import get_camera
        camera = get_camera()
        status = camera.get_connection_status()
        return {
            "success": True,
            "camera_status": status
        }, 200
    except Exception as e:
        return {
            "success": False,
            "message": f"获取相机状态失败: {str(e)}",
            "error_code": "CAMERA_STATUS_ERROR"
        }, 500

@app.route('/api/capture_image', methods=['POST'])
def capture_image_request():
    """单独控制相机拍照并按日期保存图片
    
    请求参数:
        save_directory: 保存目录，默认为"images"
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json or {}
        save_directory = json_data.get('save_directory', 'images')
        
        # 调用控制器方法
        result = bc.capture_and_save_image(save_directory=save_directory)
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"拍照API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/move_arm', methods=['POST'])
def move_arm_request():
    """单独控制机械臂x,y,z移动
    
    请求参数:
        x: X轴坐标（毫米）
        y: Y轴坐标（毫米）
        z: Z轴坐标（毫米）
        rx: X轴旋转角度（度），默认0
        ry: Y轴旋转角度（度），默认0
        rz: Z轴旋转角度（度），默认0
        movement_type: 运动类型，"joint"或"linear"，默认"joint"
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json
        if not json_data or 'x' not in json_data or 'y' not in json_data or 'z' not in json_data:
            return {
                "success": False,
                "message": "缺少必需参数 'x', 'y', 'z'",
                "error_code": "MISSING_PARAMETER"
            }, 400
        
        x = json_data['x']
        y = json_data['y']
        z = json_data['z']
        rx = json_data.get('rx', 0)
        ry = json_data.get('ry', 0)
        rz = json_data.get('rz', 0)
        movement_type = json_data.get('movement_type', 'joint')
        
        # 调用控制器方法
        result = bc.control_arm_movement(
            x=x, y=y, z=z, rx=rx, ry=ry, rz=rz, 
            movement_type=movement_type
        )
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"机械臂移动API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/pick_sequence_from_queue', methods=['POST'])
def pick_sequence_from_queue_request():
    """
    从TCP队列中获取坐标并执行完整的抓取序列
    
    请求参数:
        timeout: 等待坐标数据的超时时间（秒），默认5.0
        approach_height: 接近高度（毫米），默认700.0
        pick_height: 抓取高度（毫米）
        command: 识别方案，可以是"start", "start1", "start2"等，默认"start1"
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json or {}
        timeout = json_data.get('timeout', 5.0)
        approach_height = json_data.get('approach_height', 700)
        pick_height = json_data.get('pick_height', 0)
        recognition_scheme = json_data.get('command', 'start1')
        
        # 调用控制器方法
        result = bc.execute_pick_sequence_from_queue(approach_height, pick_height, timeout, recognition_scheme)

        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"队列抓取序列API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/put_back', methods=['POST'])
def put_back_request():
    """
    从TCP队列中获取坐标并执行完整的放回序列
    
    请求参数:
        timeout: 等待坐标数据的超时时间（秒），默认5.0
        approach_height: 接近高度（毫米），默认700.0
        pick_height:放回高度（毫米），默认446
        command: 识别方案，可以是"start", "start1", "start2"等，默认"start1"
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json or {}
        timeout = json_data.get('timeout', 5.0)
        approach_height = json_data.get('approach_height', 700)
        pick_height = json_data.get('pick_height', 0)
        recognition_scheme = json_data.get('command', 'start1')
        
        # 调用控制器方法
        result = bc.put_back(approach_height, pick_height, timeout, recognition_scheme)
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"队列放回序列API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/send_tcp_command', methods=['POST'])
def send_tcp_command_request():
    """
    向TCP服务器发送指定命令
    
    请求参数:
        command: 要发送的命令，可以是"start", "start1", "start2"等，默认"start1"
        clear_queue: 是否清空队列中的旧数据，默认True
    
    Returns:
        JSON: 执行结果
    """
    try:
        json_data = request.json or {}
        command = json_data.get('command', 'start1')
        clear_queue = json_data.get('clear_queue', True)
        
        # 调用控制器方法
        result = bc.send_start_command(command=command, clear_queue=clear_queue)
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"发送TCP命令API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/send_start_command', methods=['POST'])
def send_start_command_request():
    """
    向TCP服务器发送start命令并清空队列数据（保持向后兼容）
    
    Returns:
        JSON: 执行结果
    """
    try:
        # 调用控制器方法
        result = bc.send_start_command()
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"发送start命令API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

@app.route('/api/tcp_status', methods=['GET'])
def get_tcp_status_request():
    """
    获取TCP客户端连接状态
    
    Returns:
        JSON: TCP连接状态信息
    """
    try:
        # 调用控制器方法
        result = bc.get_tcp_status()
        
        if result['success']:
            return result, 200
        else:
            return result, 400
            
    except Exception as e:
        error_result = {
            "success": False,
            "message": f"获取TCP状态API调用异常: {str(e)}",
            "error_code": "API_ERROR"
        }
        return error_result, 500

if __name__ == '__main__':
    app.run(host="0.0.0.0",
            port=5999,
            debug=False)
