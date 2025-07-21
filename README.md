# 工程说明
## 文件说明
dobot_arm_controller:越疆python控制驱动
app.py:fastapi服务程序
controller.py:主控程序，各设备模块控制与任务流程
rotavapor_controller.py:仪器控制程序
bench.json:点位参数
config.yaml:相机标定及二维码参数

## 更新dobot_arm_controller版本
https://github.com/Dobot-Arm/TCP-IP-Python-V4

## 调试
1. 模块控制：modbus通信
2. 基础点位调试：修改越疆机械臂版本，机械臂控制
3. 视觉识别与操作：海康相机+Apriltag（梅卡曼德）

## 工作台流程
等待硬件部门提供
