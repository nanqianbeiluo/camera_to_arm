# Dobot机械臂控制模块

## 概述

本模块提供Dobot机械臂的完整控制功能，包括关节运动、直线运动、夹爪控制等，支持实体机械臂和模拟模式，主要用于自动化装配和视觉引导应用。

## 文件结构

```
dobot/
├── arm.py                 # 主要机械臂控制类
├── libs/                  # Dobot SDK库文件
└── README.md              # 本文档
```

## 主要功能

### 1. 连接管理

- **自动连接**: 自动连接到指定IP的Dobot机械臂
- **模拟模式**: 当实体机械臂不可用时自动切换到模拟模式
- **连接状态监控**: 实时监控机械臂连接状态
- **错误处理**: 自动清除机械臂错误状态

### 2. 运动控制

- **关节运动(MovJ)**: 基于关节角度的运动控制
- **直线运动(MovL)**: 基于笛卡尔坐标的直线运动
- **位置获取**: 实时获取当前关节角度和笛卡尔坐标
- **运动等待**: 自动等待运动完成并验证到位精度

### 3. 夹爪控制

- **开合控制**: 支持夹爪的开启和关闭操作
- **位置控制**: 精确控制夹爪开合位置
- **力控制**: 支持夹爪力度控制（通过Modbus）
- **状态反馈**: 实时获取夹爪当前位置

### 4. 预设动作

- **初始化位置**: 多种预设的初始化位置
- **安全位置**: 预定义的安全操作位置
- **标准动作**: 常用的标准操作动作

## 配置参数

### 连接配置

```python
DOBOT_IP = '192.168.1.106'    # Dobot机械臂IP地址
DOBOT_PORT = 29999            # Dobot通信端口
SLAVE_ID_US = 2               # 超声波传感器Modbus从站ID
SLAVE_ID_GRIPPER = 1          # 夹爪Modbus从站ID
```

### 运动参数

```python
JOINTS_LIMIT = [0.1] * 6      # 关节运动误差限制(度)
POINT_LIMIT = [0.1] * 6       # 直线运动误差限制(mm/度)
GRIPPER_AXIS_LIMIT = 1.0      # 夹爪轴误差限制(mm)
GRIPPER_FORCE_LIMIT = 0.1     # 夹爪力误差限制
ACC = 100                     # 加速度
VEL = 3                       # 速度
```

## 使用方法

### 基本使用

```python
from controllers.dobot.arm import arm

# 创建机械臂实例
robot = arm(simulation_mode=False)  # False为实体模式，True为模拟模式

# 检查连接状态
if robot.is_connected:
    print("机械臂连接成功")
    
    # 获取当前位置
    current_joints = robot.get_angle()  # 获取关节角度
    current_pose = robot.get_point()    # 获取笛卡尔坐标
    
    print(f"当前关节角度: {current_joints}")
    print(f"当前位置: {current_pose}")
    
    # 执行初始化动作
    robot.home1()  # 移动到初始位置1
    
    # 夹爪操作
    robot.open_gripper()   # 打开夹爪
    robot.close_gripper()  # 关闭夹爪
else:
    print("机械臂连接失败，使用模拟模式")
```

### 关节运动

```python
# 关节运动到指定角度
target_joints = [0, -45, 135, 0, -90, 0]  # [J1, J2, J3, J4, J5, J6]
limits = [0.1] * 6  # 每个关节的误差限制

robot.move_j(target_joints, limits)
print("关节运动完成")
```

### 直线运动

```python
# 直线运动到指定位置
target_pose = [300, 0, 200, 0, 0, 0]  # [X, Y, Z, Rx, Ry, Rz]
limits = [0.1] * 6  # 位置和姿态的误差限制

robot.move_l(target_pose, limits)
print("直线运动完成")
```

### 夹爪控制

```python
# 基本夹爪操作
robot.open_gripper()    # 完全打开夹爪
robot.close_gripper()   # 关闭夹爪到预设位置
robot.gripper_move()    # 移动到特定位置

# 获取夹爪状态
gripper_pos = robot._gripper_pose()
print(f"当前夹爪位置: {gripper_pos}mm")
```

### 预设动作

```python
# 初始化位置1（侧面位置）
robot.home1()  # 关节角度: (-90, -45, 135, 0, -90, 0)

# 初始化位置2（正面位置）
robot.home2()  # 关节角度: (0, -45, 135, 0, -90, 0)
```

## API 参考

### arm 类

#### 构造函数
```python
__init__(simulation_mode=False)
```
- `simulation_mode`: 是否启用模拟模式，默认False

#### 位置获取方法

##### get_angle()
```python
get_angle() -> List[float]
```
获取当前关节角度。

**返回:** 6个关节的角度值列表 [J1, J2, J3, J4, J5, J6]

##### get_point()
```python
get_point() -> List[float]
```
获取当前笛卡尔坐标。

**返回:** 位置和姿态列表 [X, Y, Z, Rx, Ry, Rz]

#### 运动控制方法

##### move_j()
```python
move_j(joints: List[float], limits: List[float])
```
关节运动到指定角度。

**参数:**
- `joints`: 目标关节角度列表
- `limits`: 每个关节的误差限制

##### move_l()
```python
move_l(point: List[float], limits: List[float])
```
直线运动到指定位置。

**参数:**
- `point`: 目标位置和姿态列表
- `limits`: 位置和姿态的误差限制

#### 夹爪控制方法

##### open_gripper()
```python
open_gripper()
```
打开夹爪到完全开启位置。

##### close_gripper()
```python
close_gripper()
```
关闭夹爪到预设抓取位置。

##### gripper_move()
```python
gripper_move()
```
移动夹爪到特定位置。

#### 预设动作方法

##### home1()
```python
home1()
```
移动到初始化位置1（侧面位置）。

##### home2()
```python
home2()
```
移动到初始化位置2（正面位置）。

#### 内部方法

##### _wait_for_result()
```python
_wait_for_result(get_current_func, target, limits)
```
等待运动完成并验证到位精度。

##### _move_gripper()
```python
_move_gripper(position: float, limit: float)
```
精确控制夹爪位置。

##### _gripper_pose()
```python
_gripper_pose() -> float
```
获取夹爪当前位置。

##### _us_get_dis()
```python
_us_get_dis() -> int
```
获取超声波传感器距离。

## 坐标系说明

### 关节坐标系

- **J1**: 基座旋转关节（-180° ~ +180°）
- **J2**: 大臂俯仰关节（-90° ~ +90°）
- **J3**: 小臂俯仰关节（-10° ~ +170°）
- **J4**: 手腕旋转关节（-180° ~ +180°）
- **J5**: 手腕俯仰关节（-90° ~ +90°）
- **J6**: 末端旋转关节（-180° ~ +180°）

### 笛卡尔坐标系

- **X轴**: 机械臂前方为正方向（mm）
- **Y轴**: 机械臂左侧为正方向（mm）
- **Z轴**: 机械臂上方为正方向（mm）
- **Rx**: 绕X轴旋转角度（度）
- **Ry**: 绕Y轴旋转角度（度）
- **Rz**: 绕Z轴旋转角度（度）

## 模拟模式

当实体机械臂不可用时，系统会自动切换到模拟模式：

- **模拟状态**: 维护虚拟的关节角度和位置状态
- **运动模拟**: 模拟运动时间和状态更新
- **调试支持**: 便于程序调试和测试
- **无缝切换**: API接口保持一致

```python
# 强制启用模拟模式
robot = arm(simulation_mode=True)

# 模拟模式下的操作与实体模式完全相同
robot.home1()
robot.move_j([0, 0, 0, 0, 0, 0], [0.1] * 6)
```

## 安全注意事项

1. **工作空间**: 确保机械臂工作空间内无障碍物和人员
2. **急停按钮**: 操作前确认急停按钮功能正常
3. **速度限制**: 调试时使用较低的运动速度
4. **关节限位**: 注意各关节的运动范围限制
5. **夹爪安全**: 夹爪操作时注意防止夹伤
6. **网络连接**: 确保网络连接稳定，避免通信中断

## 故障排除

### 常见问题

1. **连接失败**
   ```
   机械臂连接失败: [Errno 10061] No connection could be made
   ```
   - 检查机械臂电源是否开启
   - 确认IP地址和端口配置正确
   - 检查网络连接状态
   - 确认防火墙设置

2. **运动异常**
   ```
   运动未到达目标位置
   ```
   - 检查目标位置是否在工作空间内
   - 确认关节限位设置
   - 检查运动参数配置
   - 验证误差限制设置

3. **夹爪控制失败**
   ```
   夹爪无响应
   ```
   - 检查Modbus连接状态
   - 确认夹爪电源和通信线路
   - 检查从站ID配置
   - 验证夹爪初始化状态

4. **模拟模式切换**
   ```
   自动切换到模拟模式
   ```
   - 这是正常的保护机制
   - 检查实体机械臂连接状态
   - 修复连接问题后重新初始化

### 调试技巧

1. **启用详细日志**: 查看控制台输出的详细状态信息
2. **分步测试**: 先测试基本连接，再测试运动功能
3. **使用模拟模式**: 在模拟模式下验证程序逻辑
4. **检查状态**: 定期检查机械臂和夹爪状态

## 集成示例

### 与视觉系统集成

```python
from controllers.camera.camera import get_camera
from controllers.dobot.arm import arm

# 初始化设备
camera = get_camera()
robot = arm()

# 视觉引导抓取
if camera.is_connected and robot.is_connected:
    # 移动到拍照位置
    robot.home2()
    
    # 获取目标坐标
    robot_coords = camera.get_circle_centers_in_robot_coords(z_height=50)
    
    if robot_coords:
        target_x, target_y, target_z = robot_coords[0]
        
        # 移动到目标上方
        robot.move_l([target_x, target_y, target_z + 50, 0, 0, 0], [1.0] * 6)
        
        # 下降到抓取位置
        robot.move_l([target_x, target_y, target_z, 0, 0, 0], [1.0] * 6)
        
        # 抓取
        robot.close_gripper()
        
        # 提升
        robot.move_l([target_x, target_y, target_z + 50, 0, 0, 0], [1.0] * 6)
        
        print("抓取完成")
```

## 更新日志

- **v1.0.0**: 初始版本，支持基本的运动控制和夹爪操作
- **v1.1.0**: 添加模拟模式支持
- **v1.2.0**: 优化连接管理和错误处理
- **v1.3.0**: 添加超声波传感器支持和预设动作