# 机械臂抓取放回系统演示文档

## 概述

本系统实现了基于TCP通信的机械臂自动抓取和放回功能，支持多种识别方案和智能排序算法。系统通过接收vision master发送的坐标数据，自动执行机械臂的抓取和放置操作。

## 核心功能

### 1. 抓取功能 (pick_up)

自动从队列中获取坐标数据并执行完整的抓取序列。

**主要步骤：**
1. 移动到初始点位
2. 发送识别方案命令
3. 获取并排序坐标数据
4. 打开夹爪
5. 移动到目标上方安全位置
6. 下降到抓取位置
7. 关闭夹爪进行抓取
8. 提升到安全高度
9. 回到原点

### 2. 放回功能 (put_back)

将物体放置到指定位置的完整序列。

**主要步骤：**
1. 发送识别方案命令
2. 获取并排序坐标数据
3. 移动到目标上方安全位置
4. 下降到放置位置
5. 打开夹爪进行放置
6. 提升到安全高度
7. 回到原点

## 识别方案配置

### 配置文件

系统使用 `recognition_schemes.json` 配置文件来管理不同识别方案的参数，包括：

- **gripper_open_pos**: 夹爪打开位置
- **gripper_close_pos**: 夹爪关闭位置
- **arm_rotation**: 机械臂旋转角度
- **origin_point**: 原点坐标（包含6个参数：X, Y, Z, RX, RY, RZ）
- **default_pick_height**: 默认抓取高度
- **target_z_default**: 默认目标Z坐标

### 配置示例

```json
{
  "start1": {
    "gripper_open_pos": 33,
    "gripper_close_pos": 35,
    "arm_rotation": 270,
    "origin_point": [-0.0268, -363.0517, 772.0800, 180, 0, 270],
    "default_pick_height": 446.0,
    "target_z_default": 500
  },
  "start2": {
    "gripper_open_pos": 16.8,
    "gripper_close_pos": 25.5,
    "arm_rotation": 90,
    "origin_point": [93.0233, -370.0210, 772.0800, 180, 0, 90],
    "default_pick_height": 525.0,
    "target_z_default": 600
  }
}
```

## 坐标排序算法

### 算法原理

本系统实现了一个智能的坐标排序算法，用于优化机械臂的抓取路径。算法的核心特点包括：

1. **Y轴误差容忍度**：引入3mm的Y轴误差容忍度，将相近的Y坐标归为同一组
2. **分组机制**：通过 `round(y_coord / 3.0) * 3.0` 实现Y坐标的智能分组
3. **旋转角度适配**：根据配置文件中原点数组的最后一个参数（旋转角度）确定排序策略
4. **路径优化**：确保机械臂按照最优路径进行操作，提高效率

### 排序逻辑

```python
def get_y_group(y_coord, tolerance=3.0):
    """将Y坐标按3mm误差分组"""
    return round(y_coord / tolerance) * tolerance

# 获取识别方案的旋转角度
scheme_config = self.recognition_schemes.get(recognition_scheme, {})
origin_point = scheme_config.get('origin_point', [0, 0, 0, 0, 0, 270])
rotation_angle = origin_point[-1]  # 获取原点数组的最后一个数值（旋转角度）

if rotation_angle == 270:
    # 旋转角度为270时：先按Y轴从大到小分组，组内X轴从大到小
    sorted_coordinates = sorted(all_coordinates, 
        key=lambda coord: (-get_y_group(coord['y']), -coord['x']))
    print(f"使用270度旋转排序规则：先按Y轴从大到小分组（3mm容忍度），组内X轴从大到小")
elif rotation_angle == 90:
    # 旋转角度为90时：先按Y轴从大到小分组，组内X轴从小到大
    sorted_coordinates = sorted(all_coordinates, 
        key=lambda coord: (-get_y_group(coord['y']), coord['x']))
    print(f"使用90度旋转排序规则：先按Y轴从大到小分组（3mm容忍度），组内X轴从小到大")
else:
    # 默认方案：先按Y轴从大到小分组，组内X轴从小到大
    sorted_coordinates = sorted(all_coordinates, 
        key=lambda coord: (-get_y_group(coord['y']), coord['x']))
    print(f"使用默认排序规则（旋转角度{rotation_angle}）：先按Y轴从大到小分组（3mm容忍度），组内X轴从小到大")
```

#### 旋转角度为270度的方案
- **Y轴分组**：先按Y轴从大到小分组（3mm容忍度内认为同一行）
- **X轴排序**：在每个Y轴组内，按X轴从大到小排序
- **抓取顺序**：从右到左，从上到下的顺序进行抓取
- **适用场景**：通常用于坩埚等需要从右侧开始抓取的场景

#### 旋转角度为90度的方案及其他角度
- **Y轴分组**：先按Y轴从大到小分组（3mm容忍度内认为同一行）
- **X轴排序**：在每个Y轴组内，按X轴从小到大排序
- **抓取顺序**：从左到右，从上到下的顺序进行抓取
- **适用场景**：通常用于玻璃管等需要从左侧开始抓取的场景

### 误差容忍度机制
- **3mm容忍度**：Y轴坐标差值在3mm以内的点被视为同一行
- **分组原理**：通过`round(y_coord / 3.0) * 3.0`将相近的Y坐标归为同一组
- **优势**：避免因微小的坐标偏差导致的排序混乱

**实际效果：**
假设有以下坐标点（考虑3mm误差容忍度）：
- 点A: (100, 200.5)
- 点B: (150, 199.8) 
- 点C: (100, 151.2)
- 点D: (150, 149.5)

由于A、B两点Y轴差值小于3mm，被归为同一行；C、D两点同样被归为同一行。

**配置示例：**
- start1方案：origin_point最后一个参数为270
- start2方案：origin_point最后一个参数为90

**旋转角度270度排序结果（start1方案）：** B → A → D → C
（第一行：右到左，第二行：右到左）

**旋转角度90度排序结果（start2方案）：** A → B → C → D
（第一行：左到右，第二行：左到右）

**优势：**
- **智能分组**：避免微小偏差影响排序结果，3mm容忍度确保稳定性
- **Y轴优先**：确保机械臂按行进行有序操作，减少跨行移动
- **配置驱动**：通过配置文件管理不同方案参数，便于维护和扩展
- **角度适配**：根据机械臂旋转角度自动选择最优排序策略
- **路径优化**：减少不必要的移动距离，提高整体操作效率
- **方案灵活**：支持多种识别方案，适应不同的抓取场景

## API 接口

### 1. 抓取接口

**端点：** `POST /api/pick_up`

**请求参数：**
```json
{
    "timeout": 10.0,           // 等待坐标数据超时时间（秒）
    "approach_height": 700.0,  // 接近高度（毫米）
    "pick_height": 446.0,      // 抓取高度（毫米）
    "command": "start1"        // 识别方案
}
```

**响应示例：**
```json
{
    "success": true,
    "message": "抓取序列执行成功",
    "recognition_scheme": "start1",
    "received_coordinates": {
        "x": 123.456,
        "y": 234.567,
        "z": 500
    },
    "executed_positions": {
        "approach_height": 700.0,
        "pick_height": 446.0
    }
}
```

### 2. 放回接口

**端点：** `POST /api/put_back`

**请求参数：**
```json
{
    "timeout": 10.0,           // 等待坐标数据超时时间（秒）
    "approach_height": 700.0,  // 接近高度（毫米）
    "pick_height": 446.0,      // 放置高度（毫米）
    "command": "start1"        // 识别方案
}
```

## 超时机制

系统为每个操作步骤都配置了5秒超时保护：

- **机械臂移动操作**：使用 `_move_with_timeout` 方法
- **夹爪操作**：`gripper_control_move`方法内置超时
- **坐标数据等待**：可配置的超时时间

**超时处理：**
- 自动停止当前操作
- 返回详细的错误信息
- 确保系统安全性和可靠性

## TCP 通信机制

### 连接配置
```python
TCP_SERVER_HOST = "127.0.0.1"  # 服务器地址
TCP_SERVER_PORT = 7930        # 服务器端口
```

### 数据格式
系统接收JSON格式的坐标数据：
```json
{
    "x": 123.456,  // X轴坐标
    "y": 234.567,  // Y轴坐标
    "z": 345.678   // Z轴坐标（可选）
}
```

### cURL 示例

```bash
# 抓取操作
curl -X POST http://localhost:5000/api/pick_up \
  -H "Content-Type: application/json" \
  -d '{
    "timeout": 10.0,
    "approach_height": 700.0,
    "pick_height": 446.0,
    "command": "start1"
  }'

# 放回操作
curl -X POST http://localhost:5000/api/put_back \
  -H "Content-Type: application/json" \
  -d '{
    "timeout": 10.0,
    "approach_height": 700.0,
    "pick_height": 446.0,
    "command": "start1"
  }'
```

## 系统架构

```
┌─────────────────┐    TCP    ┌─────────────────┐    API    ┌─────────────────┐
│   视觉识别服务器  │ ────────→ │   坐标队列管理    │ ────────→ │   机械臂控制器    │
│                 │           │                │           │                │
│ - 物体识别       │            │ - 数据接收      │            │ - 抓取执行      │
│ - 坐标计算       │            │ - 智能排序      │            │ - 放回执行      │
│ - 数据发送       │            │ - 队列管理      │            │ - 超时保护      │
└─────────────────┘           └─────────────────┘           └─────────────────┘
```
