# 海康工业相机控制模块

## 概述

本模块提供海康工业相机的完整控制功能，包括图像采集、目标检测、坐标转换等功能，主要用于视觉引导的机器人应用。

## 文件结构

```
camera/
├── camera.py              # 主要相机控制类
├── MvImport/              # 海康相机SDK封装
│   ├── MvCameraControl_class.py    # 相机控制类
│   ├── CameraParams_header.py      # 相机参数定义
│   ├── PixelType_header.py         # 像素类型定义
│   ├── lib/                        # SDK动态链接库
│   └── ...
└── README.md              # 本文档
```

## 主要功能

### 1. 相机连接与控制

- **自动设备枚举**: 自动检测并连接可用的海康工业相机
- **连接状态管理**: 实时监控相机连接状态，支持断线重连
- **参数配置**: 支持触发模式、曝光时间等参数设置

### 2. 图像采集

- **实时图像采集**: 支持连续图像采集
- **多种像素格式**: 支持Mono8、RGB8、BGR8等多种像素格式
- **格式自动转换**: 自动将不同格式转换为标准BGR格式

### 3. 目标检测

- **圆形检测**: 基于霍夫变换的圆形目标检测
- **参数可调**: 支持最小/最大半径、检测阈值等参数调整
- **结果可视化**: 在图像上标注检测结果并保存

### 4. 坐标转换

- **像素到机器人坐标**: 将图像像素坐标转换为机器人坐标系
- **手眼标定支持**: 支持加载Vision Master标定文件
- **多点转换**: 批量转换多个目标点坐标

## 使用方法

### 基本使用

```python
from controllers.camera.camera import get_camera

# 获取相机实例（单例模式）
camera = get_camera()

# 检查连接状态
if camera.is_connected:
    # 采集图像
    image = camera.capture_image()
    
    # 检测圆形目标
    circles = camera.detect_circles(image)
    
    # 获取机器人坐标
    robot_coords = camera.get_circle_centers_in_robot_coords(z_height=100.0)
    
    print(f"检测到 {len(circles)} 个圆形目标")
    for i, (x, y, z) in enumerate(robot_coords):
        print(f"目标 {i+1}: 机器人坐标 ({x:.2f}, {y:.2f}, {z:.2f})")
else:
    print("相机未连接")
```

### 高级功能

```python
# 自定义检测参数
circles = camera.detect_circles(
    image,
    min_radius=50,      # 最小半径
    max_radius=200,     # 最大半径
    dp=1.0,            # 累加器分辨率
    min_dist=100,      # 最小距离
    param1=50,         # Canny高阈值
    param2=30          # 累加器阈值
)

# 保存带检测结果的图像
success = camera.save_image_with_detection(
    save_path="public/images/detection_result.jpg",
    min_radius=50,
    max_radius=200
)

# 手动重连相机
if not camera.is_connected:
    success = camera.reconnect()
    if success:
        print("相机重连成功")
    else:
        print("相机重连失败")
```

## 配置文件

### 手眼标定文件

标定文件位置: `params/hand_eye_calibration.json`

```json
{
    "camera_matrix": [
        [fx, 0, cx],
        [0, fy, cy],
        [0, 0, 1]
    ],
    "transformation_matrix": [
        [r11, r12, r13, tx],
        [r21, r22, r23, ty],
        [r31, r32, r33, tz],
        [0, 0, 0, 1]
    ]
}
```

- `camera_matrix`: 相机内参矩阵
- `transformation_matrix`: 手眼标定变换矩阵

## API 参考

### HikCamera 类

#### 构造函数
```python
__init__(camera_index: int = 0)
```
- `camera_index`: 相机索引，默认为0

#### 主要方法

##### capture_image()
```python
capture_image() -> Optional[np.ndarray]
```
采集一帧图像，返回BGR格式的numpy数组。

##### detect_circles()
```python
detect_circles(image: np.ndarray, 
              min_radius: int = 1000, 
              max_radius: int = 1500,
              dp: float = 1.0,
              min_dist: int = 50,
              param1: int = 50,
              param2: int = 30) -> List[Tuple[int, int, int]]
```
检测图像中的圆形目标。

**参数:**
- `image`: 输入图像
- `min_radius`: 最小半径
- `max_radius`: 最大半径
- `dp`: 累加器分辨率与图像分辨率的反比
- `min_dist`: 检测到的圆心之间的最小距离
- `param1`: Canny边缘检测的高阈值
- `param2`: 累加器阈值

**返回:** 检测到的圆形列表，每个元素为(x, y, radius)

##### pixel_to_robot_coordinates()
```python
pixel_to_robot_coordinates(pixel_x: int, pixel_y: int, z_height: float = 0.0) -> Tuple[float, float, float]
```
将像素坐标转换为机器人坐标系坐标。

##### get_circle_centers_in_robot_coords()
```python
get_circle_centers_in_robot_coords(z_height: float = 0.0) -> List[Tuple[float, float, float]]
```
获取图像中所有圆心在机器人坐标系下的坐标。

##### save_image_with_detection()
```python
save_image_with_detection(save_path: str = "public/images/detection_result.jpg", 
                         min_radius: int = 10, max_radius: int = 100,
                         dp: float = 1.0, min_dist: int = 50,
                         param1: int = 50, param2: int = 30) -> bool
```
保存带有检测结果的图像。

##### reconnect()
```python
reconnect() -> bool
```
手动重连相机。

##### get_connection_status()
```python
get_connection_status() -> dict
```
获取相机连接状态信息。

### 全局函数

##### get_camera()
```python
get_camera() -> HikCamera
```
获取相机实例（单例模式）。

##### release_camera()
```python
release_camera()
```
释放相机实例。

## 注意事项

1. **SDK依赖**: 需要安装海康工业相机SDK，确保MvImport文件夹中的库文件完整
2. **权限要求**: 在Windows系统下可能需要管理员权限来访问相机设备
3. **单例模式**: 相机实例采用单例模式，避免多次初始化
4. **资源管理**: 程序结束时会自动释放相机资源，也可手动调用release()方法
5. **标定文件**: 为获得准确的坐标转换结果，建议使用Vision Master进行手眼标定

## 故障排除

### 常见问题

1. **相机连接失败**
   - 检查相机电源和网络连接
   - 确认相机IP地址设置正确
   - 检查防火墙设置

2. **图像采集失败**
   - 检查相机触发模式设置
   - 确认相机参数配置正确
   - 检查SDK库文件是否完整

3. **检测结果不准确**
   - 调整检测参数（半径范围、阈值等）
   - 检查图像质量和光照条件
   - 确认目标物体符合检测要求

4. **坐标转换错误**
   - 检查标定文件是否正确加载
   - 确认手眼标定精度
   - 验证坐标系定义是否一致

### 日志信息

模块使用统一的日志系统，相关日志信息会记录在 `logs/` 目录下，可通过日志文件排查问题。

## 更新日志

- **v1.0.0**: 初始版本，支持基本的图像采集和圆形检测功能
- **v1.1.0**: 添加手眼标定支持和坐标转换功能
- **v1.2.0**: 优化连接管理和错误处理机制