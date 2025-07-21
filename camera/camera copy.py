import cv2
import numpy as np
import json
import os
import cv2
from typing import Tuple, Optional, List
from utils.logger import logger
from .MvImport.MvCameraControl_class import *
from .MvImport.CameraParams_header import *
from .MvImport.PixelType_header import *
from .MvImport.PixelType_header import (
    PixelType_Gvsp_BayerGR8, PixelType_Gvsp_BayerRG8, 
    PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerBG8,
    PixelType_Gvsp_Mono10, PixelType_Gvsp_Mono12, PixelType_Gvsp_Mono16
)
from ctypes import *

class HikCamera:
    """
    海康工业相机控制类
    用于图像采集、目标识别和坐标转换
    """
    
    def __init__(self, camera_index: int = 0):
        """
        初始化相机
        
        Args:
            camera_index: 相机索引，默认为0
        """
        self.camera_index = camera_index
        self.cam = MvCamera()
        self.device_list = MV_CC_DEVICE_INFO_LIST()
        self.is_connected = False
        self.calibration_matrix = None
        self.transformation_matrix = None
        self.data_buf = None
        self.nDataSize = 0
        self.reconnect_attempts = 0
        self.max_reconnect_attempts = 3
        
        # 加载标定文件
        self._load_calibration()
        
        # 尝试连接相机
        self._connect_camera()
    
    def _connect_camera(self):
        """
        连接相机
        """
        try:
            # 如果已经连接，先断开
            if self.is_connected:
                self._disconnect_camera()
            
            # 枚举设备
            tlayerType = MV_GIGE_DEVICE | MV_USB_DEVICE
            ret = MvCamera.MV_CC_EnumDevices(tlayerType, self.device_list)
            if ret != 0:
                logger.error(f"枚举设备失败，错误码: {ret}")
                self.is_connected = False
                return
            
            if self.device_list.nDeviceNum == 0:
                logger.warning("未发现相机设备")
                self.is_connected = False
                return
            
            # 检查相机索引是否有效
            if self.camera_index >= self.device_list.nDeviceNum:
                logger.error(f"相机索引 {self.camera_index} 超出范围，共发现 {self.device_list.nDeviceNum} 台设备")
                self.is_connected = False
                return
            
            # 选择设备并创建句柄
            stDeviceInfo = cast(self.device_list.pDeviceInfo[self.camera_index], POINTER(MV_CC_DEVICE_INFO)).contents
            ret = self.cam.MV_CC_CreateHandle(stDeviceInfo)
            if ret != 0:
                logger.error(f"创建设备句柄失败，错误码: {ret}")
                self.is_connected = False
                return
            
            # 打开设备
            ret = self.cam.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
            if ret != 0:
                logger.error(f"打开设备失败，错误码: {ret}")
                self.is_connected = False
                return
            
            # 设置触发模式为off
            ret = self.cam.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
            if ret != 0:
                logger.warning(f"设置触发模式失败，错误码: {ret}")
            
            # 获取数据包大小
            stParam = MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
            ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                logger.error(f"获取数据包大小失败，错误码: {ret}")
                self.is_connected = False
                return
            
            self.nDataSize = stParam.nCurValue * 2  # 预留2倍空间用于像素格式转换
            self.data_buf = (c_ubyte * self.nDataSize)()
            logger.info(f"初始化缓冲区大小: {self.nDataSize} (PayloadSize: {stParam.nCurValue})")
            
            # 开始取流
            ret = self.cam.MV_CC_StartGrabbing()
            if ret != 0:
                logger.error(f"开始取流失败，错误码: {ret}")
                self.is_connected = False
                return
            
            self.is_connected = True
            self.reconnect_attempts = 0  # 重置重连计数
            logger.info(f"海康相机 {self.camera_index} 连接成功")
            
        except Exception as e:
            logger.error(f"相机连接异常: {e}")
            self.is_connected = False
    
    def _disconnect_camera(self):
        """
        断开相机连接
        """
        try:
            if self.is_connected:
                # 停止取流
                ret = self.cam.MV_CC_StopGrabbing()
                if ret != 0:
                    logger.warning(f"停止取流失败，错误码: {ret}")
                
                # 关闭设备
                ret = self.cam.MV_CC_CloseDevice()
                if ret != 0:
                    logger.warning(f"关闭设备失败，错误码: {ret}")
                
                # 销毁句柄
                ret = self.cam.MV_CC_DestroyHandle()
                if ret != 0:
                    logger.warning(f"销毁句柄失败，错误码: {ret}")
                
                self.is_connected = False
                logger.info("相机连接已断开")
        except Exception as e:
            logger.error(f"断开相机连接异常: {e}")
            self.is_connected = False
    
    def _check_connection(self) -> bool:
        """
        检查相机连接状态
        
        Returns:
            True: 连接正常，False: 连接异常
        """
        if not self.is_connected:
            return False
        
        try:
            # 尝试获取设备信息来检测连接状态
            stParam = MVCC_INTVALUE()
            memset(byref(stParam), 0, sizeof(MVCC_INTVALUE))
            ret = self.cam.MV_CC_GetIntValue("PayloadSize", stParam)
            if ret != 0:
                logger.warning(f"检测到相机连接异常，错误码: {ret}")
                self.is_connected = False
                return False
            return True
        except Exception as e:
            logger.warning(f"检测相机连接状态异常: {e}")
            self.is_connected = False
            return False
    
    def _auto_reconnect(self) -> bool:
        """
        自动重连相机
        
        Returns:
            True: 重连成功，False: 重连失败
        """
        if self.reconnect_attempts >= self.max_reconnect_attempts:
            logger.error(f"相机重连次数已达上限 {self.max_reconnect_attempts}，停止重连")
            return False
        
        self.reconnect_attempts += 1
        logger.info(f"尝试重连相机，第 {self.reconnect_attempts} 次")
        
        # 先断开现有连接
        self._disconnect_camera()
        
        # 等待一段时间后重连
        import time
        time.sleep(1)
        
        # 尝试重新连接
        self._connect_camera()
        
        if self.is_connected:
            logger.info("相机重连成功")
            return True
        else:
            logger.warning(f"相机重连失败，第 {self.reconnect_attempts} 次")
            return False
    
    def _load_calibration(self):
        """
        加载Vision Master标定文件
        """
        calibration_file = "params/hand_eye_calibration.json"
        if os.path.exists(calibration_file):
            try:
                with open(calibration_file, 'r', encoding='utf-8') as f:
                    calibration_data = json.load(f)
                
                # 提取相机内参矩阵
                if 'camera_matrix' in calibration_data:
                    self.calibration_matrix = np.array(calibration_data['camera_matrix'])
                
                # 提取手眼标定变换矩阵
                if 'transformation_matrix' in calibration_data:
                    self.transformation_matrix = np.array(calibration_data['transformation_matrix'])
                elif 'hand_eye_matrix' in calibration_data:
                    self.transformation_matrix = np.array(calibration_data['hand_eye_matrix'])
                
                logger.info("标定文件加载成功")
            except Exception as e:
                logger.error(f"标定文件加载失败: {e}")
        else:
            logger.warning("标定文件不存在，将使用默认参数")
            # 设置默认的变换矩阵（单位矩阵）
            self.transformation_matrix = np.eye(4)
    
    def capture_image(self) -> Optional[np.ndarray]:
        """
        采集图像
        
        Returns:
            图像数组，如果采集失败返回None
        """
        # 只检查基本连接状态，不进行深度连接检查
        if not self.is_connected:
            logger.error("相机未连接，无法采集图像")
            return None
        
        try:
            stFrameInfo = MV_FRAME_OUT_INFO_EX()
            memset(byref(stFrameInfo), 0, sizeof(stFrameInfo))
            
            # 获取一帧数据，增加重试机制
            max_retries = 3
            for retry in range(max_retries):
                ret = self.cam.MV_CC_GetOneFrameTimeout(byref(self.data_buf), self.nDataSize, stFrameInfo, 1000)
                if ret == 0:
                    break
                elif retry < max_retries - 1:
                    logger.warning(f"获取图像失败，错误码: {ret}，重试 {retry + 1}/{max_retries}")
                    import time
                    time.sleep(0.1)  # 短暂等待后重试
                else:
                    logger.error(f"获取图像失败，错误码: {ret}，已重试 {max_retries} 次")
                    # 标记连接异常，但不自动重连
                    self.is_connected = False
                    return None
            
            # 转换图像格式
            if stFrameInfo.enPixelType == PixelType_Gvsp_Mono8:
                # 单通道灰度图
                image_data = np.frombuffer(self.data_buf, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight), dtype=np.uint8)
                image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth))
                # 转换为BGR格式
                image = cv2.cvtColor(image, cv2.COLOR_GRAY2BGR)
            elif stFrameInfo.enPixelType == PixelType_Gvsp_RGB8_Packed:
                # RGB格式
                image_data = np.frombuffer(self.data_buf, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 3), dtype=np.uint8)
                image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
                # 转换为BGR格式
                image = cv2.cvtColor(image, cv2.COLOR_RGB2BGR)
            elif stFrameInfo.enPixelType == PixelType_Gvsp_BGR8_Packed:
                # BGR格式
                image_data = np.frombuffer(self.data_buf, count=int(stFrameInfo.nWidth * stFrameInfo.nHeight * 3), dtype=np.uint8)
                image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
            else:
                # 其他格式，尝试转换为BGR
                logger.warning(f"不支持的像素格式: {stFrameInfo.enPixelType}，图像尺寸: {stFrameInfo.nWidth}x{stFrameInfo.nHeight}，帧长度: {stFrameInfo.nFrameLen}")
                
                # 根据像素格式计算正确的缓冲区大小
                if stFrameInfo.enPixelType in [PixelType_Gvsp_BayerGR8, PixelType_Gvsp_BayerRG8, 
                                              PixelType_Gvsp_BayerGB8, PixelType_Gvsp_BayerBG8]:  # Bayer格式
                    # Bayer格式是单通道，每像素1字节
                    src_data_size = stFrameInfo.nWidth * stFrameInfo.nHeight
                elif stFrameInfo.enPixelType in [PixelType_Gvsp_Mono10, PixelType_Gvsp_Mono12, PixelType_Gvsp_Mono16]:  # 高位深度单通道
                    # 高位深度单通道格式，每像素2字节
                    src_data_size = stFrameInfo.nWidth * stFrameInfo.nHeight * 2
                else:
                    # 默认使用实际帧长度
                    src_data_size = stFrameInfo.nFrameLen
                
                logger.info(f"计算的源数据大小: {src_data_size}，当前缓冲区大小: {self.nDataSize}")
                
                # 确保源数据缓冲区足够大，为像素格式转换预留额外空间
                required_buffer_size = max(src_data_size, stFrameInfo.nFrameLen) * 2  # 预留2倍空间
                if required_buffer_size > self.nDataSize:
                    logger.warning(f"缓冲区不足，需要 {required_buffer_size}，当前 {self.nDataSize}，重新分配")
                    # 重新分配更大的缓冲区
                    self.nDataSize = required_buffer_size
                    self.data_buf = (c_ubyte * self.nDataSize)()
                    logger.info(f"重新分配缓冲区大小: {self.nDataSize}")
                    return None  # 本次采集失败，下次会使用新缓冲区
                
                # 使用SDK的直接BGR转换功能
                nConvertSize = stFrameInfo.nWidth * stFrameInfo.nHeight * 3
                pBGRData = (c_ubyte * nConvertSize)()
                
                logger.info(f"尝试直接BGR转换，目标缓冲区大小: {nConvertSize}")
                
                # 使用MV_CC_GetImageForBGR直接获取BGR格式图像
                ret = self.cam.MV_CC_GetImageForBGR(pBGRData, nConvertSize, stFrameInfo, 1000)
                if ret != 0:
                    logger.error(f"直接BGR转换失败，错误码: {ret}，尝试通用转换")
                    
                    # 如果直接转换失败，使用通用像素格式转换
                    stConvertParam = MV_CC_PIXEL_CONVERT_PARAM()
                    memset(byref(stConvertParam), 0, sizeof(stConvertParam))
                    stConvertParam.nWidth = stFrameInfo.nWidth
                    stConvertParam.nHeight = stFrameInfo.nHeight
                    stConvertParam.pSrcData = cast(self.data_buf, POINTER(c_ubyte))
                    stConvertParam.nSrcDataLen = stFrameInfo.nFrameLen  # 使用实际帧长度
                    stConvertParam.enSrcPixelType = stFrameInfo.enPixelType
                    stConvertParam.enDstPixelType = PixelType_Gvsp_BGR8_Packed
                    
                    # 分配更大的目标缓冲区
                    nConvertSize = stFrameInfo.nWidth * stFrameInfo.nHeight * 4  # 预留更多空间
                    stConvertParam.pDstBuffer = (c_ubyte * nConvertSize)()
                    stConvertParam.nDstBufferSize = nConvertSize
                    
                    ret = self.cam.MV_CC_ConvertPixelType(stConvertParam)
                    if ret != 0:
                        logger.error(f"像素格式转换失败，错误码: {ret}")
                        return None
                    
                    # 只取前3个通道的数据
                    actual_size = stFrameInfo.nWidth * stFrameInfo.nHeight * 3
                    image_data = np.frombuffer(stConvertParam.pDstBuffer, count=actual_size, dtype=np.uint8)
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
                else:
                    logger.info("直接BGR转换成功")
                    image_data = np.frombuffer(pBGRData, count=nConvertSize, dtype=np.uint8)
                    image = image_data.reshape((stFrameInfo.nHeight, stFrameInfo.nWidth, 3))
            
            return image
            
        except Exception as e:
            logger.error(f"图像采集异常: {e}")
            return None
    
    def detect_circles(self, image: np.ndarray, 
                      min_radius: int = 50, 
                      max_radius: int = 150,
                      dp: float = 0.9,
                      min_dist: int = 50,
                      param1: int = 100,
                      param2: int = 15) -> List[Tuple[int, int, int]]:
        """
        检测图像中的圆形，仅使用霍夫圆检测、Canny边缘检测和亮度检测
        """
        try:
            # 转换为灰度图像
            gray = cv2.cvtColor(image, cv2.COLOR_BGR2GRAY)
            
            # 仅使用霍夫圆检测方法
            hough_circles = self._detect_circles_by_hough(gray, min_radius, max_radius, dp, min_dist, param1, param2)
            
            if len(hough_circles) > 0:
                logger.info(f"霍夫圆检测到 {len(hough_circles)} 个圆形")
            else:
                logger.warning("霍夫圆检测未检测到圆形")
            
            # 限制检测结果数量，只保留最大的6个圆形
            if len(hough_circles) > 6:
                # 按半径排序，保留最大的6个
                hough_circles.sort(key=lambda c: c[2], reverse=True)
                hough_circles = hough_circles[:6]
                logger.info(f"检测到过多圆形，已筛选为最大的6个")
            
            return hough_circles
            
        except Exception as e:
            logger.error(f"圆形检测异常: {e}")
            return []
    

    
    def _filter_nearby_circles(self, circles: List[Tuple[int, int, int]], min_dist: int) -> List[Tuple[int, int, int]]:
        """
        过滤距离太近的圆形
        """
        if not circles:
            return []
        
        filtered = []
        for circle in circles:
            x1, y1, r1 = circle
            is_valid = True
            
            for existing in filtered:
                x2, y2, r2 = existing
                distance = np.sqrt((x1 - x2) ** 2 + (y1 - y2) ** 2)
                if distance < min_dist:
                    is_valid = False
                    break
            
            if is_valid:
                filtered.append(circle)
        
        return filtered
    
    def _detect_circles_by_hough(self, gray: np.ndarray, min_radius: int, max_radius: int, dp: float, min_dist: int, param1: int, param2: int) -> List[Tuple[int, int, int]]:
        """
        使用霍夫圆检测方法，结合Canny边缘检测和亮度验证
        """
        try:
            # 确保参数为整数类型
            min_radius = int(min_radius)
            max_radius = int(max_radius)
            min_dist = int(min_dist)
            param1 = int(param1)
            param2 = int(param2)
            
            # 1. 高斯模糊去噪，保持图像平滑
            blurred = cv2.GaussianBlur(gray, (5, 5), 0)
            
            # 2. CLAHE自适应直方图均衡化，更好地增强局部对比度
            clahe = cv2.createCLAHE(clipLimit=3.0, tileGridSize=(8, 8))
            clahe_enhanced = clahe.apply(blurred)
            
            # 3. 应用形态学操作增强边缘
            kernel = np.ones((3, 3), np.uint8)
            morphed = cv2.morphologyEx(clahe_enhanced, cv2.MORPH_GRADIENT, kernel)
            
            # 4. 再次应用高斯模糊平滑处理后的图像
            enhanced = cv2.GaussianBlur(morphed, (3, 3), 0)
            
            # 3. 霍夫圆检测 - 在增强后的灰度图像上进行检测
            circles = cv2.HoughCircles(
                enhanced,
                cv2.HOUGH_GRADIENT,
                dp=dp,
                minDist=min_dist,
                param1=param1,
                param2=param2,
                minRadius=min_radius,
                maxRadius=max_radius
            )
            
            # 如果第一次检测失败，尝试更宽松的参数
            if circles is None:
                circles = cv2.HoughCircles(
                    enhanced,
                    cv2.HOUGH_GRADIENT,
                    dp=1.0,
                    minDist=int(min_dist * 0.8),
                    param1=int(param1 * 0.8),
                    param2=int(param2 * 0.7),
                    minRadius=int(min_radius * 0.9),
                    maxRadius=int(max_radius * 1.1)
                )
            
            # 第三次尝试，进一步放宽参数
            if circles is None:
                circles = cv2.HoughCircles(
                    enhanced,
                    cv2.HOUGH_GRADIENT,
                    dp=1.1,
                    minDist=int(min_dist * 0.6),
                    param1=int(param1 * 0.6),
                    param2=int(param2 * 0.4),
                    minRadius=int(min_radius * 0.8),
                    maxRadius=int(max_radius * 1.2)
                )
            
            # 第四次尝试，最宽松的参数
            if circles is None:
                circles = cv2.HoughCircles(
                    enhanced,
                    cv2.HOUGH_GRADIENT,
                    dp=1.2,
                    minDist=int(min_dist * 0.4),
                    param1=int(param1 * 0.5),
                    param2=int(param2 * 0.3),
                    minRadius=int(min_radius * 0.7),
                    maxRadius=int(max_radius * 1.3)
                )
            
            detected_circles = []
            if circles is not None:
                circles = np.round(circles[0, :]).astype("int")
                
                # 按照圆心坐标排序，确保结果稳定
                circles_list = [(x, y, r) for (x, y, r) in circles]
                circles_list.sort(key=lambda c: (c[1], c[0]))  # 先按y坐标，再按x坐标排序
                
                for (x, y, r) in circles_list:
                    # 验证圆心是否在图像边界内
                    if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                        # 简化验证：只进行基本的半径和位置检查
                        if r >= min_radius and r <= max_radius:
                            detected_circles.append((x, y, r))
                            logger.info(f"检测到有效圆心: ({x}, {y}), 半径: {r}")
                        else:
                            logger.debug(f"圆心 ({x}, {y}) 半径不符合要求，跳过 (半径: {r})")
                    else:
                        logger.debug(f"圆心 ({x}, {y}) 超出图像边界，跳过")
            
            return detected_circles
            
        except Exception as e:
            logger.error(f"霍夫圆检测异常: {e}")
            return []
    
    def _check_circle_edge_continuity(self, gray: np.ndarray, center_x: int, center_y: int, radius: int) -> float:
        """
        检查圆形边缘的连续性
        
        Args:
            gray: 灰度图像
            center_x: 圆心X坐标
            center_y: 圆心Y坐标
            radius: 半径
        
        Returns:
            边缘连续性分数 (0-1)
        """
        try:
            # 在圆周上采样点
            angles = np.linspace(0, 2*np.pi, 36)  # 每10度采样一个点
            edge_points = []
            
            for angle in angles:
                x = int(center_x + radius * np.cos(angle))
                y = int(center_y + radius * np.sin(angle))
                
                # 确保点在图像范围内
                if 0 <= x < gray.shape[1] and 0 <= y < gray.shape[0]:
                    edge_points.append((x, y))
            
            if len(edge_points) < 20:  # 至少需要20个有效点
                return 0.0
            
            # 计算边缘强度
            edge_strengths = []
            for x, y in edge_points:
                # 计算该点的梯度强度
                if 1 <= x < gray.shape[1]-1 and 1 <= y < gray.shape[0]-1:
                    gx = int(gray[y, x+1]) - int(gray[y, x-1])
                    gy = int(gray[y+1, x]) - int(gray[y-1, x])
                    gradient_magnitude = np.sqrt(gx*gx + gy*gy)
                    edge_strengths.append(gradient_magnitude)
            
            if not edge_strengths:
                return 0.0
            
            # 计算边缘连续性分数
            mean_strength = np.mean(edge_strengths)
            std_strength = np.std(edge_strengths)
            
            # 连续性分数：平均强度高且方差小表示连续性好
            if mean_strength > 10:  # 最小梯度阈值
                continuity_score = min(1.0, mean_strength / 50.0) * (1.0 - min(1.0, std_strength / mean_strength))
            else:
                continuity_score = 0.0
            
            return continuity_score
            
        except Exception as e:
            logger.debug(f"边缘连续性检查异常: {e}")
            return 0.0
    
    def _check_center_gradient(self, gray: np.ndarray, center_x: int, center_y: int, radius: int) -> float:
        """
        检查圆心周围的梯度分布
        
        Args:
            gray: 灰度图像
            center_x: 圆心X坐标
            center_y: 圆心Y坐标
            radius: 半径
        
        Returns:
            梯度分数 (0-1)
        """
        try:
            # 检查圆心周围区域的梯度分布
            check_radius = max(5, int(radius * 0.3))
            
            # 确保检查区域在图像范围内
            x1 = max(0, center_x - check_radius)
            x2 = min(gray.shape[1], center_x + check_radius + 1)
            y1 = max(0, center_y - check_radius)
            y2 = min(gray.shape[0], center_y + check_radius + 1)
            
            if x2 - x1 < 5 or y2 - y1 < 5:
                return 0.0
            
            # 提取圆心周围区域
            center_region = gray[y1:y2, x1:x2]
            
            # 计算梯度
            grad_x = cv2.Sobel(center_region, cv2.CV_64F, 1, 0, ksize=3)
            grad_y = cv2.Sobel(center_region, cv2.CV_64F, 0, 1, ksize=3)
            gradient_magnitude = np.sqrt(grad_x**2 + grad_y**2)
            
            # 计算梯度方向的一致性
            gradient_angles = np.arctan2(grad_y, grad_x)
            
            # 对于圆形，梯度应该指向圆心
            expected_angles = []
            actual_angles = []
            
            h, w = center_region.shape
            center_region_x = center_x - x1
            center_region_y = center_y - y1
            
            for y in range(h):
                for x in range(w):
                    if gradient_magnitude[y, x] > 10:  # 只考虑强梯度点
                        # 计算该点到圆心的期望角度
                        dx = center_region_x - x
                        dy = center_region_y - y
                        if dx != 0 or dy != 0:
                            expected_angle = np.arctan2(dy, dx)
                            actual_angle = gradient_angles[y, x]
                            
                            expected_angles.append(expected_angle)
                            actual_angles.append(actual_angle)
            
            if len(expected_angles) < 5:
                return 0.0
            
            # 计算角度差异
            angle_diffs = []
            for exp, act in zip(expected_angles, actual_angles):
                diff = abs(exp - act)
                # 处理角度环绕
                if diff > np.pi:
                    diff = 2*np.pi - diff
                angle_diffs.append(diff)
            
            # 梯度分数：角度差异越小分数越高
            mean_angle_diff = np.mean(angle_diffs)
            gradient_score = max(0.0, 1.0 - mean_angle_diff / (np.pi/2))
            
            return gradient_score
            
        except Exception as e:
            logger.debug(f"梯度检查异常: {e}")
            return 0.0

    
    def pixel_to_robot_coordinates(self, pixel_x: int, pixel_y: int, z_height: float = 0.0) -> Tuple[float, float, float]:
        """
        将像素坐标转换为机器人坐标系坐标
        
        Args:
            pixel_x: 像素X坐标
            pixel_y: 像素Y坐标
            z_height: Z轴高度（毫米）
        
        Returns:
            机器人坐标系下的(x, y, z)坐标
        """
        try:
            if self.transformation_matrix is None:
                logger.warning("变换矩阵未加载，使用默认转换")
                # 简单的比例转换（需要根据实际情况调整）
                robot_x = (pixel_x - 960) * 0.1  # 假设图像中心为(960, 540)
                robot_y = (540 - pixel_y) * 0.1  # Y轴翻转
                robot_z = z_height
                return robot_x, robot_y, robot_z
            
            # 使用标定的变换矩阵进行坐标转换
            # 构建齐次坐标
            pixel_coords = np.array([pixel_x, pixel_y, 1.0, 1.0])
            
            # 应用变换矩阵
            robot_coords = self.transformation_matrix @ pixel_coords
            
            return float(robot_coords[0]), float(robot_coords[1]), float(robot_coords[2] + z_height)
            
        except Exception as e:
            logger.error(f"坐标转换异常: {e}")
            return 0.0, 0.0, z_height
    
    def get_circle_centers_in_robot_coords(self, z_height: float = 0.0) -> List[Tuple[float, float, float]]:
        """
        获取图像中所有圆心在机器人坐标系下的坐标
        
        Args:
            z_height: Z轴高度（毫米）
        
        Returns:
            机器人坐标系下的圆心坐标列表
        """
        # 采集图像
        image = self.capture_image()
        if image is None:
            return []
        
        # 检测圆形
        circles = self.detect_circles(image)
        
        # 转换坐标
        robot_coords = []
        for x, y, r in circles:
            robot_x, robot_y, robot_z = self.pixel_to_robot_coordinates(x, y, z_height)
            robot_coords.append((robot_x, robot_y, robot_z))
            logger.info(f"圆心像素坐标: ({x}, {y}) -> 机器人坐标: ({robot_x:.2f}, {robot_y:.2f}, {robot_z:.2f})")
        
        return robot_coords
    
    def save_image_with_detection(self, save_path: str = "public/images/detection_result.jpg", 
                                 image: np.ndarray = None, circles: List[Tuple[int, int, int]] = None,
                                 min_radius: int = 10, max_radius: int = 100,
                                 dp: float = 1.0, min_dist: int = 50,
                                 param1: int = 50, param2: int = 30) -> bool:
        """
        保存带有检测结果的图像
        
        Args:
            save_path: 保存路径
            image: 输入图像，如果为None则重新采集
            circles: 检测到的圆形列表，如果为None则重新检测
            min_radius: 最小半径（仅在重新检测时使用）
            max_radius: 最大半径（仅在重新检测时使用）
            dp: 累加器分辨率与图像分辨率的反比（仅在重新检测时使用）
            min_dist: 检测到的圆心之间的最小距离（仅在重新检测时使用）
            param1: Canny边缘检测的高阈值（仅在重新检测时使用）
            param2: 累加器阈值（仅在重新检测时使用）
        
        Returns:
            保存是否成功
        """
        try:
            # 如果没有提供图像，则采集图像
            if image is None:
                image = self.capture_image()
                if image is None:
                    return False
            
            # 如果没有提供圆形数据，则检测圆形
            if circles is None:
                circles = self.detect_circles(image, min_radius=min_radius, max_radius=max_radius,
                                            dp=dp, min_dist=min_dist, param1=param1, param2=param2)
            
            # 在图像上绘制检测结果
            result_image = image.copy()
            for x, y, r in circles:
                # 绘制圆形
                cv2.circle(result_image, (x, y), r, (0, 255, 0), 2)
                # 绘制圆心
                cv2.circle(result_image, (x, y), 2, (0, 0, 255), 3)
                # 添加坐标标注
                robot_x, robot_y, robot_z = self.pixel_to_robot_coordinates(x, y)
                cv2.putText(result_image, f"({robot_x:.1f},{robot_y:.1f})", 
                           (x-50, y-r-10), cv2.FONT_HERSHEY_SIMPLEX, 0.5, (255, 0, 0), 1)
            
            # 确保目录存在
            dir_path = os.path.dirname(save_path)
            if not os.path.exists(dir_path):
                os.makedirs(dir_path, exist_ok=True)
                logger.info(f"创建目录: {dir_path}")
            
            # 检查文件扩展名
            if not save_path.lower().endswith(('.jpg', '.jpeg', '.png', '.bmp', '.tiff')):
                save_path += '.jpg'
                logger.warning(f"文件扩展名不正确，自动添加.jpg扩展名: {save_path}")
            
            # 保存图像并检查结果
            success = cv2.imwrite(save_path, result_image)
            if not success:
                logger.error(f"cv2.imwrite保存失败，可能是路径无效或图像数据有问题: {save_path}")
                return False
                
            logger.info(f"检测结果图像已保存至: {save_path}")
            return True
            
        except Exception as e:
            logger.error(f"保存检测结果图像失败: {e}")
            return False
    
    def release(self):
        """
        释放相机资源
        """
        try:
            self._disconnect_camera()
            logger.info("海康相机资源已释放")
        except Exception as e:
            logger.error(f"释放相机资源异常: {e}")
    
    def reconnect(self) -> bool:
        """
        手动重连相机
        
        Returns:
            True: 重连成功，False: 重连失败
        """
        logger.info("手动重连相机")
        self.reconnect_attempts = 0  # 重置重连计数
        return self._auto_reconnect()
    
    def get_connection_status(self) -> dict:
        """
        获取相机连接状态信息
        
        Returns:
            包含连接状态信息的字典
        """
        return {
            "is_connected": self.is_connected,
            "reconnect_attempts": self.reconnect_attempts,
            "max_reconnect_attempts": self.max_reconnect_attempts,
            "camera_index": self.camera_index
        }
    
    def __del__(self):
        """
        析构函数，自动释放资源
        """
        self.release()


# 全局相机实例
camera_instance = None

def get_camera() -> HikCamera:
    """
    获取相机实例（单例模式）
    
    Returns:
        相机实例
    """
    global camera_instance
    if camera_instance is None:
        camera_instance = HikCamera()
    # 移除自动重连逻辑，避免频繁的连接检查
    # 如果需要重连，应该通过手动调用 camera_reconnect 操作来实现
    return camera_instance

def release_camera():
    """
    释放相机实例
    """
    global camera_instance
    if camera_instance is not None:
        camera_instance.release()
        camera_instance = None