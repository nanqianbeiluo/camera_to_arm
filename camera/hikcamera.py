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
from ctypes import *
import threading
import sys
import platform

# 根据操作系统选择合适的C库
if platform.system() == "Windows":
    # Windows系统使用msvcrt
    try:
        libc = ctypes.CDLL("msvcrt.dll")
    except OSError:
        # 如果msvcrt不可用，使用ctypes的内置函数
        libc = None
else:
    # Linux/Unix系统使用libc.so.6
    try:
        libc = ctypes.CDLL("libc.so.6")
    except OSError:
        libc = None
class HikCamera:
    """
    海康工业相机控制类
    用于图像采集、目标识别和坐标转换
    """
    
    def __init__(self):
        """
        初始化 HikCameraController 类。
        创建并配置摄像头实例，启动读取帧的线程。
        """

        self.latest_frame = None
        self.lock = threading.Lock()
        self.running = False
        self.thread = None


    def __del__(self):
        self.stop()

    def _configure_camera(self):
        """
        配置摄像头并创建句柄。
        """
        # ch:初始化SDK | en: initialize SDK
        MvCamera.MV_CC_Initialize()

        SDKVersion = MvCamera.MV_CC_GetSDKVersion()
        print ("SDKVersion[0x%x]" % SDKVersion)

        deviceList = MV_CC_DEVICE_INFO_LIST()
        ret = MvCamera.MV_CC_EnumDevices(MV_USB_DEVICE, deviceList)
        if ret != 0:
            print ("enum devices fail! ret[0x%x]" % ret)
            sys.exit()

        if deviceList.nDeviceNum == 0:
            print ("find no device!")
            sys.exit()

        print ("Find %d devices!" % deviceList.nDeviceNum)

        # 创建相机实例
        self._camera = MvCamera()

        # 选择设备并创建句柄
        stDeviceList = cast(deviceList.pDeviceInfo[0], POINTER(MV_CC_DEVICE_INFO)).contents

        ret = self._camera.MV_CC_CreateHandle(stDeviceList)
        if ret != 0:
            print ("create handle fail! ret[0x%x]" % ret)
            sys.exit()

        # 打开设备
        ret = self._camera.MV_CC_OpenDevice(MV_ACCESS_Exclusive, 0)
        if ret != 0:
            print ("open device fail! ret[0x%x]" % ret)
            sys.exit()

        stBool = c_bool(False)
        ret =self._camera.MV_CC_GetBoolValue("AcquisitionFrameRateEnable", stBool)
        if ret != 0:
            print ("get AcquisitionFrameRateEnable fail! ret[0x%x]" % ret)

        # 设置触发模式为off
        ret = self._camera.MV_CC_SetEnumValue("TriggerMode", MV_TRIGGER_MODE_OFF)
        if ret != 0:
            print ("set trigger mode fail! ret[0x%x]" % ret)
            sys.exit()

        # 开始取流
        ret = self._camera.MV_CC_StartGrabbing()
        if ret != 0:
            print ("start grabbing fail! ret[0x%x]" % ret)
            sys.exit()
        
    def _release_camera(self):
        """释放相机
        """

        # 停止取流
        ret = self._camera.MV_CC_StopGrabbing()
        if ret != 0:
            logger.info("stop grabbing fail! ret[0x%x]" % ret)
            sys.exit()

        # 关闭设备
        ret = self._camera.MV_CC_CloseDevice()
        if ret != 0:
            logger.info("close deivce fail! ret[0x%x]" % ret)
            sys.exit()

        # 销毁句柄
        ret = self._camera.MV_CC_DestroyHandle()
        if ret != 0:
            logger.info("destroy handle fail! ret[0x%x]" % ret)

    def _work_thread(self):
        """
        摄像头的工作线程，不断读取帧。
        """
        
        self._configure_camera()

        stOutFrame = MV_FRAME_OUT()  
        memset(byref(stOutFrame), 0, sizeof(stOutFrame))
        while self.running:
            ret = self._camera.MV_CC_GetImageBuffer(stOutFrame, 1000)
            # 转换为OpenCV格式图像
            # print(ret)
            # print(stOutFrame.pBufAddr)
            if None != stOutFrame.pBufAddr and 0 == ret:
                img_buff = (c_ubyte * stOutFrame.stFrameInfo.nFrameLen)()
                
                # 使用跨平台的内存复制方法
                if libc is not None:
                    libc.memcpy(byref(img_buff), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
                else:
                    # 如果libc不可用，使用ctypes的memmove作为替代
                    ctypes.memmove(byref(img_buff), stOutFrame.pBufAddr, stOutFrame.stFrameInfo.nFrameLen)
                
                img_buff = np.frombuffer(img_buff, count=int(stOutFrame.stFrameInfo.nFrameLen), dtype=np.uint8)
                img_buff = img_buff.reshape((stOutFrame.stFrameInfo.nHeight, stOutFrame.stFrameInfo.nWidth))

                with self.lock:
                    self.latest_frame = img_buff

                self._camera.MV_CC_FreeImageBuffer(stOutFrame)

        self._release_camera()

    def start(self):
        """
        启动摄像头读取线程。
        """
        self.running = True
        self.thread = threading.Thread(target=self._work_thread)
        self.thread.daemon = True
        self.thread.start()

    def stop(self):
        """
        停止摄像头读取线程。
        """
        self.running = False
        if self.thread is not None:
            self.thread.join()

    def get_latest_frame(self):
        """
        获取最新的帧。
        使用锁来确保线程安全。
        """

        with self.lock:
            return self.latest_frame


