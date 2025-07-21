#!/usr/bin/env python3
# -*- coding: utf-8 -*-
"""
日志记录模块

提供统一的日志记录功能，支持控制台输出和文件保存
"""

import logging
import os
from datetime import datetime
from logging.handlers import RotatingFileHandler

class Logger:
    """
    日志记录器类
    """
    
    def __init__(self, name="IQR_Backend", level=logging.INFO):
        """
        初始化日志记录器
        
        Args:
            name: 日志记录器名称
            level: 日志级别
        """
        self.logger = logging.getLogger(name)
        self.logger.setLevel(level)
        
        # 避免重复添加处理器
        if not self.logger.handlers:
            self._setup_handlers()
    
    def _setup_handlers(self):
        """
        设置日志处理器
        """
        # 创建日志目录
        log_dir = "logs"
        if not os.path.exists(log_dir):
            os.makedirs(log_dir)
        
        # 日志格式
        formatter = logging.Formatter(
            '%(asctime)s - %(name)s - %(levelname)s - %(filename)s:%(lineno)d - %(message)s',
            datefmt='%Y-%m-%d %H:%M:%S'
        )
        
        # 控制台处理器
        console_handler = logging.StreamHandler()
        console_handler.setLevel(logging.INFO)
        console_handler.setFormatter(formatter)
        self.logger.addHandler(console_handler)
        
        # 文件处理器 - 所有日志
        log_file = os.path.join(log_dir, f"iqr_backend_{datetime.now().strftime('%Y%m%d')}.log")
        file_handler = RotatingFileHandler(
            log_file, 
            maxBytes=10*1024*1024,  # 10MB
            backupCount=5,
            encoding='utf-8'
        )
        file_handler.setLevel(logging.DEBUG)
        file_handler.setFormatter(formatter)
        self.logger.addHandler(file_handler)
        
        # 错误日志文件处理器
        error_log_file = os.path.join(log_dir, f"iqr_backend_error_{datetime.now().strftime('%Y%m%d')}.log")
        error_handler = RotatingFileHandler(
            error_log_file,
            maxBytes=5*1024*1024,  # 5MB
            backupCount=3,
            encoding='utf-8'
        )
        error_handler.setLevel(logging.ERROR)
        error_handler.setFormatter(formatter)
        self.logger.addHandler(error_handler)
    
    def debug(self, message):
        """调试级别日志"""
        self.logger.debug(message)
    
    def info(self, message):
        """信息级别日志"""
        self.logger.info(message)
    
    def warning(self, message):
        """警告级别日志"""
        self.logger.warning(message)
    
    def error(self, message):
        """错误级别日志"""
        self.logger.error(message)
    
    def critical(self, message):
        """严重错误级别日志"""
        self.logger.critical(message)
    
    def exception(self, message):
        """异常日志（包含堆栈信息）"""
        self.logger.exception(message)

# 创建全局日志实例
logger = Logger()

# 为了兼容性，也提供直接的函数接口
def debug(message):
    """调试级别日志"""
    logger.debug(message)

def info(message):
    """信息级别日志"""
    logger.info(message)

def warning(message):
    """警告级别日志"""
    logger.warning(message)

def error(message):
    """错误级别日志"""
    logger.error(message)

def critical(message):
    """严重错误级别日志"""
    logger.critical(message)

def exception(message):
    """异常日志（包含堆栈信息）"""
    logger.exception(message)

# 设置日志级别的便捷函数
def set_log_level(level):
    """
    设置日志级别
    
    Args:
        level: 日志级别 (DEBUG, INFO, WARNING, ERROR, CRITICAL)
    """
    if isinstance(level, str):
        level = getattr(logging, level.upper())
    logger.logger.setLevel(level)

# 获取特定模块的日志记录器
def get_logger(name):
    """
    获取特定名称的日志记录器
    
    Args:
        name: 日志记录器名称
    
    Returns:
        Logger实例
    """
    return Logger(name)