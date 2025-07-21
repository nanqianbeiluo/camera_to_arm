import cv2
import numpy as np
import os
import glob
import json
from typing import List, Tuple, Optional

def calibrate_camera(images_dir, chessboard_size, square_size):
    """
    标定相机，计算相机内参和畸变系数。

    参数:
    images_dir (str): 棋盘格图片的存储目录。
    chessboard_size (tuple): 棋盘格内角点数量 (宽度, 高度)。
    square_size (float): 每个棋盘格的边长，以米为单位。

    返回:
    tuple: 包含相机内参矩阵和畸变系数的元组。
    """
    chessboard_width, chessboard_height = chessboard_size
    # 3D点在真实世界坐标系
    objp = np.zeros((chessboard_height * chessboard_width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_width, 0:chessboard_height].T.reshape(-1, 2)
    objp *= square_size

    objpoints = []  # 3D点在真实世界坐标系中的坐标
    imgpoints = []  # 2D点在图像坐标系中的坐标

    images = glob.glob(os.path.join(images_dir, '*.png'))  # 获取所有PNG格式的棋盘格图片

    if not images:
        print(f"未找到任何图片在目录: {images_dir}")
        return None, None
    print("next")
    for fname in images:
        img = cv2.imread(fname)
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)

        # 寻找棋盘格的角点
        ret, corners = cv2.findChessboardCorners(gray, (chessboard_width, chessboard_height), None)

        # 如果找到了足够的角点，添加对象点和图像点
        if ret:
            objpoints.append(objp)
            imgpoints.append(corners)
            print(f"找到角点: {fname}")

            # 显示角点
            cv2.drawChessboardCorners(img, (chessboard_width, chessboard_height), corners, ret)
            cv2.imshow('Corners', img)
            cv2.waitKey(500)
        else:
            print(f"no ret: {fname}")

    cv2.destroyAllWindows()

    if not objpoints or not imgpoints:
        print("未找到足够的角点进行标定")
        return None, None

    # 标定相机
    ret, mtx, dist, rvecs, tvecs = cv2.calibrateCamera(objpoints, imgpoints, gray.shape[::-1], None, None)

    if ret:
        print("相机内参矩阵:\n", mtx)
        print("畸变系数:\n", dist)
        return mtx, dist
    else:
        print("相机标定失败")
        return None, None

def save_calibration_results(filename, camera_matrix, distortion_coefficients):
    with open(filename, 'w', encoding='utf-8') as f:
        f.write("相机内参矩阵:\n")
        np.savetxt(f, camera_matrix, fmt='%f')
        f.write("\n畸变系数:\n")
        np.savetxt(f, distortion_coefficients, fmt='%f')

def load_robot_poses(poses_file: str = "params/poses.txt") -> List[Tuple[float, float, float, float, float, float]]:
    """
    从params/poses.txt文件中加载机械臂位姿数据
    
    参数:
    poses_file: 位姿文件路径
    
    返回:
    位姿列表，每个元素为(x, y, z, rx, ry, rz)
    """
    poses = []
    try:
        with open(poses_file, 'r', encoding='utf-8') as f:
            lines = f.readlines()
        
        for line in lines:
            line = line.strip()
            if line and not line.startswith('#'):
                # 支持逗号分隔和空格分隔两种格式
                if ',' in line:
                    values = [float(x.strip()) for x in line.split(',')]
                else:
                    values = [float(x) for x in line.split()]
                
                if len(values) == 6:
                    poses.append(tuple(values))
                else:
                    print(f"警告：跳过格式不正确的行: {line}")
        
        print(f"成功加载 {len(poses)} 组机械臂位姿数据")
        return poses
    
    except Exception as e:
        print(f"加载位姿数据失败: {e}")
        return []

def detect_chessboard_poses(images_dir: str, chessboard_size: Tuple[int, int], 
                           square_size: float, camera_matrix: np.ndarray, 
                           dist_coeffs: np.ndarray) -> List[Tuple[np.ndarray, np.ndarray]]:
    """
    检测标定板在相机坐标系中的位姿
    
    参数:
    images_dir: 图片目录
    chessboard_size: 棋盘格尺寸
    square_size: 棋盘格边长
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    
    返回:
    位姿列表，每个元素为(旋转向量, 平移向量)
    """
    chessboard_width, chessboard_height = chessboard_size
    
    # 准备棋盘格的3D点
    objp = np.zeros((chessboard_height * chessboard_width, 3), np.float32)
    objp[:, :2] = np.mgrid[0:chessboard_width, 0:chessboard_height].T.reshape(-1, 2)
    objp *= square_size
    
    # 获取图片文件列表
    images = glob.glob(os.path.join(images_dir, '*.png'))
    images.sort()  # 确保顺序一致
    
    board_poses = []
    
    for fname in images:
        img = cv2.imread(fname)
        if img is None:
            continue
            
        gray = cv2.cvtColor(img, cv2.COLOR_BGR2GRAY)
        
        # 寻找棋盘格角点
        ret, corners = cv2.findChessboardCorners(gray, (chessboard_width, chessboard_height), None)
        
        if ret:
            # 精确化角点位置
            criteria = (cv2.TERM_CRITERIA_EPS + cv2.TERM_CRITERIA_MAX_ITER, 30, 0.001)
            corners2 = cv2.cornerSubPix(gray, corners, (11, 11), (-1, -1), criteria)
            
            # 求解PnP问题，获取标定板位姿
            success, rvec, tvec = cv2.solvePnP(objp, corners2, camera_matrix, dist_coeffs)
            
            if success:
                board_poses.append((rvec, tvec))
                print(f"检测到标定板位姿: {os.path.basename(fname)}")
            else:
                print(f"PnP求解失败: {os.path.basename(fname)}")
        else:
            print(f"未找到棋盘格角点: {os.path.basename(fname)}")
    
    print(f"成功检测到 {len(board_poses)} 个标定板位姿")
    return board_poses

def pose_to_transformation_matrix(pose: Tuple[float, float, float, float, float, float]) -> np.ndarray:
    """
    将6D位姿转换为4x4变换矩阵
    
    参数:
    pose: (x, y, z, rx, ry, rz) - 位置(米)和旋转角度(弧度)
    
    返回:
    4x4变换矩阵
    """
    x, y, z, rx, ry, rz = pose
    
    # 创建旋转矩阵（使用欧拉角ZYX顺序）
    R_x = np.array([[1, 0, 0],
                    [0, np.cos(rx), -np.sin(rx)],
                    [0, np.sin(rx), np.cos(rx)]])
    
    R_y = np.array([[np.cos(ry), 0, np.sin(ry)],
                    [0, 1, 0],
                    [-np.sin(ry), 0, np.cos(ry)]])
    
    R_z = np.array([[np.cos(rz), -np.sin(rz), 0],
                    [np.sin(rz), np.cos(rz), 0],
                    [0, 0, 1]])
    
    # 组合旋转矩阵 (ZYX顺序)
    R = R_z @ R_y @ R_x
    
    # 创建4x4变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = [x, y, z]
    
    return T

def rvec_tvec_to_transformation_matrix(rvec: np.ndarray, tvec: np.ndarray) -> np.ndarray:
    """
    将旋转向量和平移向量转换为4x4变换矩阵
    
    参数:
    rvec: 旋转向量
    tvec: 平移向量
    
    返回:
    4x4变换矩阵
    """
    # 将旋转向量转换为旋转矩阵
    R, _ = cv2.Rodrigues(rvec)
    
    # 创建4x4变换矩阵
    T = np.eye(4)
    T[:3, :3] = R
    T[:3, 3] = tvec.flatten()
    
    return T

def perform_hand_eye_calibration(images_dir: str, chessboard_size: Tuple[int, int],
                                square_size: float, camera_matrix: np.ndarray,
                                dist_coeffs: np.ndarray) -> Optional[np.ndarray]:
    """
    执行手眼标定
    
    参数:
    images_dir: 图片目录
    chessboard_size: 棋盘格尺寸
    square_size: 棋盘格边长
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    
    返回:
    手眼变换矩阵，失败返回None
    """
    # 加载机械臂位姿数据
    robot_poses = load_robot_poses()
    if not robot_poses:
        print("无法加载机械臂位姿数据")
        return None
    
    # 检测标定板位姿
    board_poses = detect_chessboard_poses(images_dir, chessboard_size, square_size, 
                                         camera_matrix, dist_coeffs)
    if not board_poses:
        print("无法检测到标定板位姿")
        return None
    
    # 确保数据数量匹配
    min_poses = min(len(robot_poses), len(board_poses))
    if min_poses < 3:
        print(f"标定数据不足！需要至少3组数据，当前只有{min_poses}组")
        return None
    
    print(f"使用 {min_poses} 组数据进行手眼标定")
    
    # 准备手眼标定数据
    R_gripper2base = []
    t_gripper2base = []
    R_target2cam = []
    t_target2cam = []
    
    # 计算相对变换
    for i in range(min_poses - 1):
        # 机械臂末端执行器的相对变换
        T1_robot = pose_to_transformation_matrix(robot_poses[i])
        T2_robot = pose_to_transformation_matrix(robot_poses[i + 1])
        T_robot_rel = np.linalg.inv(T1_robot) @ T2_robot
        
        R_gripper2base.append(T_robot_rel[:3, :3])
        t_gripper2base.append(T_robot_rel[:3, 3])
        
        # 标定板在相机中的相对变换
        T1_board = rvec_tvec_to_transformation_matrix(board_poses[i][0], board_poses[i][1])
        T2_board = rvec_tvec_to_transformation_matrix(board_poses[i + 1][0], board_poses[i + 1][1])
        T_board_rel = np.linalg.inv(T1_board) @ T2_board
        
        R_target2cam.append(T_board_rel[:3, :3])
        t_target2cam.append(T_board_rel[:3, 3])
    
    try:
        # 使用OpenCV的手眼标定函数
        R_cam2gripper, t_cam2gripper = cv2.calibrateHandEye(
            R_gripper2base, t_gripper2base,
            R_target2cam, t_target2cam,
            method=cv2.CALIB_HAND_EYE_TSAI
        )
        
        # 构建完整的手眼变换矩阵
        hand_eye_matrix = np.eye(4)
        hand_eye_matrix[:3, :3] = R_cam2gripper
        hand_eye_matrix[:3, 3] = t_cam2gripper.flatten()
        
        print("手眼标定成功！")
        print("手眼变换矩阵:")
        print(hand_eye_matrix)
        
        # 评估标定精度
        evaluate_hand_eye_calibration(robot_poses[:min_poses], board_poses[:min_poses], 
                                    hand_eye_matrix, camera_matrix, dist_coeffs)
        
        return hand_eye_matrix
        
    except Exception as e:
        print(f"手眼标定失败: {e}")
        return None

def evaluate_hand_eye_calibration(robot_poses: List[Tuple], board_poses: List[Tuple],
                                 hand_eye_matrix: np.ndarray, camera_matrix: np.ndarray,
                                 dist_coeffs: np.ndarray):
    """
    评估手眼标定精度
    
    参数:
    robot_poses: 机械臂位姿列表
    board_poses: 标定板位姿列表
    hand_eye_matrix: 手眼变换矩阵
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    """
    print("\n=== 手眼标定精度评估 ===")
    
    errors = []
    
    for i, (robot_pose, (rvec, tvec)) in enumerate(zip(robot_poses, board_poses)):
        # 将标定板位姿转换到机械臂坐标系
        T_board_cam = rvec_tvec_to_transformation_matrix(rvec, tvec)
        T_robot_base = pose_to_transformation_matrix(robot_pose)
        
        # 通过手眼变换预测标定板在机械臂坐标系中的位置
        T_board_predicted = T_robot_base @ hand_eye_matrix @ T_board_cam
        
        # 计算位置误差（这里简化为只计算平移误差）
        predicted_pos = T_board_predicted[:3, 3]
        
        # 由于我们没有标定板在机械臂坐标系中的真实位置，
        # 这里计算的是相对一致性误差
        if i == 0:
            reference_pos = predicted_pos
            error = 0.0
        else:
            error = np.linalg.norm(predicted_pos - reference_pos)
        
        errors.append(error)
        print(f"第{i+1}组数据预测位置: [{predicted_pos[0]:.4f}, {predicted_pos[1]:.4f}, {predicted_pos[2]:.4f}], 误差: {error*1000:.2f}mm")
    
    if len(errors) > 1:
        mean_error = np.mean(errors[1:])  # 排除第一个参考点
        max_error = np.max(errors[1:])
        
        print(f"\n平均位置误差: {mean_error*1000:.2f} mm")
        print(f"最大位置误差: {max_error*1000:.2f} mm")
        
        if mean_error < 0.005:  # 5mm
            print("标定精度: 优秀")
        elif mean_error < 0.010:  # 10mm
            print("标定精度: 良好")
        else:
            print("标定精度: 需要改进，建议重新采集数据")

def save_hand_eye_calibration(hand_eye_matrix: np.ndarray, camera_matrix: np.ndarray, 
                             dist_coeffs: np.ndarray, filename: str = "params/hand_eye_calibration.json"):
    """
    保存手眼标定结果到JSON文件
    
    参数:
    hand_eye_matrix: 手眼变换矩阵
    camera_matrix: 相机内参矩阵
    dist_coeffs: 畸变系数
    filename: 保存文件名
    """
    calibration_data = {
        "hand_eye_matrix": hand_eye_matrix.tolist(),
        "camera_matrix": camera_matrix.tolist(),
        "distortion_coefficients": dist_coeffs.tolist(),
        "description": "手眼标定结果 - 相机到机械臂基座的变换矩阵",
        "units": {
            "translation": "meters",
            "rotation": "rotation_matrix"
        }
    }
    
    try:
        with open(filename, 'w', encoding='utf-8') as f:
            json.dump(calibration_data, f, indent=4, ensure_ascii=False)
        
        print(f"\n手眼标定结果已保存到: {filename}")
        
        # 同时保存为文本格式便于查看
        txt_filename = filename.replace('.json', '.txt')
        with open(txt_filename, 'w', encoding='utf-8') as f:
            f.write("手眼标定结果\n")
            f.write("=" * 50 + "\n")
            f.write("手眼变换矩阵 (相机到机械臂基座):\n")
            for row in hand_eye_matrix:
                f.write(' '.join([f'{val:10.6f}' for val in row]) + '\n')
            
            f.write("\n相机内参矩阵:\n")
            for row in camera_matrix:
                f.write(' '.join([f'{val:10.6f}' for val in row]) + '\n')
            
            f.write("\n畸变系数:\n")
            f.write(' '.join([f'{val:10.6f}' for val in dist_coeffs.flatten()]) + '\n')
        
        print(f"文本格式结果已保存到: {txt_filename}")
        
    except Exception as e:
        print(f"保存手眼标定结果失败: {e}")

def load_hand_eye_calibration(filename: str = "params/hand_eye_calibration.json") -> Tuple[Optional[np.ndarray], Optional[np.ndarray], Optional[np.ndarray]]:
    """
    加载手眼标定结果
    
    参数:
    filename: 标定文件名
    
    返回:
    (手眼变换矩阵, 相机内参矩阵, 畸变系数)
    """
    try:
        with open(filename, 'r', encoding='utf-8') as f:
            data = json.load(f)
        
        hand_eye_matrix = np.array(data['hand_eye_matrix'])
        camera_matrix = np.array(data['camera_matrix'])
        dist_coeffs = np.array(data['distortion_coefficients'])
        
        print(f"成功加载手眼标定结果: {filename}")
        return hand_eye_matrix, camera_matrix, dist_coeffs
        
    except Exception as e:
        print(f"加载手眼标定结果失败: {e}")
        return None, None, None

if __name__ == "__main__":
    images_directory = "calibration_image"  # 棋盘格图片存放的目录
    chessboard_dimensions = (11, 8)  # 棋盘格的尺寸 (宽度, 高度)，通常内角点数量为 (列数-1, 行数-1)
    square_size = 0.022  # 每个棋盘格的边长，以米为单位

    camera_matrix, distortion_coefficients = calibrate_camera(images_directory, chessboard_dimensions, square_size)

    if camera_matrix is not None and distortion_coefficients is not None:
        # 保存内参和畸变系数到文件
        # np.savez("camera_calibration.npz", camera_matrix=camera_matrix, distortion_coefficients=distortion_coefficients)
        # print("相机标定结果已保存到 'camera_calibration.npz'")
        save_calibration_results("params/camera_calibration.txt", camera_matrix, distortion_coefficients)
        print("标定结果已保存到 params/camera_calibration.txt")
        
        # 如果存在位姿数据，进行手眼标定
        if os.path.exists("params/poses.txt"):
            print("\n检测到位姿数据，开始手眼标定...")
            hand_eye_matrix = perform_hand_eye_calibration(
                images_directory, chessboard_dimensions, square_size, 
                camera_matrix, distortion_coefficients
            )
            if hand_eye_matrix is not None:
                save_hand_eye_calibration(hand_eye_matrix, camera_matrix, distortion_coefficients)
                print("手眼标定完成！")
            else:
                print("手眼标定失败！")
        else:
            print("\n未找到位姿数据文件 params/poses.txt，跳过手眼标定")
            print("请先运行 save_image.py 收集标定数据")
