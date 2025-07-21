import cv2
import threading
import numpy as np
# import pyrealsense2 as rs
import os
from camera.hikcamera import HikCamera
import time
from dobot.arm import arm
import math

def main():
    camera = HikCamera()
    arm_controller = arm()
    # arm_controller.dashboard.DisableRobot()
    # time.sleep(3.0)
    arm_controller.dashboard.EnableRobot()
    camera.start()
    time.sleep(3.0)
    arm_controller.dashboard.StartDrag()
    # 创建保存图片的文件夹
    save_dir = "calibration_image"
    if not os.path.exists(save_dir):
        os.makedirs(save_dir)
    
    # 清空之前的位姿数据文件
    poses_file = 'params/poses.txt'
    with open(poses_file, 'w') as f:
        pass  # 创建空文件或清空现有文件
    print("已清空位姿数据文件，开始新的数据采集")
    
    num = 0
    while True:
        frame = camera.get_latest_frame()
        if frame is not None:
            screen_width = 1920
            screen_height = 1080

            # 获取图像的尺寸
            img_height, img_width = frame.shape[:2]

            # 计算缩放比例
            scale_width = screen_width / img_width
            scale_height = screen_height / img_height
            scale = min(scale_width, scale_height)  # 选择适合的缩放比例

            # 缩放图像
            new_width = int(img_width * scale)
            new_height = int(img_height * scale)
            resized_image = cv2.resize(frame, (new_width, new_height))

            cv2.imshow("Realsense Camera", resized_image)

        key = cv2.waitKey(1)
        if key == ord('q'):
            break
        elif key == ord('s'):
            if frame is not None:
                name = str(num) + ".png"
                save_path = os.path.join(save_dir, name)
                cv2.imwrite(save_path, frame)
                print(f"Image saved to {save_path}!")
                # with open(f'poses.txt', 'a+') as f:
                #     # 将列表中的元素用空格连接成一行
                #     pose = arm_controller.current_actual
                #     # pose_ = [str(i) for i in pose]
                #     new_line = f'{pose}'
                #     # 将新行附加到文件的末尾
                #     f.write(new_line)
                with open('params/poses.txt', 'a') as f:
                    # 获取机械臂当前的实际位姿
                    pose = arm_controller.current_actual
                    converted_pose = [
                        pose[0] / 1000.0,  # x 从毫米到米
                        pose[1] / 1000.0,  # y 从毫米到米
                        pose[2] / 1000.0,  # z 从毫米到米
                        math.radians(pose[3]),  # roll 从角度到弧度
                        math.radians(pose[4]),  # pitch 从角度到弧度
                        math.radians(pose[5])   # yaw 从角度到弧度
                    ]
                    # 将列表转换为以逗号分隔的字符串
                    formatted_pose = ",".join(f"{x:.6f}" for x in converted_pose)
                    # 写入文件并换行
                    f.write(f"{formatted_pose}\n")
                num+=1

    camera.stop()
    cv2.destroyAllWindows()

if __name__ == "__main__":
    main()
