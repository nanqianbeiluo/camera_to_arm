import math

def image_to_world(u, v):
    """将图像坐标转换为世界坐标"""
    # 标定矩阵参数
    h11, h12, h13 = 0.10313439, 0.0017332305, -111.84082
    h21, h22, h23 = 0.002168329, -0.1026563, 513.4729
    
    # 计算特征点世界坐标
    feature_x = h11 * u + h12 * v + h13
    feature_y = h21 * u + h22 * v + h23
    
    # 应用校准后的偏移量
    obj_x = feature_x + 50.9267
    obj_y = feature_y - 606.9468
    
    return obj_x, obj_y

def main():
    x = 1403.578
    y = 1026.111
    point = image_to_world(x, y)
    print(point)

if __name__ == "__main__":
    main()

