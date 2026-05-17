import time
import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

def set_speed(left, right):
    sim.setJointTargetVelocity(left_motor, left)
    sim.setJointTargetVelocity(right_motor, right)

def get_red_center(mask):
    """从红色掩码中找到红色区域的中心点坐标"""
    # 找到所有白色像素的位置
    points = cv2.findNonZero(mask)
    if points is None:
        return None
    # 计算中心点
    center = np.mean(points, axis=0)[0]
    return center


client = RemoteAPIClient(port=23000)
sim = client.require('sim')

left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')
vision_sensor = sim.getObject('/visionSensor') 

sensor=[]
for i in range(2):
    sensor_ = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]/proximitySensor')
    sensor.append(sensor_)
client.setStepping(False) 
sim.startSimulation()

try:
    print("开始移动...")
    speed = 2    
    print("设置目标速度...")
    set_speed(speed, speed)
    print("成功设置目标速度，开始仿真步进...")  

    iter_end=50000
    TARGET_DIST = 0.25
    SAFE_DIST = 1        # 正向安全距离
    Kp_right = 20 # PID控制器的比例增益
    Kp_front = 5         # 正向P增益

    while True:
        # 2. 获取图像（格式是：字节流, 分辨率）
        img, res = sim.getVisionSensorImg(vision_sensor)
        
        # 3. 语义转化：把二进制字节流变成 Python 能看懂的图片矩阵
        image = np.frombuffer(img, dtype=np.uint8).reshape([res[1], res[0], 3])
        image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 0)
        
        # 在显示之前，加这段

        # 转换到 HSV 颜色空间
        hsv = cv2.cvtColor(image, cv2.COLOR_BGR2HSV)

        # 红色的范围（HSV）
        # 红色在色环上跨越了0和180两个区间，需要两个范围
        lower_red1 = np.array([0, 100, 100])
        upper_red1 = np.array([10, 255, 255])
        lower_red2 = np.array([160, 100, 100])
        upper_red2 = np.array([180, 255, 255])

        # 生成红色掩码（红色区域为白色，其他为黑色）
        mask1 = cv2.inRange(hsv, lower_red1, upper_red1)
        mask2 = cv2.inRange(hsv, lower_red2, upper_red2)
        mask = cv2.bitwise_or(mask1, mask2)

        # 在原图上标记红色区域（把红色区域变成绿色，便于观察）
        image_with_mask = image.copy()
        image_with_mask[mask > 0] = [0, 255, 0]  # 绿色标记

        # 4. 展示画面
        # 显示原图和掩码
        cv2.imshow('Original', image)
        cv2.imshow('Red Mask', mask)
        cv2.imshow('Detected Red (Green)', image_with_mask)
        
        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
        center = get_red_center(mask)

        if center is not None:
            # 获取图像宽度
            h, w = mask.shape
            center_x = center[0]
            
            # 根据红色位置控制小车
            porpotional_gain = 0.45
            if center_x < w * porpotional_gain:   # 红色在左边
                print(f"红色在左边 ({center_x:.0f})，左转")
                set_speed(-porpotional_gain, porpotional_gain)
            elif center_x > w * (1-porpotional_gain): # 红色在右边
                print(f"红色在右边 ({center_x:.0f})，右转")
                set_speed(porpotional_gain, -porpotional_gain)
            else:                     # 红色在中间
                print(f"红色在中间 ({center_x:.0f})，停止")
                set_speed(0, 0)
        else:
            state, point, _, _, _ = sim.readProximitySensor(sensor[0])
            state1, point1, _, _, _ = sim.readProximitySensor(sensor[1])

            print(f"距离传感器0读数: {point}")
            print(f"距离传感器1读数: {point1}")

            error_right = TARGET_DIST - point
            error_front = SAFE_DIST - point1 
            if(state1 == 0): error_front = 0
            if(state == 0): error_right = -0.03

            turn = Kp_right * error_right + Kp_front * error_front

            left_speed = speed - turn
            right_speed = speed + turn
            set_speed(left_speed, right_speed)
        
finally:
    print("停止仿真...")
    time.sleep(0.5) 
    sim.stopSimulation()
    cv2.destroyAllWindows()