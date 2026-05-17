import cv2
import numpy as np
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(port=23000)
sim = client.getObject('sim')

# 1. 找到你的视觉传感器（记得在场景里改名或对齐路径）
vision_sensor = sim.getObject('/visionSensor') 

sim.startSimulation()

try:
    while True:
        # 2. 获取图像（格式是：字节流, 分辨率）
        img, res = sim.getVisionSensorImg(vision_sensor)
        
        # 3. 语义转化：把二进制字节流变成 Python 能看懂的图片矩阵
        image = np.frombuffer(img, dtype=np.uint8).reshape([res[1], res[0], 3])
        image = cv2.flip(cv2.cvtColor(image, cv2.COLOR_RGB2BGR), 0)
        
        # 4. 展示画面
        cv2.imshow('Robot View', image)
        
        # 按 'q' 键退出
        if cv2.waitKey(1) & 0xFF == ord('q'):
            break
finally:
    sim.stopSimulation()
    cv2.destroyAllWindows()