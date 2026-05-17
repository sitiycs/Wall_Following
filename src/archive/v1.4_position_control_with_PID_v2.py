import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(port=23000)
sim = client.require('sim')

left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sensor=[]
for i in range(2):
    sensor_ = sim.getObject(f'/PioneerP3DX/ultrasonicSensor[{i}]/proximitySensor')
    sensor.append(sensor_)
client.setStepping(True) 
sim.startSimulation()

try:
    print("开始移动...")
    speed = 2
    
    print("设置目标速度...")

    sim.setJointTargetVelocity(left_motor, speed)
    sim.setJointTargetVelocity(right_motor, speed)
    
    print("成功设置目标速度，开始仿真步进...")  

    iter_end=50000
    TARGET_DIST = 0.25
    SAFE_DIST = 1        # 正向安全距离
    Kp_right = 20 # PID控制器的比例增益
    Kp_front = 5         # 正向P增益

    for i in range(iter_end):
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

        sim.setJointTargetVelocity(left_motor, left_speed) 
        sim.setJointTargetVelocity(right_motor, right_speed) 
        client.step() 
    
finally:
    print("停止仿真...")
    time.sleep(0.5) 
    sim.stopSimulation()