import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(port=23000)
sim = client.require('sim')

left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')
sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[0]/proximitySensor')

client.setStepping(True) 
sim.startSimulation()

try:
    print("开始移动...")
    speed = 1.1
    
    print("设置目标速度...")

    sim.setJointTargetVelocity(left_motor, speed)
    sim.setJointTargetVelocity(right_motor, speed)
    
    print("成功设置目标速度，开始仿真步进...")  

    for i in range(500):
        client.step()
        actual_vel = sim.getJointVelocity(left_motor)
        state, point, _, _, _ = sim.readProximitySensor(sensor)
        print(f"距离传感器读数: {point}")
        print(f"目标速度: {speed}, 实际速度反馈: {actual_vel}")
        if i % 50 == 0:
            print(f"当前仿真位置(时间): {sim.getSimulationTime():.2f}")
        
    print("开始大幅度转弯...")
    sim.setJointTargetVelocity(left_motor, -speed) 
    sim.setJointTargetVelocity(right_motor, speed)
    
    for i in range(200):
        client.step()

finally:
    print("停止仿真...")
    time.sleep(0.5) 
    sim.stopSimulation()