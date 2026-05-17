import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(port=23000)
sim = client.require('sim')

left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')
sensor0 = sim.getObject('/PioneerP3DX/ultrasonicSensor[0]/proximitySensor')
sensor1 = sim.getObject('/PioneerP3DX/ultrasonicSensor[1]/proximitySensor')

client.setStepping(True) 
sim.startSimulation()

def left_rotate(speed, dis_threshold):
    sim.setJointTargetVelocity(left_motor, -speed) 
    sim.setJointTargetVelocity(right_motor, speed)
    for i in range(500):
        state0, point0, _, _, _ = sim.readProximitySensor(sensor0)
        state1, point1, _, _, _ = sim.readProximitySensor(sensor1)
        if((state0 == 0 or point0 >= dis_threshold) and (state1 == 0 or point1 >= dis_threshold)):
            break
        client.step()
    sim.setJointTargetVelocity(left_motor, speed) 
    sim.setJointTargetVelocity(right_motor, speed)

try:
    print("开始移动...")
    speed = 1.1
    
    print("设置目标速度...")

    sim.setJointTargetVelocity(left_motor, speed)
    sim.setJointTargetVelocity(right_motor, speed)
    
    print("成功设置目标速度，开始仿真步进...")  

    iter_end=50000
    dis_threshold=0.4

    for i in range(iter_end):
        state0, point0, _, _, _ = sim.readProximitySensor(sensor0)
        print(f"距离传感器0读数: {point0}")
        state1, point1, _, _, _ = sim.readProximitySensor(sensor1)
        print(f"距离传感器1读数: {point1}")

        if((state0 == 1 and point0 < dis_threshold) or (state1 == 1 and point1 < dis_threshold)):
            left_rotate(speed, dis_threshold)
        client.step()
    
finally:
    print("停止仿真...")
    time.sleep(0.5) 
    sim.stopSimulation()