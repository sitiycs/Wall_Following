import time
from coppeliasim_zmqremoteapi_client import RemoteAPIClient

client = RemoteAPIClient(port=23000)
sim = client.require('sim')

left_motor = sim.getObject('/PioneerP3DX/leftMotor')
right_motor = sim.getObject('/PioneerP3DX/rightMotor')

sensor = sim.getObject('/PioneerP3DX/ultrasonicSensor[0]/proximitySensor')
client.setStepping(False) 
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
    ERROR_THRESHOLD = 0.1
    Kp = 10 # PID控制器的比例增益
    for i in range(iter_end):
        state, point, _, _, _ = sim.readProximitySensor(sensor)

        print(f"距离传感器读数: {point}")

        error = TARGET_DIST - point
        if(state == 0): error = -0.1
        turn = Kp * error

        left_speed = speed - turn
        right_speed = speed + turn

        sim.setJointTargetVelocity(left_motor, left_speed) 
        sim.setJointTargetVelocity(right_motor, right_speed) 
        client.step() 
    
finally:
    print("停止仿真...")
    time.sleep(0.5) 
    sim.stopSimulation()