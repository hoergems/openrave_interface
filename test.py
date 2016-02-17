import sys
import time
print sys.path.append("/usr/local/lib")
from libopenrave_interface import Environment, v_string, v_double

env = Environment()
env.setupEnvironment("env_3dof.xml")
sensors = v_string()
sensors[:] = ["sensor_BaseLaser2D.xml"]
env.loadSensors(sensors)
env.showViewer()
env.getSensorManager()
env.loadRobotFromURDF("test_3dof.urdf")

joint_angles = v_double()
joint_angles[:] = [0.0, 0.0, 0.0]
env.transformSensorToEndEffector(joint_angles, "2DLaser")

while True:
    time.sleep(10)

