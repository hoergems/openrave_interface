import sys
import time
print sys.path.append("/usr/local/lib")
from libopenrave_interface import Environment, v_string

env = Environment()
env.setupEnvironment("env_3dof.xml")
sensors = v_string()
sensors[:] = ["sensor_BaseLaser2D.xml", 
              "sensor_BaseSpinningLaser2D.xml",
              "sensor_BaseCamera.xml"]
env.loadSensors(sensors)
env.showViewer()
env.getSensorManager()
env.loadRobotFromURDF("test_3dof.urdf")
env.getRobot()
time.sleep(10)

