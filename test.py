import sys
import time
print sys.path.append("/usr/local/lib")
from libopenrave_interface import Environment, v_string

env = Environment()
env.setupEnvironment("env_3dof.xml")
sensors = v_string()
sensors[:] = ["sensor_BaseLaser2D.xml"]
env.loadSensors(sensors)
env.showViewer()
time.sleep(10)

