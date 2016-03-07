import sys
import time
print sys.path.append("/usr/local/lib")
from libopenrave_interface import Environment, v_string, v_double, v2_double

def test_collision(env):
    robot = env.getRobot()
    collision_manager = env.getCollisionManager()
    joint_angles_v = v_double()
    joint_angles_v_2 = v_double()
    joint_angles_v[:] = [0.2, 0.0, 0.0]
    joint_angles_v_2[:] = [0.2, 0.0, 0.0]
    particle_joint_values = v2_double()
    
    env.updateRobotValues(joint_angles_v, 
                          joint_angles_v,
                          particle_joint_values,
                          particle_joint_values)
    
    robot_collision_objects_start = robot.createRobotCollisionObjects(joint_angles_v)
    robot_collision_objects_goal = robot.createRobotCollisionObjects(joint_angles_v_2)
    for i in xrange(len(robot_collision_objects_start)):
        in_collision = collision_manager.inCollisionContinuousEnvironment([robot_collision_objects_start[i],
                                                                           robot_collision_objects_goal[i]])
        print in_collision
    time.sleep(100)
    

def prog(joint_angles, sensor_name):
    joint_angles_v = v_double()
    joint_velocities = v_double()
    particle_joint_values = v2_double()
    joint_angles_v[:] = [joint_angles[i] for i in xrange(len(joint_angles))]
    joint_velocities[:] = [0.0, 0.0, 0.0]
    env.getRobot().setState(joint_angles_v, joint_velocities)
    env.transformSensorToEndEffector(joint_angles_v, sensor_name)
    env.updateRobotValues(joint_angles_v, 
                          joint_angles_v,
                          particle_joint_values,
                          particle_joint_values)
    time.sleep(1)
    env.getSensorManager().activateSensor(sensor_name)
    time.sleep(1)
    env.drawBoxes()
    time.sleep(1)
    env.getSensorManager().deactivateSensor(sensor_name)
    time.sleep(1.0)

sensor_file = "sensor_BaseFlashLidar3D.xml"
sensor_name = "FlashLidar3D"

env = Environment()
env.setupEnvironment("env_3dof.xml")
sensors = v_string()
sensors[:] = [sensor_file]
env.loadSensors(sensors)
env.showViewer()
env.getSensorManager()
env.loadRobotFromURDF("test_3dof.urdf")
env.initOctree()
<<<<<<< HEAD
time.sleep(10)
print "activate"
env.getSensorManager().activateSensor(sensor_name)
=======

#test_collision(env)
>>>>>>> 69ad5ce209961ef418b014a1250ccf8a24e65368

time.sleep(5)
joint_angles = [0.0, 0.0, 0.0]
prog(joint_angles, sensor_name)
joint_angles[0] = 0.1
prog(joint_angles, sensor_name)
joint_angles[0] = -0.1
prog(joint_angles, sensor_name)
joint_angles[0] = 0.0
joint_angles[1] = 0.1
prog(joint_angles, sensor_name)
joint_angles[0] = 0.3
joint_angles[1] = -0.1
prog(joint_angles, sensor_name)
joint_angles[0] = -0.3
prog(joint_angles, sensor_name)
joint_angles[0] = 0.7
joint_angles[1] = 0.2
joint_angles[2] = -0.2
prog(joint_angles, sensor_name)
time.sleep(20)










       

