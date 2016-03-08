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
    
def propagate(env, sensor_name):
    current_state = v_double()
    control_input = v_double()
    control_error = v_double()
    simulation_step_size = 0.001
    duration = 0.03
    result = v_double()
    
    robot_dof_values = v_double()
    env.getRobotDOFValues(robot_dof_values)
    print "len robot_dof_values " + str(len(robot_dof_values))
    
    #cs = [0.0 for i in xrange(len(robot_dof_values) * 2)]
    cv = [0.0 for i in xrange(len(robot_dof_values))]
    cv[0] = 0.3
    cv[1] = -0.3
    cv[4] = -0.3
    cv[5] = 0.3
    cv[6] = 0.3
    cv[7] = -0.3
    cv[8] = 0.3
    cv[9] = -0.3
    cv[10] = 0.3
    cv[11] = -0.3
    cv[12] = -1.5
    cv[13] = 1.5
    cv[14] = -1.5
    cv[15] = 1.5
    cv[16] = -1.5
    cv[17] = 1.5
    cs = [cv[i] for i in xrange(len(cv))]
    cs.extend([0.0 for i in xrange(len(robot_dof_values))]) 
    #cv[2] = 0.0
    current_state[:] = cs
    control_input[:] = [0.0 for i in xrange(len(robot_dof_values))]
    control_error[:] = [0.0 for i in xrange(len(robot_dof_values))]   
    robot = env.getRobot()    
   
    robot_dof_values[:] = cv
    env.setRobotDOFValues(robot_dof_values)
    time.sleep(5)
    env.getSensorManager().activateSensor(sensor_name)
    time.sleep(2)
    env.drawBoxes()    
    env.getSensorManager().deactivateSensor(sensor_name)
    
    time.sleep(100)
   
    while True:
        #print "propagating"
        robot.propagate(current_state,
                        control_input,
                        control_error,
                        simulation_step_size,
                        duration,
                        result)
        result_vec = [result[i] for i in xrange(len(result))]
        print "result " + str(result_vec)
        robot_dof_values[:] = [result[i] for i in xrange(len(result) / 2)]
        
        t0 = time.time()
        collides = env.robotCollidesDiscrete(robot_dof_values)
        t1 = time.time() - t0
        print "collision check time: " + str(t1)        
        #print "collides: " + str(collides)
        env.setRobotDOFValues(robot_dof_values)
        
        current_state[:] = [result[i] for i in xrange(len(result))]
        #print "result_vec " + str(result_vec)
        #print "propagated"
        time.sleep(0.03)
                

def prog(joint_angles, sensor_name):
    joint_angles_v = v_double()
    joint_velocities = v_double()
    particle_joint_values = v2_double()
    joint_angles_v[:] = [joint_angles[i] for i in xrange(len(joint_angles))]
    joint_velocities[:] = [0.0, 0.0, 0.0]
    env.getRobot().setState(joint_angles_v, joint_velocities)
    env.transformSensorSensorLink(joint_angles_v, sensor_name)
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
env.loadRobotFromURDF("model/hexapod.urdf")

env.getRobot().setGravityConstant(9.81)
env.transformSensorToSensorLink(sensor_name)
env.initOctree(0.05)
robot_dof_values = v_double()
env.getRobotDOFValues(robot_dof_values)
robot_dof_values_arr = [robot_dof_values[i] for i in xrange(len(robot_dof_values))]
propagate(env, sensor_name)

time.sleep(100)

new_robot_dof_values[:] = robot_dof_values_arr
env.setRobotDOFValues(new_robot_dof_values)
time.sleep(3)
robot_trans = v_double()
robot_rot = v_double()
robot_trans[:] = [0.0, 0.0, 0.0]
robot_rot[:] = [0.4, 0.4, 0.4]
env.setRobotTransform(robot_trans, robot_rot)
time.sleep(3)
env.transformSensorToSensorLink(sensor_name)
print "activate"



time.sleep(50)
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










       

