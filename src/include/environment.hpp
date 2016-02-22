#ifndef _OPENRAVE_ENVIRONMENT_HPP
#define _OPENRAVE_ENVIRONMENT_HPP
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "urdf_loader.hpp"
#include "viewer.hpp"
#include "sensor_manager.hpp"
#include "robot.hpp"
#include <map>
#include <tuple>
#include <fcl/octree.h>
#include <fcl/config.h>

namespace shared {

class Environment {
    public:
	    Environment();
	    
	    /**
	     * Loads the environment and the robot model
	     */
	    bool setupEnvironment(std::string environment_file);	    
	    
	    /**
	     * Loads a robot from a URDF description
	     */
	    bool loadRobotFromURDF(std::string robot_file);
	    
	    /**
	     * Load the sensors from a XML file
	     */
	    bool loadSensorsFromXML(std::vector<std::string> &sensor_files);
	    
	    /**
	     * Shows the viewer
	     */
	    bool showViewer(); 
	    
	    /**
	     * Returns the sensor manager
	     */
	    std::shared_ptr<SensorManager> getSensorManager();
	    
	    /**
	     * Gets the robot
	     */
	    std::shared_ptr<shared::Robot> getRobot();
	    
	    /**
	     * Update the robot values in the viewer
	     */
	    void updateRobotValues(std::vector<double> &current_joint_values,
	    		   	   		   std::vector<double> &current_joint_velocities,	
	    					   std::vector<std::vector<double>> &particle_joint_values,
	    					   std::vector<std::vector<double>> &particle_colors);
	    
	    /**
	     * Plot permanent particles
	     */
	    void plotPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
	    		                    const std::vector<std::vector<double>> &particle_colors);
	    
	    /**
	     * Remove the permanent particles
	     */
	    void removePermanentParticles();
	    
	    /**
	     * Transforms a given sensor to the end effector frame
	     */
	    void transformSensorToEndEffector(const std::vector<double> &joint_angles, std::string name);
	    
	    /**
	     * Triangulate the scene
	     */
	    void triangulateScene();
	    
	    /**
	     * Initializes the Octree
	     */
	    void initOctree();
	    
	    void drawBoxes();
	    
	    void setObstacleColor(std::string &obstacle_name, 
	         		          std::vector<double> &diffuse_color,
	        		          std::vector<double> &ambient_color);
	    
	    void getGoalArea(std::vector<double> &goal_area);
	    
	    void setKinBodiesDefaultColor();
	
    private:
	    /**
	     * Gets the OpenRAVE robot from the environment
	     */
	    OpenRAVE::RobotBasePtr getRaveRobot();	    
	    
	    /**
	     * The sensor manager
	     */
	    std::shared_ptr<SensorManager> sensor_manager_;
	    
	    void setupDefaultObstacleColors_();
	    
	    /**
	     * Determines of the environment has been set up
	     */
	    bool environment_setup_;
	    
	    /**
	     * The OpenRAVE environment
	     */
	    OpenRAVE::EnvironmentBasePtr env_;
	    
	    /**
	     * The viewer
	     */
	    std::shared_ptr<shared::RaveViewer> viewer_;
	    
	    /**
	     * Parses URDF files
	     */
	    std::shared_ptr<shared::URDFLoader> urdf_loader_;
	    
	    /**
	     * The robot
	     */
	    std::shared_ptr<shared::Robot> robot_;
	    
	    /**
	     * The robot model file
	     */
	    std::string robot_model_file_;
	    
	    /**
	     * The maximum number of particles to plot
	     */
	    unsigned int particle_plot_limit_;
	    
	    boost::shared_ptr<octomap::OcTree> octree_;
	    
	    boost::shared_ptr<fcl::OcTree> tree_ptr_;
	    
	    std::map<std::string, std::vector<OpenRAVE::RaveVector<float>>> kin_bodies_default_color_;
	
};

}

#endif