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
	    void updateRobotValues(const std::vector<double> &current_joint_values,
	    		   	   		   const std::vector<double> &current_joint_velocities,	
	    					   const std::vector<std::vector<double>> &particle_joint_values,
	    					   const std::vector<std::vector<double>> &particle_colors,
	    		   	   		   OpenRAVE::RobotBasePtr robot);
	    
	    /**
	     * Plot permanent particles
	     */
	    void plotPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
	    		                    const std::vector<std::vector<double>> &particle_colors);
	    
	    void transformSensorToEndEffector(const std::vector<double> &joint_angles, std::string name);
	
    private:
	    /**
	     * Gets the OpenRAVE robot from the environment
	     */
	    OpenRAVE::RobotBasePtr getRaveRobot();
	    
	    /**
	     * The sensor manager
	     */
	    std::shared_ptr<SensorManager> sensor_manager_;
	    
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
	
};

}

#endif