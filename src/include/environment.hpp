#ifndef _OPENRAVE_ENVIRONMENT_HPP
#define _OPENRAVE_ENVIRONMENT_HPP
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "urdf_loader.hpp"
#include "viewer.hpp"
#include "sensor_manager.hpp"
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
	
    private:
	    /**
	     * The sensor loop
	     */
	    void sensor_loop_();
	    
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
	
};

}

#endif