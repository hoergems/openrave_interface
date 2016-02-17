#ifndef _SENSOR_MANAGER_HPP_
#define _SENSOR_MANAGER_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/make_shared.hpp>
#include "SensorDataBase.hpp"

namespace shared {

class SensorManager {
    public:
	    SensorManager();
	    
	    /**
	     * Load the sensors from a XML file
	     */
	    bool loadSensorsFromXML(std::vector<std::string> &sensor_files);
	    
	    /**
	     * Sets the robot environment
	     */
	    bool setEnvironment(OpenRAVE::EnvironmentBasePtr &env);
	    
    private:	    
	    /**
	     * The main sensor loop
	     */
	    void sensor_loop_();
	    
	    /**
	     * The external sensors
	     */   
	    //std::map<unsigned int, std::pair<OpenRAVE:SensorBase::SensorType, std::string>> mymap;	   
	    std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr> > sensor_map_;
	    
	    OpenRAVE::EnvironmentBasePtr env_;
	    
	    bool environment_setup_;
	
};

}

#endif