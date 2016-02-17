#ifndef _SENSOR_MANAGER_HPP_
#define _SENSOR_MANAGER_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
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
	    
	    /**
	     * Transforms the origin of a sensor
	     */
	    void transformSensor(std::string &name, Eigen::MatrixXd &transform);
	    
	    /**
	     * Turn of a sensor
	     */
	    void disableSensor(std::string &name);
	    
    private:	    
	    /**
	     * The main sensor loop
	     */
	    void sensor_loop_();
	    
	    /**
	     * The external sensors
	     */   
	    //std::map<unsigned int, std::pair<OpenRAVE:SensorBase::SensorType, std::string>> mymap;	   
	    std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr> > sensor_map_;
	    
	    std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr> > latest_sensor_readings_;
	    
	    OpenRAVE::EnvironmentBasePtr env_;
	    
	    bool environment_setup_;
	
};

}

#endif