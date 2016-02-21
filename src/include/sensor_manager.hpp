#ifndef _SENSOR_MANAGER_HPP_
#define _SENSOR_MANAGER_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/make_shared.hpp>
#include <Eigen/Dense>
#include <boost/thread/mutex.hpp>
#include "SensorDataBase.hpp"

using std::cout;
using std::endl;

namespace shared {

typedef boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> LaserSensorDataPtr;
typedef boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData const> LaserSensorDataConstPtr;



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
	    bool transformSensor(std::string &name, Eigen::MatrixXd &transform);
	    
	    /**
	     * Turn of a sensor
	     */
	    bool disableSensor(std::string name);
	    
	    /**
	     * Activates a sensor
	     */
	    bool activateSensor(std::string name);
	    
	    void setLatestSensorData(LaserSensorDataConstPtr &sensor_data);
	    
	    /**
	     * Obtain the latest laser sensor data
	     */
	    void getLatestSensorData(LaserSensorDataConstPtr &sensor_data);
	    
    private:	    
	    /**
	     * The main sensor loop
	     */
	    void sensor_loop_();
	    
	    void register_callback_(const std::string &sensor_name,
	    		                OpenRAVE::SensorBase::SensorType sensor_type);
	    
	    /**
	     * The external sensors
	     */   
	    //std::map<unsigned int, std::pair<OpenRAVE:SensorBase::SensorType, std::string>> mymap;	   
	    std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr> > sensor_map_;
	    
	    std::map<std::string, OpenRAVE::SensorBase::SensorDataConstPtr> latest_sensor_readings_;
	    
	    OpenRAVE::EnvironmentBasePtr env_;
	    
	    bool environment_setup_;
	    
	    LaserSensorDataConstPtr latest_sensor_data_;
	    
	    boost::mutex mutex_;
	    
	    std::map<std::string, boost::function<void(OpenRAVE::SensorBase::SensorDataConstPtr, shared::SensorManager*)>> data_callbacks_;
	
};

struct sensor_callback {
	OpenRAVE::SensorBase::SensorType type;
	uint64_t last_stamp = 0;
	void operator()(OpenRAVE::SensorBase::SensorDataConstPtr &sensor_data,
			        shared::SensorManager *manager) {
		if (sensor_data->__stamp > last_stamp) {
			last_stamp = sensor_data->__stamp;
			LaserSensorDataConstPtr laser_data = 
					boost::static_pointer_cast<OpenRAVE::SensorBase::LaserSensorData const>(sensor_data);
			cout << "HELLO" << endl;
			manager->setLatestSensorData(laser_data);
			/**for (size_t i = 0; i < laser_data->ranges.size(); i++) {
				cout << "len positions: " << laser_data->positions.size() << endl;;
				cout << "len ranges: " << laser_data->ranges.size() << endl;
				cout << "range " << i << "(" <<
						laser_data->ranges[i].x << ", " <<
						laser_data->ranges[i].y << ", " <<
						laser_data->ranges[i].z << ", " <<
						laser_data->ranges[i].w << ")" << endl;
				cout << "position " << i << "(" <<
						laser_data->positions[0].x << ", " <<
						laser_data->positions[0].y << ", " <<
						laser_data->positions[0].z << ", " <<
						laser_data->positions[0].w << ")" << endl;
				
				
			}*/
		}
		
	}
};

}

#endif