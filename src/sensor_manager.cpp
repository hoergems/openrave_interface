#include "include/sensor_manager.hpp"

using std::cout;
using std::endl;

namespace shared {

SensorManager::SensorManager():
	env_(nullptr),
	sensor_map_(){
	
}

void SensorManager::sensor_loop_() {
	OpenRAVE::SensorBase::SensorDataPtr sensor_data;
	
	// Turn on the sensors
	for(std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
			sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
		OpenRAVE::SensorBasePtr s = iter->second.second;
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
	}
		    
	while (true) {
		std::vector<boost::shared_ptr<SensorDataBase>> last_sensor_readings;
		for(std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
				sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
			OpenRAVE::SensorBasePtr s = iter->second.second;
			sensor_data = s->CreateSensorData(iter->second.first);
			s->GetSensorData(sensor_data);
			boost::shared_ptr<SensorDataBase> sensor_data_ptr;
			if (iter->second.first == OpenRAVE::SensorBase::SensorType::ST_Laser) {
				sensor_data_ptr = boost::make_shared<SensorData<OpenRAVE::SensorBase::LaserSensorData>>(sensor_data);
				last_sensor_readings.push_back(sensor_data_ptr);
			}
			//shared::SensorData<OpenRAVE::SensorBase::LaserSensorData> d(sensor_data);
			
			//d.getSensorData()->intensity;
			boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> laser_sensor_data = 
					boost::static_pointer_cast<OpenRAVE::SensorBase::LaserSensorData>(sensor_data);
			
			sleep(2);
			/**for (size_t i = 0; i < laser_sensor_data->intensity.size(); i++) {
				if (laser_sensor_data->intensity[i] > 0) {
					cout << "INTENSE!!!" << endl;
				}
			}*/
			
			usleep(0.02 * 1e6);
		}
	}
}

bool SensorManager::setEnvironment(OpenRAVE::EnvironmentBasePtr &env) {
	if (env_) {
		cout << "SensorManager: Error: Environment has already been set" << endl;
		return false;
	}
	env_ = env;
	return true;
}


bool SensorManager::loadSensorsFromXML(std::vector<std::string> &sensor_files) {
	if (!env_) {
		cout << "Error: Can't load sensors. Setup your environment first" << endl;
		return false;
	}
	
	for (unsigned int i = 0; i < sensor_files.size(); i++) {		
		OpenRAVE::InterfaceBasePtr sensor_interface = env_->ReadInterfaceXMLFile(sensor_files[i]);
		env_->Add(sensor_interface);
		OpenRAVE::SensorBasePtr sensor_ptr = boost::static_pointer_cast<OpenRAVE::SensorBase>(sensor_interface);
		if (sensor_ptr->Supports(OpenRAVE::SensorBase::SensorType::ST_Laser)) {
			sensor_map_[i] = std::make_pair(OpenRAVE::SensorBase::SensorType::ST_Laser,
					                        sensor_ptr);
		}
		else if (sensor_ptr->Supports(OpenRAVE::SensorBase::SensorType::ST_Camera)) {
			sensor_map_[i] = std::make_pair(OpenRAVE::SensorBase::SensorType::ST_Camera,
								            sensor_ptr);
		}
	}
	
	boost::thread sensor_thread(boost::bind(&SensorManager::sensor_loop_, this));	
	return true;
	
}

}