#include "include/sensor_manager.hpp"

using std::cout;
using std::endl;

namespace shared {

SensorManager::SensorManager():
	env_(nullptr),
	sensor_map_(),
	latest_sensor_readings_(){
	
}

void SensorManager::sensor_loop_() {
	OpenRAVE::SensorBase::SensorDataPtr sensor_data;
	
	// Turn on the sensors
	for(std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
			sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
		OpenRAVE::SensorBasePtr s = iter->second.second;
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
	}
		    
	while (true) {
		std::vector<boost::shared_ptr<SensorDataBase>> last_sensor_readings;
		std::vector<boost::shared_ptr<SensorDataBase>> camera_sensor_readings;
		for(std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
				sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
			OpenRAVE::SensorBasePtr s = iter->second.second;
			sensor_data = s->CreateSensorData(iter->second.first);
			s->GetSensorData(sensor_data);
			boost::shared_ptr<SensorDataBase> sensor_data_ptr;
			if (iter->second.first == OpenRAVE::SensorBase::SensorType::ST_Laser) {
				cout << "got lazrrrr!!!" << endl;
				sensor_data_ptr = boost::make_shared<SensorData<OpenRAVE::SensorBase::LaserSensorData>>(sensor_data);
				
				//For later
				boost::shared_ptr<SensorData<OpenRAVE::SensorBase::LaserSensorData>> laser_data =
						boost::static_pointer_cast<SensorData<OpenRAVE::SensorBase::LaserSensorData>>(sensor_data_ptr);
				for (size_t i = 0; i < laser_data->getSensorData()->ranges.size(); i++) {
					cout << "range: (" << laser_data->getSensorData()->ranges[i].x << ", " << 
							       laser_data->getSensorData()->ranges[i].y << ", " <<
							       laser_data->getSensorData()->ranges[i].z << ")" << endl;
				}
				last_sensor_readings.push_back(sensor_data_ptr);
			}
			else if (iter->second.first == OpenRAVE::SensorBase::SensorType::ST_Camera) {				
				sensor_data_ptr = boost::make_shared<SensorData<OpenRAVE::SensorBase::CameraSensorData>>(sensor_data);
				last_sensor_readings.push_back(sensor_data_ptr);
				boost::static_pointer_cast<SensorData<OpenRAVE::SensorBase::CameraSensorData>>(sensor_data_ptr)->getSensorData()->vimagedata[0];
			}
			//shared::SensorData<OpenRAVE::SensorBase::LaserSensorData> d(sensor_data);
			
			//d.getSensorData()->intensity;
			boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> laser_sensor_data = 
					boost::static_pointer_cast<OpenRAVE::SensorBase::LaserSensorData>(sensor_data);
			
			sleep(1);
			/**for (size_t i = 0; i < laser_sensor_data->intensity.size(); i++) {
				if (laser_sensor_data->intensity[i] > 0) {
					cout << "INTENSE!!!" << endl;
				}
			}*/
			
			usleep(0.02 * 1e6);
		}
	}
}

bool SensorManager::transformSensor(std::string &name, Eigen::MatrixXd &transform) {
	if (sensor_map_.find(name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << name << " doesn't exist" << endl;
		return false;
	}
	//Eigen::MatrixXd rot1(4, 4);
	//Eigen::MatrixXd trans1(4, 4);
	cout << transform << endl;
	double angle = M_PI;
	/**rot1 << cos(angle), -sin(angle), 0.0, 0.0,
			sin(angle), cos(angle), 0.0, 0.0,
			0.0, 0.0, 1.0, 0.0,
			0.0, 0.0, 0.0, 1.0;*/
	/**rot1 << cos(angle), 0.0, sin(angle), 0.0,
			0.0, 1.0, 0.0, 0.0,
			-sin(angle), 0.0, cos(angle), 0.0,
			0.0, 0.0, 0.0, 1.0;*/
	/**trans1 << 1.0, 0.0, 0.0, 1.0,
			  0.0, 1.0, 0.0, 0.0,
			  0.0, 0.0, 1.0, 0.0,
			  0.0, 0.0, 0.0, 1.0;*/
	/**rot1 << 1.0, 0.0, 0.0, 1.0,
				0.0, 1.0, 0.0, 0.0,
				0.0, 0.0, 1.0, 0.0,
				0.0, 0.0, 0.0, 1.0;*/
    //transform = transform * trans1;
	Eigen::Matrix3d mat;
	mat << transform(0, 0), transform(0, 1), transform(0, 2),
		   transform(1, 0), transform(1, 1), transform(1, 2),
		   transform(2, 0), transform(2, 1), transform(2, 2);
	Eigen::Quaternion<double> quat(mat);
	OpenRAVE::geometry::RaveVector<double> rot(quat.w(), quat.x(), quat.y(), quat.z());
	OpenRAVE::geometry::RaveVector<double> trans(transform(0, 3) + 0.01, 
				                                 transform(1, 3),
											     transform(2, 3));
	const OpenRAVE::Transform sensor_trans(rot, trans);
	sensor_map_[name].second->SetTransform(sensor_trans);
	return true;
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
		const std::string sensor_name = sensor_ptr->GetName();
		if (sensor_ptr->Supports(OpenRAVE::SensorBase::SensorType::ST_Laser)) {
			sensor_map_[sensor_name] = std::make_pair(OpenRAVE::SensorBase::SensorType::ST_Laser,
					                        sensor_ptr);
		}
		else if (sensor_ptr->Supports(OpenRAVE::SensorBase::SensorType::ST_Camera)) {
			sensor_map_[sensor_name] = std::make_pair(OpenRAVE::SensorBase::SensorType::ST_Camera,
								            sensor_ptr);
		}
	}
	
	boost::thread sensor_thread(boost::bind(&SensorManager::sensor_loop_, this));	
	return true;
	
}

}