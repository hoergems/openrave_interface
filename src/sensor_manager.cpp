#include "include/sensor_manager.hpp"
#include <unistd.h>

using std::cout;
using std::endl;

namespace shared {

SensorManager::SensorManager():
	env_(nullptr),
	sensor_map_(),
	latest_sensor_readings_(),
	data_callbacks_(),
	latest_sensor_data_(nullptr),
	mutex_(){
	
}

void SensorManager::sensor_loop_() {
	OpenRAVE::SensorBase::SensorDataPtr sensor_data;	
	
	// All sensors are off per default
	for(std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
			sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
		OpenRAVE::SensorBasePtr s = iter->second.second;
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOff, true);
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOff, true);		
	}
	
	while (true) {
		std::vector<boost::shared_ptr<SensorDataBase>> last_sensor_readings;
		std::vector<boost::shared_ptr<SensorDataBase>> camera_sensor_readings;
		
		for(std::map<std::string, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
				sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
			OpenRAVE::SensorBasePtr s = iter->second.second;
			sensor_data = s->CreateSensorData(iter->second.first);			
			s->GetSensorData(sensor_data);			
			data_callbacks_[s->GetName()](sensor_data, this);					
		}
		
		usleep(0.02 * 1e6);
	}
}

void SensorManager::setLatestSensorData(LaserSensorDataConstPtr &sensor_data) {	
	boost::mutex::scoped_lock scoped_lock(mutex_);	
	latest_sensor_data_ = sensor_data;
}

void SensorManager::getLatestSensorData(LaserSensorDataConstPtr &sensor_data) {	
	boost::mutex::scoped_lock scoped_lock(mutex_);
	sensor_data = latest_sensor_data_;
	
}

bool SensorManager::activateSensor(std::string name, bool wait_for_sensor_data) {
	if (sensor_map_.find(name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << name << " doesn't exist" << endl;
		return false;
	}
	boost::recursive_mutex::scoped_lock scoped_lock(env_->GetMutex());
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
	if (wait_for_sensor_data) {
		LaserSensorDataConstPtr sensor_data = nullptr;
		unsigned int microseconds = 1000000;
		while (!sensor_data) {			
			getLatestSensorData(sensor_data);
			usleep(microseconds);
		}
	}
	return true;
}

bool SensorManager::disableSensor(std::string name) {
	if (sensor_map_.find(name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << name << " doesn't exist" << endl;
		return false;
	}
	boost::recursive_mutex::scoped_lock scoped_lock(env_->GetMutex());
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOff, true);
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOff, true);
	return true;
}

bool SensorManager::transformSensor(std::string &robot_name, std::string &sensor_name) {
	if (sensor_map_.find(sensor_name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << sensor_name << " doesn't exist" << endl;
		return false;
	}
		
	OpenRAVE::KinBodyPtr robot_kin_body = env_->GetKinBody(robot_name);	
	OpenRAVE::KinBody::LinkPtr sensor_link = robot_kin_body->GetLink("sensor_link");
	OpenRAVE::Transform sensor_link_transform = sensor_link->GetTransform();
	OpenRAVE::Vector new_trans(sensor_link_transform.trans.x + 0.001,
			                   sensor_link_transform.trans.y,
							   sensor_link_transform.trans.z);
	Eigen::Quaternion<double> quat1(sensor_link_transform.rot.w,
			                        sensor_link_transform.rot.x,
									sensor_link_transform.rot.y,
									sensor_link_transform.rot.z);	
	Eigen::Matrix3d rot1;
	double angle = -M_PI / 2.0;
	rot1 << cos(angle), 0.0, sin(angle), 
			0.0, 1.0, 0.0,
			-sin(angle), 0.0, cos(angle);
	Eigen::Quaternion<double> quat2(rot1);
	Eigen::Quaternion<double> quatres = quat2 * quat1;
    OpenRAVE::geometry::RaveVector<double> rot(quatres.x(), quatres.y(), quatres.z(), quatres.w());
	OpenRAVE::Transform sensor_link_transform_rot(rot, new_trans);
	sensor_map_[sensor_name].second->SetTransform(sensor_link_transform_rot);	
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

void SensorManager::register_callback_(const std::string &sensor_name,
	    		                       OpenRAVE::SensorBase::SensorType sensor_type) {
	boost::function<void(OpenRAVE::SensorBase::SensorDataConstPtr, shared::SensorManager*)> f;
	shared::sensor_callback sc;
	sc.type = sensor_type;	
	f = sc;
	data_callbacks_[sensor_name] = f;
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
			register_callback_(sensor_name, OpenRAVE::SensorBase::SensorType::ST_Laser);
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