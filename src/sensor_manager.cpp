#include "include/sensor_manager.hpp"

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

bool SensorManager::activateSensor(std::string name) {
	if (sensor_map_.find(name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << name << " doesn't exist" << endl;
		return false;
	}
	boost::recursive_mutex::scoped_lock scoped_lock(env_->GetMutex());
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
	sensor_map_[name].second->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
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

bool SensorManager::transformSensor(std::string &name, Eigen::MatrixXd &transform) {
	if (sensor_map_.find(name) == sensor_map_.end()) {
		cout << "SensorManager: Error: sensor " << name << " doesn't exist" << endl;
		return false;
	}
	if (name == "FlashLidar3D") {
		Eigen::MatrixXd rot1(4, 4);
		double angle = M_PI / 2.0;
		rot1 << cos(angle), 0.0, sin(angle), 0.0,
				0.0, 1.0, 0.0, 0.0,
				-sin(angle), 0.0, cos(angle), 0.0,
				0.0, 0.0, 0.0, 1.0;
		transform = transform * rot1;
	}
	
	Eigen::Matrix3d mat;
	mat << transform(0, 0), transform(0, 1), transform(0, 2),
		   transform(1, 0), transform(1, 1), transform(1, 2),
		   transform(2, 0), transform(2, 1), transform(2, 2);
	Eigen::Quaternion<double> quat(mat);
	OpenRAVE::geometry::RaveVector<double> rot(quat.w(), quat.x(), quat.y(), quat.z());
	OpenRAVE::geometry::RaveVector<double> trans(transform(0, 3) + 0.001, 
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

void SensorManager::register_callback_(const std::string &sensor_name,
	    		                       OpenRAVE::SensorBase::SensorType sensor_type) {
	boost::function<void(OpenRAVE::SensorBase::SensorDataConstPtr, shared::SensorManager*)> f;
	shared::sensor_callback sc;
	sc.type = sensor_type;
	//f = &sc;
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