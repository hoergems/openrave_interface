#include "include/environment.hpp"
#include <unistd.h>

using std::cout;
using std::endl;

namespace shared {

Environment::Environment():
    environment_setup_(false),
	env_(nullptr),
	viewer_(new shared::RaveViewer()),
	urdf_loader_(),
	sensor_map_()
{
	
}

void Environment::sensor_loop_() {
	OpenRAVE::SensorBase::SensorDataPtr sensor_data;
	
	// Turn on the sensors
	for(std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
			sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
		OpenRAVE::SensorBasePtr s = iter->second.second;
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_PowerOn, true);
		s->Configure(OpenRAVE::SensorBase::ConfigureCommand::CC_RenderDataOn, true);
	}
		    
	while (true) {
		for(std::map<unsigned int, std::pair<OpenRAVE::SensorBase::SensorType, OpenRAVE::SensorBasePtr>>::iterator iter = 
				sensor_map_.begin(); iter != sensor_map_.end(); ++iter) {
			OpenRAVE::SensorBasePtr s = iter->second.second;
			sensor_data = s->CreateSensorData(iter->second.first);
			s->GetSensorData(sensor_data);
			boost::shared_ptr<OpenRAVE::SensorBase::LaserSensorData> laser_sensor_data = 
					boost::static_pointer_cast<OpenRAVE::SensorBase::LaserSensorData>(sensor_data);
			for (size_t i = 0; i < laser_sensor_data->intensity.size(); i++) {
				if (laser_sensor_data->intensity[i] > 0) {
					cout << "INTENSE!!!" << endl;
				}
			}
			
			usleep(0.02 * 1e6);
		}
	}
}

bool Environment::setupEnvironment(std::string environment_file) {
	if (environment_setup_) {
		return false;
	}
	
	OpenRAVE::RaveInitialize(true);	
	OpenRAVE::RaveSetDebugLevel(OpenRAVE::Level_Debug);
	env_ = OpenRAVE::RaveCreateEnvironment();
	env_->SetPhysicsEngine(nullptr);
	cout << "loading " << environment_file << endl;
	env_->Load(environment_file);	
	cout << "loaded environment" << endl;
	environment_setup_ = true;
	//loadSensorsFromXML(sensor_files);
	return environment_setup_;
}

bool Environment::showViewer() {
	if (!environment_setup_) {
		cout << "Error: Can't show viewer. Setup your environment first" << endl;
		return false;
	}
	shared::RaveViewer viewer;
	viewer_->testView(env_);
	return true;	
}

bool Environment::loadSensorsFromXML(std::vector<std::string> &sensor_files) {
	if (!environment_setup_) {
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
	
	boost::thread sensor_thread(boost::bind(&Environment::sensor_loop_, this));	
	return true;
	
}

bool Environment::loadRobotFromURDF(std::string robot_file) {
	if (!environment_setup_) {
		cout << "Error: Couldn't load robot. Setup the environment first" << endl;
		return false;				
	}
	
	OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_file, env_);
	env_->Add(robot_ptr, true);	
	return true;
}

BOOST_PYTHON_MODULE(libopenrave_interface) { 
	using namespace boost::python;
	
	class_<std::vector<std::string> > ("v_string")
	                 .def(vector_indexing_suite<std::vector<std::string> >());  
	
	class_<Environment, boost::shared_ptr<Environment>>("Environment", init<>())
		.def("setupEnvironment", &Environment::setupEnvironment)
		.def("loadSensors", &Environment::loadSensorsFromXML)
		.def("showViewer", &Environment::showViewer)
	;
	
}

}