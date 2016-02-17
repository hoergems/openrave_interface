#include "include/environment.hpp"
#include <unistd.h>

using std::cout;
using std::endl;

namespace shared {

Environment::Environment():
    environment_setup_(false),
	env_(nullptr),
	sensor_manager_(new shared::SensorManager()),
	viewer_(new shared::RaveViewer()),
	urdf_loader_()
{
	
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
	sensor_manager_->setEnvironment(env_);
	return environment_setup_;
}

bool Environment::loadSensorsFromXML(std::vector<std::string> &sensor_files) {
	return sensor_manager_->loadSensorsFromXML(sensor_files);
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