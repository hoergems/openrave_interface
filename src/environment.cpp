#include "include/environment.hpp"
#include <unistd.h>

using std::cout;
using std::endl;

namespace shared {

template<class T>
struct VecToList
{
    static PyObject* convert(const std::vector<T>& vec)
    {
        boost::python::list* l = new boost::python::list();
        for(size_t i = 0; i < vec.size(); i++)
            (*l).append(vec[i]);

        return l->ptr();
    }
};

Environment::Environment():
    environment_setup_(false),
	env_(nullptr),
	sensor_manager_(new shared::SensorManager()),
	viewer_(new shared::RaveViewer()),
	urdf_loader_(),
	robot_(nullptr)
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

std::shared_ptr<SensorManager> Environment::getSensorManager() {
	return sensor_manager_;
}

std::shared_ptr<shared::Robot> Environment::getRobot() {
	return robot_;
}


bool Environment::loadRobotFromURDF(std::string robot_file) {
	if (!environment_setup_) {
		cout << "Error: Couldn't load robot. Setup the environment first" << endl;
		return false;				
	}
	
	OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_file, env_);
	env_->Add(robot_ptr, true);	
	robot_ = std::make_shared<shared::Robot>(robot_file);
	return true;
}

BOOST_PYTHON_MODULE(libopenrave_interface) { 
	using namespace boost::python;
	
	class_<std::vector<double> > ("v_double")
	             .def(vector_indexing_suite<std::vector<double> >());
	    
	class_<std::vector<int> > ("v_int")
	             .def(vector_indexing_suite<std::vector<int> >());
	    
	class_<std::vector<std::vector<double> > > ("v2_double")
	             .def(vector_indexing_suite<std::vector<std::vector<double> > >());
	    
	class_<std::vector<std::vector<int> > > ("v2_int")
	             .def(vector_indexing_suite<std::vector<std::vector<int> > >());
	
	class_<std::vector<std::string> > ("v_string")
	                 .def(vector_indexing_suite<std::vector<std::string> >());
	
	class_<fcl::OBB>("OBB");
	class_<fcl::CollisionObject>("CollisionObject", init<const boost::shared_ptr<fcl::CollisionGeometry>, const fcl::Transform3f>());
	to_python_converter<std::vector<fcl::OBB, std::allocator<fcl::OBB> >, VecToList<fcl::OBB> >();
	to_python_converter<std::vector<fcl::CollisionObject, std::allocator<fcl::CollisionObject> >, VecToList<fcl::CollisionObject> >();
	to_python_converter<std::vector<std::shared_ptr<fcl::CollisionObject>, std::allocator<std::shared_ptr<fcl::CollisionObject>> >, 
		                    VecToList<std::shared_ptr<fcl::CollisionObject>> >();
	
	
	class_<shared::SensorManager>("SensorManager");
	class_<shared::Robot>("Robot", init<std::string>());
	register_ptr_to_python<std::shared_ptr<fcl::CollisionObject>>();
	register_ptr_to_python<std::shared_ptr<shared::SensorManager>>();
	register_ptr_to_python<std::shared_ptr<shared::Robot>>();
	
	class_<Robot>("Robot", init<std::string>())
	                        .def("getLinkNames", &Robot::getLinkNames)
	                        .def("getLinkDimension", &Robot::getLinkDimension)
	                        .def("getActiveLinkDimensions", &Robot::getActiveLinkDimensions)
	                        .def("getLinkMasses", &Robot::getLinkMasses)
	                        .def("getLinkPose", &Robot::getLinkPose)
	                        .def("getLinkInertialPose", &Robot::getLinkInertialPose)
	                        .def("getLinkInertias", &Robot::getLinkInertias)
	                        .def("getJointNames", &Robot::getJointNames)
	                        .def("getActiveJoints", &Robot::getActiveJoints)
	                        .def("getJointType", &Robot::getJointType)
							.def("getJointDamping", &Robot::getJointDamping)
	                        .def("getJointOrigin", &Robot::getJointOrigin)
	                        .def("getJointAxis", &Robot::getJointAxis)
	                        .def("propagate", &Robot::propagate)
	                        //.def("createRobotCollisionStructures", &Robot::createRobotCollisionStructuresPy)
	                        .def("createRobotCollisionObjects", &Robot::createRobotCollisionObjectsPy)
							.def("createEndEffectorCollisionObject", &Robot::createEndEffectorCollisionObjectPy)
	                        .def("getEndEffectorPosition", &Robot::getEndEffectorPosition)
	                        .def("getDOF", &Robot::getDOF)
							.def("getJointLowerPositionLimits", &Robot::getJointLowerPositionLimits)
							.def("getJointUpperPositionLimits", &Robot::getJointUpperPositionLimits)
							.def("getJointVelocityLimits", &Robot::getJointVelocityLimits)
							.def("getJointTorqueLimits", &Robot::getJointTorqueLimits)
							.def("enforceConstraints", &Robot::enforceConstraints)
							.def("constraintsEnforced", &Robot::constraintsEnforced)
							.def("setGravityConstant", &Robot::setGravityConstant)
							.def("setExternalForce", &Robot::setExternalForce)
							.def("setAccelerationLimit", &Robot::setAccelerationLimit)
							.def("getEndEffectorVelocity", &Robot::getEndEffectorVelocity)
							.def("getProcessMatrices", &Robot::getProcessMatrices)						
							.def("getEndEffectorJacobian", &Robot::getEndEffectorJacobian)
	;
	
	class_<Environment, boost::shared_ptr<Environment>>("Environment", init<>())
		.def("setupEnvironment", &Environment::setupEnvironment)
		.def("loadSensors", &Environment::loadSensorsFromXML)
		.def("showViewer", &Environment::showViewer)
		.def("getSensorManager", &Environment::getSensorManager)
		.def("loadRobotFromURDF", &Environment::loadRobotFromURDF)
		.def("getRobot", &Environment::getRobot)
	;
	
}

}