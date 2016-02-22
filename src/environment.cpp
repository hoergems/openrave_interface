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
	robot_(nullptr),
	collision_manager_(nullptr),
	robot_model_file_(),
	particle_plot_limit_(50),
	octree_(nullptr),
	tree_ptr_(nullptr),
	kin_bodies_default_color_()
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
	setupDefaultObstacleColors_();
	collision_manager_ = std::make_shared<shared::CollisionManager>();
	collision_manager_->setEnvironment(env_);
	collision_manager_->triangulateScene();
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

std::shared_ptr<shared::CollisionManager> Environment::getCollisionManager() {
	return collision_manager_;
}

void Environment::transformSensorToEndEffector(const std::vector<double> &joint_angles, std::string name) {
	Eigen::MatrixXd end_effector_transform = robot_->getEndEffectorTransform(joint_angles);
	sensor_manager_->transformSensor(name, end_effector_transform);
}

bool Environment::loadRobotFromURDF(std::string robot_file) {
	if (!environment_setup_) {
		cout << "Error: Couldn't load robot. Setup the environment first" << endl;
		return false;				
	}
	
	OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_file, env_);
	env_->Add(robot_ptr, true);	
	robot_ = std::make_shared<shared::Robot>(robot_file);
	robot_model_file_ = robot_file;
	return true;
}

void Environment::initOctree() {
	// The FCL octree
	octree_ = boost::make_shared<octomap::OcTree>(0.1);
	tree_ptr_ = boost::make_shared<fcl::OcTree>(octree_);
	collision_manager_->setOctree(tree_ptr_);
}

void Environment::drawBoxes() {	
	LaserSensorDataConstPtr laser_data; 
	sensor_manager_->getLatestSensorData(laser_data);
	std::vector<double> robot_state;
	std::vector<double> joint_angles;
	robot_->getState(robot_state);
	for (size_t i = 0; i < robot_state.size() / 2; i++) {
		joint_angles.push_back(robot_state[i]);
	}
	
	octomap::Pointcloud scan_points;
	const octomap::point3d origin(laser_data->positions[0].x, 
			                      laser_data->positions[0].y, 
								  laser_data->positions[0].z);
	
 	for (size_t i = 0; i < laser_data->ranges.size() - 1; i++) {
		if (laser_data->intensity[i] > 0) {
			const octomap::point3d point(laser_data->positions[0].x + laser_data->ranges[i].x,
					                     laser_data->positions[0].y + laser_data->ranges[i].y,
										 laser_data->positions[0].z + laser_data->ranges[i].z);
			scan_points.push_back(point);
		}
	}
 	
	octree_->insertScan(scan_points, origin);
	std::vector<boost::array<double, 6> > tree_boxes = tree_ptr_->toBoxes();
	std::vector<OpenRAVE::AABB> rave_boxes;
	OpenRAVE::KinBodyPtr box_kin_body = OpenRAVE::RaveCreateKinBody(env_);
	for (size_t i = 0; i < tree_boxes.size(); i++) {
		cout << "x: " << tree_boxes[i][0] << endl;
		cout << "y: " << tree_boxes[i][1] << endl;
		cout << "z: " << tree_boxes[i][2] << endl;		
		cout << "size: " << tree_boxes[i][3] << endl;
		cout << "c: " << tree_boxes[i][4] << endl;
		cout << "t: " << tree_boxes[i][5] << endl;
		OpenRAVE::Vector trans(tree_boxes[i][0], 
				               tree_boxes[i][1],
				               tree_boxes[i][2]);
		OpenRAVE::Vector extends(tree_boxes[i][3] / 2, 
				                 tree_boxes[i][3] / 2, 
				                 tree_boxes[i][3] / 2);
		OpenRAVE::AABB aabb(trans, extends);
		rave_boxes.push_back(aabb);
	}
	const std::vector<OpenRAVE::AABB> const_rave_boxes = rave_boxes;
	const std::string box_name = "octomap";
	box_kin_body->SetName(box_name);    
	box_kin_body->InitFromBoxes(const_rave_boxes, true);
	box_kin_body->Enable(false); 
	env_->Add(box_kin_body, true);
	cout << "tree depth: " << octree_->getTreeDepth() << endl;
}

void Environment::plotPermanentParticles(const std::vector<std::vector<double>> &particle_joint_values,
		                                 const std::vector<std::vector<double>> &particle_colors) {	
	double num_plot = particle_joint_values.size();
	for (size_t i = 0; i < num_plot; i++) {
	    OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_model_file_, env_);
		std::string name = "permanent_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
	    env_->Add(robot_ptr, true);
	    std::vector<OpenRAVE::dReal> joint_vals;
	    for (auto &k: particle_joint_values[i]) {
	        joint_vals.push_back(k);
	    }
	        
	    joint_vals.push_back(0);
	        
	    robot_ptr->SetDOFValues(joint_vals);
	        
	    const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
	    for (auto &link: links) {
	        const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
	        for (auto &geometry: link_geometries) {
	            if (geometry->IsVisible()) {
	        	    OpenRAVE::Vector color(particle_colors[i][0], 
	        					           particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
	        		geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
	        		geometry->SetTransparency(0.75);
	        	}
	        }
	    }
	}	
}

void Environment::removePermanentParticles() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    // Remove the particle bodies from the scene
    std::string particle_string = "permanent_";
    for (auto &body: bodies) {		
    	if (body->GetName().find(particle_string) != std::string::npos) {			
    		env_->Remove(body);
    	}		
    }
}

void Environment::setupDefaultObstacleColors_() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	for (auto &body: bodies) {
		const std::vector<OpenRAVE::KinBody::LinkPtr> links(body->GetLinks());
		for (auto &link: links) {
			const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries(link->GetGeometries());
			std::vector<OpenRAVE::RaveVector<float>> colors;
			colors.push_back(geometries[0]->GetDiffuseColor());
			colors.push_back(geometries[0]->GetAmbientColor());
			kin_bodies_default_color_[body->GetName()] = colors;	
		}		
	}
}

void Environment::setKinBodiesDefaultColor() {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	for (auto &body: bodies) {
		const std::vector<OpenRAVE::KinBody::LinkPtr> links(body->GetLinks());
		for (auto &link: links) {			
			const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries(link->GetGeometries());			
			std::vector<OpenRAVE::RaveVector<float>> colors = kin_bodies_default_color_[body->GetName()];
			if (colors.size() > 0) {
			    geometries[0]->SetDiffuseColor(kin_bodies_default_color_[body->GetName()][0]);
			    geometries[0]->SetAmbientColor(kin_bodies_default_color_[body->GetName()][1]);
			}
		}
	}
}

void Environment::setObstacleColor(std::string &obstacle_name, 
 		                               std::vector<double> &diffuse_color,
 		                               std::vector<double> &ambient_color) {
 	std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
     for (auto &body: bodies) {    	
     	const std::vector<OpenRAVE::KinBody::LinkPtr> links(body->GetLinks());
     	for (auto &link: links) {
     		const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> geometries(link->GetGeometries());
     		for (auto &geom: geometries) {
     			if (body->GetName().find(obstacle_name) != std::string::npos) {
     				const OpenRAVE::RaveVector<float> d_color(diffuse_color[0],
     						                                  diffuse_color[1],
     						                                  diffuse_color[2],
     						                                  diffuse_color[3]);
     				const OpenRAVE::RaveVector<float> a_color(ambient_color[0],
     				    						              ambient_color[1],
     				    						              ambient_color[2],
     				    						              ambient_color[3]);     				
     				geom->SetDiffuseColor(d_color);
     				geom->SetAmbientColor(a_color);
     						                                        		
     			}
     		}		
     	}
    }
}

void Environment::updateRobotValues(std::vector<double> &current_joint_values,
		                            std::vector<double> &current_joint_velocities,
								    std::vector<std::vector<double>> &particle_joint_values,
									std::vector<std::vector<double>> &particle_colors) {	
	OpenRAVE::RobotBasePtr robot_to_use = getRaveRobot();	
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	std::string particle_string = "particle";
		
	// Remove the particle bodies from the scene	
	for (auto &body: bodies) {		
		if (body->GetName().find(particle_string) != std::string::npos) {			
			env_->Remove(body);
		}		
	}		
	
	if (robot_to_use == nullptr) {
		cout << "Propagator: Error: Environment or robot has not been initialized or passed as argument. Can't propagate the state" << endl;
		return;	
	}
	
	std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_to_use->GetLinks();
	std::vector<OpenRAVE::KinBody::JointPtr> joints = robot_to_use->GetJoints();	
	std::vector<OpenRAVE::KinBody::LinkInfo> link_infos;
	std::vector<OpenRAVE::KinBody::JointInfo> joint_infos;	
	for (auto &link: links) {
		link_infos.push_back(link->GetInfo());
	}
	
	for (auto &joint: joints) {
		joint_infos.push_back(joint->GetInfo());
	}
	
	std::vector<OpenRAVE::dReal> newJointValues;
	for (size_t i = 0; i < current_joint_values.size(); i++) {		
		if (current_joint_values[i] < -M_PI) {
			newJointValues.push_back(2.0 * M_PI + current_joint_values[i]);
		}
		else if (current_joint_values[i] > M_PI) {
			newJointValues.push_back(-2.0 * M_PI + current_joint_values[i]);
		}
		else {
			newJointValues.push_back(current_joint_values[i]);
		}
	}
	
	boost::recursive_mutex::scoped_lock scoped_lock(env_->GetMutex());	
	newJointValues.push_back(0);	
	robot_to_use->SetDOFValues(newJointValues);
	size_t num_plot = particle_plot_limit_;	
	if (particle_joint_values.size() < num_plot) {
		num_plot = particle_joint_values.size();
	}
	
	
	// Here we plot non-permanent particles
	for (size_t i = 0; i < num_plot; i++) {
		OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_model_file_, env_);
		std::string name = "particle_robot_";
		name.append(std::to_string(i));
		robot_ptr->SetName(name);
        env_->Add(robot_ptr, true);
        std::vector<OpenRAVE::dReal> joint_vals;
        for (auto &k: particle_joint_values[i]) {
        	joint_vals.push_back(k);
        }
        
        joint_vals.push_back(0);        
        robot_ptr->SetDOFValues(joint_vals);        
        const std::vector<OpenRAVE::KinBody::LinkPtr> links = robot_ptr->GetLinks();
        for (auto &link: links) {
        	const std::vector<OpenRAVE::KinBody::Link::GeometryPtr> link_geometries = link->GetGeometries();
        	for (auto &geometry: link_geometries) {
        		if (geometry->IsVisible()) {
        			OpenRAVE::Vector color(particle_colors[i][0], 
        					               particle_colors[i][1], 
										   particle_colors[i][2], 
										   particle_colors[i][3]);
        			geometry->SetDiffuseColor(color);
					geometry->SetAmbientColor(color);
        			geometry->SetTransparency(particle_colors[i][3]);
        		}
        	}
        }
	}
}

OpenRAVE::RobotBasePtr Environment::getRaveRobot() {
    std::vector<OpenRAVE::KinBodyPtr> bodies;
    env_->GetBodies(bodies);
    for (auto &body: bodies) {    	
    	if (body->GetLinks().size() >1) {
    		OpenRAVE::RobotBasePtr robot = boost::static_pointer_cast<OpenRAVE::RobotBase>(body);
    		return robot;
    	}    	
    }   
}

void Environment::getGoalArea(std::vector<double> &goal_area) {
	std::vector<OpenRAVE::KinBodyPtr> bodies;
	env_->GetBodies(bodies);
	for (auto &body: bodies) {
		cout << "body name: " << body->GetName() << endl;
		if (body->GetName() == "GoalArea") {
			goal_area.push_back(body->GetLinks()[0]->GetGeometries()[0]->GetTransform().trans.x);
			goal_area.push_back(body->GetLinks()[0]->GetGeometries()[0]->GetTransform().trans.y);
			goal_area.push_back(body->GetLinks()[0]->GetGeometries()[0]->GetTransform().trans.z);
			goal_area.push_back(body->GetLinks()[0]->GetGeometries()[0]->GetSphereRadius());
		}
	}
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
	
	
	class_<shared::SensorManager, boost::noncopyable>("SensorManager")
			.def("activateSensor", &shared::SensorManager::activateSensor)
			.def("deactivateSensor", &shared::SensorManager::disableSensor)
	;
	//class_<shared::Robot>("Robot", init<std::string>());
	register_ptr_to_python<std::shared_ptr<fcl::CollisionObject>>();
	register_ptr_to_python<std::shared_ptr<shared::SensorManager>>();
	register_ptr_to_python<std::shared_ptr<shared::Robot>>();
	register_ptr_to_python<std::shared_ptr<shared::CollisionManager>>();
	
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
							.def("setState", &Robot::setState)
							.def("getState", &Robot::getState)
	;
	
	class_<CollisionManager>("CollisionChecker", init<>())				
				.def("inCollisionDiscreteEnvironment", &CollisionManager::inCollisionDiscreteEnvironmentPy)
				.def("inCollisionContinuousEnvironment", &CollisionManager::inCollisionContinuousEnvironmentPy)
				.def("inCollisionDiscreteOctree", &CollisionManager::inCollisionDiscreteOctreePy)
				.def("inCollisionContinuousOctree", &CollisionManager::inCollisionContinuousOctreePy)
	;
	
	
	class_<Environment, boost::shared_ptr<Environment>>("Environment", init<>())
		.def("setupEnvironment", &Environment::setupEnvironment)
		.def("loadSensors", &Environment::loadSensorsFromXML)
		.def("showViewer", &Environment::showViewer)
		.def("getSensorManager", &Environment::getSensorManager)
		.def("getCollisionManager", &Environment::getCollisionManager)
		.def("loadRobotFromURDF", &Environment::loadRobotFromURDF)
		.def("getRobot", &Environment::getRobot)
		.def("plotPermanentParticles", &Environment::plotPermanentParticles)
		.def("transformSensorToEndEffector", &Environment::transformSensorToEndEffector)		
		.def("updateRobotValues", &Environment::updateRobotValues)
		.def("initOctree", &Environment::initOctree)
		.def("drawBoxes", &Environment::drawBoxes)
		.def("setKinBodiesDefaultColor", &Environment::setKinBodiesDefaultColor)
		.def("setObstacleColor", &Environment::setObstacleColor)
		.def("getGoalArea", &Environment::getGoalArea)
		.def("plotPermanentParticles", &Environment::plotPermanentParticles)
		.def("removePermanentParticles", &Environment::removePermanentParticles)
	;
	
}

}