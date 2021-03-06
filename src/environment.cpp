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
	robot_name_(""),
	rave_robot_(nullptr),
	rave_robot_clone_(nullptr),
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
	
	//Create a trimesh
	/**OpenRAVE::TriMesh trimesh;
	trimesh.vertices.push_back(OpenRAVE::Vector(0, 0, 1.5));
	trimesh.vertices.push_back(OpenRAVE::Vector(0.5, 0, 1.5));
	trimesh.vertices.push_back(OpenRAVE::Vector(0, 2, 0));
	trimesh.vertices.push_back(OpenRAVE::Vector(-0.5, -1, 0));
	trimesh.vertices.push_back(OpenRAVE::Vector(0, -1.5, 1));
	trimesh.vertices.push_back(OpenRAVE::Vector(1, -0.5, 0));
	trimesh.indices.push_back(0);
	trimesh.indices.push_back(1);
	trimesh.indices.push_back(2);
	trimesh.indices.push_back(0);
	trimesh.indices.push_back(2);
	trimesh.indices.push_back(3);
	trimesh.indices.push_back(0);
	trimesh.indices.push_back(3);
	trimesh.indices.push_back(4);
	trimesh.indices.push_back(0);
	trimesh.indices.push_back(4);
	trimesh.indices.push_back(5);
	//trimesh.indices.push_back(0);
	//trimesh.indices.push_back(5);
	//trimesh.indices.push_back(2);
	
	OpenRAVE::KinBodyPtr trimesh_kinbody = OpenRAVE::RaveCreateKinBody(env_);
	cout << "created" << endl;
	trimesh_kinbody->SetName("trimesh_kinbody");
	trimesh_kinbody->InitFromTrimesh(trimesh);	
	env_->Add(trimesh_kinbody);*/
	
	
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

void Environment::transformSensorToSensorLink(std::string sensor_name) {	
	sensor_manager_->transformSensor(robot_name_, sensor_name);
}

bool Environment::loadRobotFromURDF(std::string robot_file) {
	if (!environment_setup_) {
		cout << "Error: Couldn't load robot. Setup the environment first" << endl;
		return false;				
	}
	
	OpenRAVE::KinBodyPtr robot_ptr = urdf_loader_->load(robot_file, env_);
	OpenRAVE::KinBodyPtr robot_ptr_clone = urdf_loader_->load(robot_file, env_);
	cout << "loaded" << endl;
	rave_robot_ = robot_ptr;
	rave_robot_clone_ = robot_ptr_clone;
	env_->Add(robot_ptr, true);	
	env_->Add(rave_robot_clone_, true);
	env_->Remove(rave_robot_clone_);
	robot_name_ = robot_ptr->GetName();
	robot_ = std::make_shared<shared::Robot>(robot_file);
	robot_model_file_ = robot_file;
	std::vector<OpenRAVE::KinBody::JointPtr> joints(robot_ptr->GetJoints());
	std::vector<double> joint_dampings;
	cout << "joint names: " << endl;
	for (auto &k: joints) {
		auto entry = k->GetInfo()._mapFloatParameters.find("damping");
		if(entry != k->GetInfo()._mapFloatParameters.end()) {
			std::vector<double> d = entry->second;
			joint_dampings.push_back(d[0]);
		}
		else {
			joint_dampings.push_back(0);
		}
		cout << k->GetName() << ", dof index: " << k->GetDOFIndex() << endl;
	}
	robot_->setJointDampings(joint_dampings);
	return true;
}

void Environment::initOctree(double octree_resolution) {
	// The FCL octree
	octree_ = boost::make_shared<octomap::OcTree>(octree_resolution);
	tree_ptr_ = boost::make_shared<fcl::OcTree>(octree_);
	collision_manager_->setOctree(tree_ptr_);
}

void Environment::drawBoxes() {	
	LaserSensorDataConstPtr laser_data; 
	sensor_manager_->getLatestSensorData(laser_data);
	if (!laser_data) {
		cout << "Error: No laser sensor data available" << endl;
		return;
	}
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
	OpenRAVE::KinBodyPtr robot_to_use = getRaveRobot();	
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
	//newJointValues.push_back(0);	
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

OpenRAVE::KinBodyPtr Environment::getRaveRobot() {
    return rave_robot_;  
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

void Environment::getRobotDOFValues(std::vector<double> &dof_values) {
	OpenRAVE::KinBodyPtr robot = getRaveRobot();
	robot->GetDOFValues(dof_values);
}

void Environment::setRobotDOFValues(std::vector<double> &dof_values) {
	OpenRAVE::KinBodyPtr robot = getRaveRobot();
	robot->SetDOFValues(dof_values);
}

void Environment::setRobotTransform(std::vector<double> &trans,
	    		                    std::vector<double> &rot) {
	/**
	 * rot is RPY angles
	 */
	OpenRAVE::KinBodyPtr robot = getRaveRobot();
	Eigen::Matrix3d roll(3, 3);
	Eigen::Matrix3d pitch(3, 3);
	Eigen::Matrix3d yaw(3, 3);
	roll << 1.0, 0.0, 0.0,
			0.0, cos(rot[0]), -sin(rot[0]),
			0.0, sin(rot[0]), cos(rot[0]);
	pitch << cos(rot[1]), 0, sin(rot[1]),
			 0.0, 1.0, 0.0,
			 -sin(rot[1]), 0, cos(rot[1]);
	yaw << cos(rot[2]), -sin(rot[2]), 0,
		   sin(rot[2]), cos(rot[2]), 0,
		   0, 0, 1;
	
	Eigen::Matrix3d rot_matrix = roll * pitch * yaw;
	Eigen::Quaternion<double> quat(rot_matrix);
	OpenRAVE::geometry::RaveVector<double> rot_vec(quat.w(), quat.x(), quat.y(), quat.z());
	OpenRAVE::Vector new_trans(trans[0],
			                   trans[1],
							   trans[2]);
	OpenRAVE::Transform robot_transform(rot_vec, new_trans);
			
	robot->SetTransform(robot_transform);
}

bool Environment::robotCollidesDiscrete(std::vector<double> &dof_values) {	
	rave_robot_clone_->SetDOFValues(dof_values);
	std::vector<OpenRAVE::KinBody::LinkPtr> links = rave_robot_clone_->GetLinks();
	std::vector<OpenRAVE::AABB> aabbs;
	for (auto &l: links) {
		OpenRAVE::AABB aabb = l->ComputeLocalAABB();		
		aabbs.push_back(aabb);		
	}
	
	std::vector<OpenRAVE::Transform> transforms;
	rave_robot_clone_->GetLinkTransformations(transforms);
	
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects;
	
	// Compute the FCL collision objects
	
	for (size_t i = 0; i < transforms.size(); i++) {		
		fcl::Quaternion3f quat(transforms[i].rot.x, transforms[i].rot.y, transforms[i].rot.z, transforms[i].rot.w);
		fcl::Vec3f trans_vec(transforms[i].trans.x, transforms[i].trans.y, transforms[i].trans.z);
		
		fcl::Transform3f trans(quat, trans_vec);
		fcl::AABB link_aabb(fcl::Vec3f(0, -aabbs[i].extents.y, -aabbs[i].extents.z), 
				            fcl::Vec3f(2.0 * aabbs[i].extents.x, aabbs[i].extents.y, aabbs[i].extents.z));
		
		fcl::Box* box = new fcl::Box();  
		fcl::Transform3f box_tf;		
		fcl::constructBox(link_aabb, trans, *box, box_tf);		
		std::shared_ptr<fcl::CollisionObject> coll_obj = 
				std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
		collision_objects.push_back(coll_obj);	
	}
	
	return collision_manager_->inCollisionDiscreteEnvironment(collision_objects);
}

CollisionReport Environment::robotCollidesContinuous(std::vector<double> &dof_values_start,
		                                             std::vector<double> &dof_values_goal) {
	rave_robot_clone_->SetDOFValues(dof_values_start);
	std::vector<OpenRAVE::KinBody::LinkPtr> links = rave_robot_clone_->GetLinks();
	std::vector<OpenRAVE::AABB> aabbs;
	for (auto &l: links) {
		OpenRAVE::AABB aabb = l->ComputeLocalAABB();		
		aabbs.push_back(aabb);		
	}
		
	std::vector<OpenRAVE::Transform> transforms;
	rave_robot_clone_->GetLinkTransformations(transforms);
		
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_start;
		
	// Compute the FCL collision objects
		
	for (size_t i = 0; i < transforms.size(); i++) {		
		fcl::Quaternion3f quat(transforms[i].rot.x, transforms[i].rot.y, transforms[i].rot.z, transforms[i].rot.w);
		fcl::Vec3f trans_vec(transforms[i].trans.x, transforms[i].trans.y, transforms[i].trans.z);
			
		fcl::Transform3f trans(quat, trans_vec);
		fcl::AABB link_aabb(fcl::Vec3f(0, -aabbs[i].extents.y, -aabbs[i].extents.z), 
					            fcl::Vec3f(2.0 * aabbs[i].extents.x, aabbs[i].extents.y, aabbs[i].extents.z));
			
		fcl::Box* box = new fcl::Box();  
		fcl::Transform3f box_tf;		
		fcl::constructBox(link_aabb, trans, *box, box_tf);		
		std::shared_ptr<fcl::CollisionObject> coll_obj = 
				std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
		collision_objects_start.push_back(coll_obj);	
	}
	
	transforms.clear();
	rave_robot_clone_->SetDOFValues(dof_values_goal);	
	rave_robot_clone_->GetLinkTransformations(transforms);
	std::vector<std::shared_ptr<fcl::CollisionObject>> collision_objects_goal;
	
	for (size_t i = 0; i < transforms.size(); i++) {		
		fcl::Quaternion3f quat(transforms[i].rot.x, transforms[i].rot.y, transforms[i].rot.z, transforms[i].rot.w);
		fcl::Vec3f trans_vec(transforms[i].trans.x, transforms[i].trans.y, transforms[i].trans.z);
				
		fcl::Transform3f trans(quat, trans_vec);
		fcl::AABB link_aabb(fcl::Vec3f(0, -aabbs[i].extents.y, -aabbs[i].extents.z), 
						    fcl::Vec3f(2.0 * aabbs[i].extents.x, aabbs[i].extents.y, aabbs[i].extents.z));
				
		fcl::Box* box = new fcl::Box();  
		fcl::Transform3f box_tf;		
		fcl::constructBox(link_aabb, trans, *box, box_tf);		
		std::shared_ptr<fcl::CollisionObject> coll_obj = 
				std::make_shared<fcl::CollisionObject>(boost::shared_ptr<fcl::CollisionGeometry>(box), box_tf);
		collision_objects_goal.push_back(coll_obj);	
	}
	CollisionReport collision_report = collision_manager_->inCollisionContinuousEnvironment(collision_objects_start,
                                                                                            collision_objects_goal);	
	collision_report.contact_body_name = links[collision_report.contact_body_index]->GetName();
	return collision_report;
	
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
	                        .def("propagate_constraints", &Robot::propagate_constraints)
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
	
	class_<CollisionReport>("CollisionReport", init<>())
		.def_readwrite("in_collision", &CollisionReport::in_collision)
	    .def_readwrite("time_of_contact", &CollisionReport::time_of_contact)
	    .def_readwrite("contact_body_index", &CollisionReport::contact_body_index)
	    .def_readwrite("contact_body_name", &CollisionReport::contact_body_name)
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
		.def("transformSensorToSensorLink", &Environment::transformSensorToSensorLink)		
		.def("updateRobotValues", &Environment::updateRobotValues)
		.def("initOctree", &Environment::initOctree)
		.def("drawBoxes", &Environment::drawBoxes)
		.def("setKinBodiesDefaultColor", &Environment::setKinBodiesDefaultColor)
		.def("setObstacleColor", &Environment::setObstacleColor)
		.def("getGoalArea", &Environment::getGoalArea)
		.def("plotPermanentParticles", &Environment::plotPermanentParticles)
		.def("removePermanentParticles", &Environment::removePermanentParticles)
		.def("getRobotDOFValues", &Environment::getRobotDOFValues)
		.def("setRobotDOFValues", &Environment::setRobotDOFValues)
		.def("setRobotTransform", &Environment::setRobotTransform)
		.def("robotCollidesDiscrete", &Environment::robotCollidesDiscrete)
		.def("robotCollidesContinuous", &Environment::robotCollidesContinuous)
	;
	
}

}