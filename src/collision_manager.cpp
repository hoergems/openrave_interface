#include "include/collision_manager.hpp"
#include <boost/make_shared.hpp>

using std::cout;
using std::endl;

namespace shared {

bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_) {
    CollisionData* cdata = static_cast<CollisionData*>(cdata_);
    const fcl::CollisionRequest& request = cdata->request;
    fcl::CollisionResult& result = cdata->result;
    if(cdata->done) return true;
    fcl::collide(o1, o2, request, result);
    if(!request.enable_cost && (result.isCollision()) && (result.numContacts() >= request.num_max_contacts))
        cdata->done = true;
    return cdata->done;
}

CollisionManager::CollisionManager():
	env_(nullptr),
	env_bvh_model_(new fcl::BVHModel<fcl::OBBRSS>),
	env_collision_object_(nullptr),
	octree_(nullptr),
	octree_collision_object_(nullptr),
	identity_transform_(),
	obstacle_collision_manager_(new fcl::DynamicAABBTreeCollisionManager()){
	
}

void CollisionManager::setEnvironment(OpenRAVE::EnvironmentBasePtr &env) {
	env_ = env;
}

void CollisionManager::setOctree(boost::shared_ptr<fcl::OcTree> &tree) {
	octree_ = tree;
	octree_collision_object_ = boost::make_shared<fcl::CollisionObject>(octree_);
}

void CollisionManager::triangulateScene() {
	OpenRAVE::TriMesh trimesh;
	const std::string s = "";
	env_->TriangulateScene(trimesh, 
			               OpenRAVE::EnvironmentBase::SelectionOptions::SO_NoRobots,
			               s);	
	std::vector<fcl::Vec3f> vertices;
	std::vector<fcl::Triangle> triangles;
	for (size_t i = 0; i < trimesh.indices.size(); i+=3) {		
		fcl::Triangle triangle(trimesh.indices[i], 
				               trimesh.indices[i + 1],
				               trimesh.indices[i + 2]);		
		triangles.push_back(triangle);
		fcl::Vec3f vertice1(trimesh.vertices[i].x, trimesh.vertices[i].y, trimesh.vertices[i].z);
		fcl::Vec3f vertice2(trimesh.vertices[i + 1].x, trimesh.vertices[i + 1].y, trimesh.vertices[i + 1].z);
		fcl::Vec3f vertice3(trimesh.vertices[i + 2].x, trimesh.vertices[i + 2].y, trimesh.vertices[i + 2].z);
		vertices.push_back(vertice1);
		vertices.push_back(vertice2);
		vertices.push_back(vertice3);
	}	
	
	env_bvh_model_->beginModel();
	env_bvh_model_->addSubModel(vertices, triangles);
	env_bvh_model_->endModel();	
	env_collision_object_ = boost::make_shared<fcl::CollisionObject>(env_bvh_model_, identity_transform_);	
}

bool CollisionManager::inCollisionDiscreteEnvironment(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {	
	return inCollisionDiscrete_(robot_collision_objects, env_collision_object_);
}

bool CollisionManager::inCollisionDiscreteOctree(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {
	return inCollisionDiscrete_(robot_collision_objects, octree_collision_object_);
}

CollisionReport CollisionManager::inCollisionContinuousEnvironment(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_start, 
				                                                   std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_goal) {
	return inCollisionContinuous_(robot_collision_objects_start,
			                      robot_collision_objects_goal,
			                      env_collision_object_);
}

CollisionReport CollisionManager::inCollisionContinuousOctree(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_start, 
								                              std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_goal) {
	return inCollisionContinuous_(robot_collision_objects_start,
			                      robot_collision_objects_goal,
			                      octree_collision_object_);
}

bool CollisionManager::inCollisionDiscrete_(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects,
				                            boost::shared_ptr<fcl::CollisionObject> &eval_collision_object) {
	for (size_t i = 0; i < robot_collision_objects.size(); i++) {
		fcl::CollisionRequest request;		
		fcl::CollisionResult result;
		fcl::collide(robot_collision_objects[i].get(), 
					 eval_collision_object.get(),
				     request,
				     result);
		if (result.isCollision()) {			
			return true;
		}
	}
		
	return false;
}



CollisionReport CollisionManager::inCollisionContinuous_(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_start,
		                                                 std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects_goal,
		                                                 boost::shared_ptr<fcl::CollisionObject> &eval_collision_object) {	
	std::vector<fcl::CollisionObject *> robot_coll_objects_start;
	std::vector<fcl::Transform3f> robot_coll_objects_goal_transform;
	for (size_t i = 0; i < robot_collision_objects_start.size(); i++) {
		robot_coll_objects_start.push_back(robot_collision_objects_start[i].get());
		robot_coll_objects_goal_transform.push_back(robot_collision_objects_goal[i]->getTransform());
	}
	
	fcl::ContinuousCollisionRequest request(10,
											0.0001,
											fcl::CCDM_LINEAR,
											fcl::GST_LIBCCD,
											fcl::CCDC_NAIVE);
	fcl::ContinuousCollisionSetResult result;
	fcl::continuousCollide(robot_coll_objects_start, robot_coll_objects_goal_transform,
			               eval_collision_object.get(), eval_collision_object->getTransform(),
			               request,
			               result);
	CollisionReport collision_report;
	collision_report.in_collision = result.is_collide;
	collision_report.time_of_contact = result.time_of_contact;
	collision_report.contact_body_index = result.colliding_body_index;
	return collision_report;
	
	/**for (size_t i = 0; i < robot_collision_objects_start.size(); i++) {
		fcl::ContinuousCollisionRequest request(10,
												0.0001,
												fcl::CCDM_LINEAR,
												fcl::GST_LIBCCD,
												fcl::CCDC_NAIVE);
		fcl::ContinuousCollisionResult result;    
		fcl::continuousCollide(robot_collision_objects_start[i].get(), 
							   robot_collision_objects_goal[i]->getTransform(), 
							   eval_collision_object.get(),
							   eval_collision_object->getTransform(),
							   request,
							   result);
		if (result.is_collide) {
			return result.is_collide;
		}		
	}
	
	return false;*/
}

bool CollisionManager::inCollisionDiscreteEnvironmentPy(boost::python::list &ns) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
	for (int i = 0; i < len(ns); ++i)
	{
	    robot_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
	}
	
	return inCollisionDiscreteEnvironment(robot_collision_objects);
}

bool CollisionManager::inCollisionContinuousEnvironmentPy(boost::python::list &ns) {
	/**std::shared_ptr<fcl::CollisionObject> collision_object_start = 
	    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[0]);
	std::shared_ptr<fcl::CollisionObject> collision_object_goal = 
	    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[1]);    
	return inCollisionContinuousEnvironment(collision_object_start, collision_object_goal);*/
}

bool CollisionManager::inCollisionDiscreteOctreePy(boost::python::list &ns) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
	for (int i = 0; i < len(ns); ++i)
	{
	    robot_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
	}
		
	return inCollisionDiscreteOctree(robot_collision_objects);
}

bool CollisionManager::inCollisionContinuousOctreePy(boost::python::list &ns) {
	/**std::shared_ptr<fcl::CollisionObject> collision_object_start = 
		    	boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[0]);
	std::shared_ptr<fcl::CollisionObject> collision_object_goal = 
		    	boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[1]);    
	return inCollisionContinuousOctree(collision_object_start, collision_object_goal);*/
}


}