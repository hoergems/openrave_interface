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
	//fcl::CollisionObject *obj = new fcl::CollisionObject(env_bvh_model_, tf);
}

bool CollisionManager::inCollisionDiscreteEnvironment(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {
	/**fcl::BroadPhaseCollisionManager* robot_collision_manager = new fcl::DynamicAABBTreeCollisionManager();
	for (auto &k: robot_collision_objects) {
		robot_collision_manager->registerObject(k.get());
	}
	robot_collision_manager->setup();
	CollisionData collision_data;
	obstacle_collision_manager_->collide(robot_collision_manager, &collision_data, shared::defaultCollisionFunction);
	return collision_data.result.isCollision();*/
	return inCollisionDiscrete_(robot_collision_objects, env_collision_object_);
}

bool CollisionManager::inCollisionDiscreteOctree(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {
	return inCollisionDiscrete_(robot_collision_objects, octree_collision_object_);
}

bool CollisionManager::inCollisionContinuousEnvironment(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
				                                        std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal) {
	return inCollisionContinuous_(robot_collision_object_start,
			                      robot_collision_object_goal,
			                      env_collision_object_);
}

bool CollisionManager::inCollisionContinuousOctree(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
								                   std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal) {
	return inCollisionContinuous_(robot_collision_object_start,
			                      robot_collision_object_goal,
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



bool CollisionManager::inCollisionContinuous_(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start,
		                                      std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal,
		                                      boost::shared_ptr<fcl::CollisionObject> &eval_collision_object) {
	fcl::ContinuousCollisionRequest request(10,
		    		                        0.0001,
		    		                        fcl::CCDM_LINEAR,
		    		                        fcl::GST_LIBCCD,
		    		                        fcl::CCDC_NAIVE);
	fcl::ContinuousCollisionResult result;    
	fcl::continuousCollide(robot_collision_object_start.get(), 
	                       robot_collision_object_goal->getTransform(), 
	                       eval_collision_object.get(),
	                       eval_collision_object->getTransform(),
	                       request,
	                       result);    
	return result.is_collide; 
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
	std::shared_ptr<fcl::CollisionObject> collision_object_start = 
	    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[0]);
	std::shared_ptr<fcl::CollisionObject> collision_object_goal = 
	    		boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[1]);    
	return inCollisionContinuousEnvironment(collision_object_start, collision_object_goal);
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
	std::shared_ptr<fcl::CollisionObject> collision_object_start = 
		    	boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[0]);
	std::shared_ptr<fcl::CollisionObject> collision_object_goal = 
		    	boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[1]);    
	return inCollisionContinuousOctree(collision_object_start, collision_object_goal);
}


}