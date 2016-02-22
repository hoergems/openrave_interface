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
	identity_transform_(),
	obstacle_collision_manager_(new fcl::DynamicAABBTreeCollisionManager()){
	
}

void CollisionManager::setEnvironment(OpenRAVE::EnvironmentBasePtr &env) {
	env_ = env;
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

/**void CollisionManager::setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles) {
	for (auto &o: obstacles) {
		//obstacles_collision_objects_.push_back(o->getCollisionObject());
		obstacle_collision_manager_->registerObject(o->getCollisionObject().get());
	}
	obstacle_collision_manager_->setup();
}*/

/**void CollisionManager::setObstaclesPy(boost::python::list &ns) {
	std::vector<std::shared_ptr<Obstacle> > obstacles;
	for (size_t i = 0; i < len(ns); i++) {
	    std::shared_ptr<ObstacleWrapper> obst_wrapper = boost::python::extract<std::shared_ptr<ObstacleWrapper>>(ns[i]);
	    obstacles.push_back(std::static_pointer_cast<shared::Obstacle>(obst_wrapper));
	    //obstacles_.push_back(std::make_shared<Obstacle>(boost::python::extract<Obstacle>(ns[i])));
	}
	
	setObstacles(obstacles);
}*/

bool CollisionManager::inCollisionDiscrete(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects) {
	/**fcl::BroadPhaseCollisionManager* robot_collision_manager = new fcl::DynamicAABBTreeCollisionManager();
	for (auto &k: robot_collision_objects) {
		robot_collision_manager->registerObject(k.get());
	}
	robot_collision_manager->setup();
	CollisionData collision_data;
	obstacle_collision_manager_->collide(robot_collision_manager, &collision_data, shared::defaultCollisionFunction);
	return collision_data.result.isCollision();*/
	for (size_t i = 0; i < robot_collision_objects.size(); i++) {
		fcl::CollisionRequest request;		
		fcl::CollisionResult result;
		fcl::collide(robot_collision_objects[i].get(), 
				     env_collision_object_.get(),
					 request,
					 result);
		if (result.isCollision()) {			
			return true;
		}
	}
	
	return false;
}

bool CollisionManager::inCollisionDiscretePy(boost::python::list &ns) {
	std::vector<std::shared_ptr<fcl::CollisionObject>> robot_collision_objects;
	for (int i = 0; i < len(ns); ++i)
	{
	    robot_collision_objects.push_back(boost::python::extract<std::shared_ptr<fcl::CollisionObject>>(ns[i]));
	}
	
	return inCollisionDiscrete(robot_collision_objects);
}

bool CollisionManager::inCollisionContinuous(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start,
		std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal) {
	return false;
}

}