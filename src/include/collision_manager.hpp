#ifndef COLLISION_MANAGER_HPP_
#define COLLISION_MANAGER_HPP_
#include <openrave-core.h>
#include <openrave/environment.h>
#include <boost/python.hpp>
#include <boost/python/suite/indexing/vector_indexing_suite.hpp>
#include "fcl/broadphase/broadphase.h"
#include "fcl/broadphase/broadphase_dynamic_AABB_tree.h"
#include "fcl/collision_data.h"
#include "fcl/collision.h"
#include "fcl/continuous_collision.h"
#include <fcl/BVH/BVH_model.h>
#include <fcl/data_types.h>
#include <fcl/octree.h>

namespace shared {
    struct CollisionData {
        CollisionData()  {
            done = false;
        }  
        fcl::CollisionRequest request;  
        fcl::CollisionResult result;  
        bool done;
    };
    
    bool defaultCollisionFunction(fcl::CollisionObject* o1, fcl::CollisionObject* o2, void* cdata_);

    
    typedef fcl::BVHModel<fcl::OBBRSS> Model;

	class CollisionManager {
	public:
		CollisionManager();
		
		/**
		 * Set the environment collision checking is performed on
		 */
		void setEnvironment(OpenRAVE::EnvironmentBasePtr &env);
		
		/**
		 * Set the Octree representing the environment
		 */
		void setOctree(boost::shared_ptr<fcl::OcTree> &tree);
		
		/**
		 * Check of the robot collision objects collide with the environment 
		 */
		bool inCollisionDiscreteEnvironment(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects);
		
		/**
		 * Determines if the robot collides with the Octree
		 */
		bool inCollisionDiscreteOctree(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects);
		
		/**
		 * Check if a robot in motion collides with the environment
		 */
		bool inCollisionContinuousEnvironment(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
						                      std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal);
		
		/**
		 * Check if the robot continuously collides with the octree
		 */
		bool inCollisionContinuousOctree(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
								         std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal);
		
		/**
		 * A python wrapper for inCollisionDiscreteEnvironment
		 */
		bool inCollisionDiscreteEnvironmentPy(boost::python::list &ns);
		
		/**
		 * A python wrapper for inCollisionDiscreteOctree
		 */
		bool inCollisionDiscreteOctreePy(boost::python::list &ns);
		
		/**
		 * A python wrapper for inCollisionContinuous
		 */
		bool inCollisionContinuousEnvironmentPy(boost::python::list &ns);
		
		/**
		 * A python wrapper for inCollisionContinuousOctree
		 */
		bool inCollisionContinuousOctreePy(boost::python::list &ns);
		
		/**
		 * Triangulate the scene
		 */
		void triangulateScene();
		
	private:
		bool inCollisionDiscrete_(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects,
				                  boost::shared_ptr<fcl::CollisionObject> &eval_collision_object);
		
		bool inCollisionContinuous_(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
						            std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal,
						            boost::shared_ptr<fcl::CollisionObject> &eval_collision_object);
		
		boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> env_bvh_model_;
		
		boost::shared_ptr<fcl::CollisionObject> env_collision_object_;
		
		boost::shared_ptr<fcl::CollisionObject> octree_collision_object_;
		
		fcl::Transform3f identity_transform_;
		
		OpenRAVE::EnvironmentBasePtr env_;
		
		boost::shared_ptr<fcl::CollisionGeometry> octree_;
		
		fcl::BroadPhaseCollisionManager* obstacle_collision_manager_;
		
	};
}

#endif