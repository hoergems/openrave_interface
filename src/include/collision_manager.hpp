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
		
		void setEnvironment(OpenRAVE::EnvironmentBasePtr &env);
		
		/**
		 * Set the obstacles that make up the terrain
		 */
		//void setObstacles(std::vector<std::shared_ptr<Obstacle> > &obstacles);
		
		/**
		 * Python wrapper for setObstacles
		 */
		//void setObstaclesPy(boost::python::list &ns);
		
		/**
		 * Check of the robot collision objects collide with the environment 
		 */
		bool inCollisionDiscrete(std::vector<std::shared_ptr<fcl::CollisionObject>> &robot_collision_objects);
		
		/**
		 * A python wrapper for inCollisionDiscrete
		 */
		bool inCollisionDiscretePy(boost::python::list &ns);
		
		/**
		 * A python wrapper for inCollisionContinuous
		 */
		bool inCollisionContinuousPy(boost::python::list &ns);
		
		/**
		 * Check if a robot in motion collides with the environment
		 */
		bool inCollisionContinuous(std::shared_ptr<fcl::CollisionObject> &robot_collision_object_start, 
				std::shared_ptr<fcl::CollisionObject> &robot_collision_object_goal);
		
		
		
		/**
		 * Triangulate the scene
		 */
		void triangulateScene();
		
	private:
		boost::shared_ptr<fcl::BVHModel<fcl::OBBRSS>> env_bvh_model_;
		
		boost::shared_ptr<fcl::CollisionObject> env_collision_object_;
		
		fcl::Transform3f identity_transform_;
		
		OpenRAVE::EnvironmentBasePtr env_;
		
		fcl::BroadPhaseCollisionManager* obstacle_collision_manager_;
		
	};
}

#endif