/*************************************************************************/
/*  space_bullet.cpp                                                     */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2022 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2022 Godot Engine contributors (cf. AUTHORS.md).   */
/*                                                                       */
/* Permission is hereby granted, free of charge, to any person obtaining */
/* a copy of this software and associated documentation files (the       */
/* "Software"), to deal in the Software without restriction, including   */
/* without limitation the rights to use, copy, modify, merge, publish,   */
/* distribute, sublicense, and/or sell copies of the Software, and to    */
/* permit persons to whom the Software is furnished to do so, subject to */
/* the following conditions:                                             */
/*                                                                       */
/* The above copyright notice and this permission notice shall be        */
/* included in all copies or substantial portions of the Software.       */
/*                                                                       */
/* THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND,       */
/* EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF    */
/* MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT.*/
/* IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY  */
/* CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT,  */
/* TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE     */
/* SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.                */
/*************************************************************************/

#include "space_bullet.h"

#include "bullet_physics_server.h"
#include "bullet_types_converter.h"
#include "bullet_utilities.h"
#include "constraint_bullet.h"
#include "core/math/quick_hull.h"
#include "core/project_settings.h"
#include "core/ustring.h"
#include "godot_collision_configuration.h"
#include "godot_collision_dispatcher.h"
#include "rigid_body_bullet.h"
#include "servers/physics_server.h"
#include "soft_body_bullet.h"

#include <BulletCollision/BroadphaseCollision/btBroadphaseProxy.h>
#include <BulletCollision/CollisionDispatch/btCollisionObject.h>
#include <BulletCollision/CollisionDispatch/btGhostObject.h>
#include <BulletCollision/CollisionShapes/btConvexPointCloudShape.h>
#include <BulletCollision/CollisionShapes/btConvexPolyhedron.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkEpaPenetrationDepthSolver.h>
#include <BulletCollision/NarrowPhaseCollision/btGjkPairDetector.h>
#include <BulletCollision/NarrowPhaseCollision/btPointCollector.h>
#include <BulletCollision/NarrowPhaseCollision/btRaycastCallback.h>
#include <BulletSoftBody/btSoftBodyRigidBodyCollisionConfiguration.h>
#include <BulletSoftBody/btSoftRigidDynamicsWorld.h>
#include <btBulletDynamicsCommon.h>

#include <assert.h>


#define USE_OLD_GODOT_BULLET_PHYSICS 1 // jit
//#pragma optimize("", off)
#define PLANE_EPSILON_CHECK 0.005

/**
	@author AndreaCatania
*/

BulletPhysicsDirectSpaceState::BulletPhysicsDirectSpaceState(SpaceBullet *p_space) :
		PhysicsDirectSpaceState(),
		space(p_space) {}

int BulletPhysicsDirectSpaceState::intersect_point(const Vector3 &p_point, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return 0;
	}

	btVector3 bt_point;
	G_TO_B(p_point, bt_point);

	btSphereShape sphere_point(0.001f);
	btCollisionObject collision_object_point;
	collision_object_point.setCollisionShape(&sphere_point);
	collision_object_point.setWorldTransform(btTransform(btQuaternion::getIdentity(), bt_point));

	// Setup query
	GodotAllContactResultCallback btResult(&collision_object_point, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;
	space->dynamicsWorld->contactTest(&collision_object_point, btResult);

	// The results is already populated by GodotAllConvexResultCallback
	return btResult.m_count;
}

bool BulletPhysicsDirectSpaceState::intersect_ray(const Vector3 &p_from, const Vector3 &p_to, RayResult &r_result, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, bool p_pick_ray) {
	btVector3 btVec_from;
	btVector3 btVec_to;

	G_TO_B(p_from, btVec_from);
	G_TO_B(p_to, btVec_to);

	// setup query
	GodotClosestRayResultCallback btResult(btVec_from, btVec_to, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;
	btResult.m_pickRay = p_pick_ray;
	btResult.m_flags |= btTriangleRaycastCallback::kF_UseGjkConvexCastRaytest;

	space->dynamicsWorld->rayTest(btVec_from, btVec_to, btResult);
	if (btResult.hasHit()) {
		B_TO_G(btResult.m_hitPointWorld, r_result.position);
		B_TO_G(btResult.m_hitNormalWorld.normalize(), r_result.normal);
		CollisionObjectBullet *gObj = static_cast<CollisionObjectBullet *>(btResult.m_collisionObject->getUserPointer());
		if (gObj) {
			r_result.shape = btResult.m_shapeId;
			r_result.rid = gObj->get_self();
			r_result.collider_id = gObj->get_instance_id();
			r_result.collider = 0 == r_result.collider_id ? nullptr : ObjectDB::get_instance(r_result.collider_id);
		} else {
			WARN_PRINT("The raycast performed has hit a collision object that is not part of Godot scene, please check it.");
		}
		return true;
	} else {
		return false;
	}
}

int BulletPhysicsDirectSpaceState::intersect_shape(const RID &p_shape, const Transform &p_xform, float p_margin, ShapeResult *r_results, int p_result_max, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return 0;
	}

	ShapeBullet *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);
	ERR_FAIL_COND_V(!shape, 0);

	btCollisionShape *btShape = shape->create_bt_shape(p_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINT("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return 0;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B(p_xform, bt_xform);
	UNSCALE_BT_BASIS(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotAllContactResultCallback btQuery(&collision_object, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	bulletdelete(btConvex);

	return btQuery.m_count;
}

bool BulletPhysicsDirectSpaceState::cast_motion(const RID &p_shape, const Transform &p_xform, const Vector3 &p_motion, float p_margin, float &r_closest_safe, float &r_closest_unsafe, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas, ShapeRestInfo *r_info) {
	r_closest_safe = 0.0f;
	r_closest_unsafe = 0.0f;

	btVector3 bt_motion;
	G_TO_B(p_motion, bt_motion);

	ShapeBullet *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	btCollisionShape *btShape = shape->create_bt_shape(p_xform.basis.get_scale(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINT("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return false;
	}
	btConvexShape *bt_convex_shape = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform_from;
	G_TO_B(p_xform, bt_xform_from);
	UNSCALE_BT_BASIS(bt_xform_from);

	btTransform bt_xform_to(bt_xform_from);
	bt_xform_to.getOrigin() += bt_motion;

	if ((bt_xform_to.getOrigin() - bt_xform_from.getOrigin()).fuzzyZero()) {
		r_closest_safe = 1.0f;
		r_closest_unsafe = 1.0f;
		bulletdelete(btShape);
		return true;
	}

	GodotClosestConvexResultCallback btResult(bt_xform_from.getOrigin(), bt_xform_to.getOrigin(), &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btResult.m_collisionFilterGroup = 0;
	btResult.m_collisionFilterMask = p_collision_mask;

	space->dynamicsWorld->convexSweepTest(bt_convex_shape, bt_xform_from, bt_xform_to, btResult, space->dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

	if (btResult.hasHit()) {
		const btScalar l = bt_motion.length();
		r_closest_unsafe = btResult.m_closestHitFraction;
		r_closest_safe = MAX(r_closest_unsafe - (1 - ((l - 0.01) / l)), 0);
		if (r_info) {
			if (btCollisionObject::CO_RIGID_BODY == btResult.m_hitCollisionObject->getInternalType()) {
				B_TO_G(static_cast<const btRigidBody *>(btResult.m_hitCollisionObject)->getVelocityInLocalPoint(btResult.m_hitPointWorld), r_info->linear_velocity);
			}
			CollisionObjectBullet *collision_object = static_cast<CollisionObjectBullet *>(btResult.m_hitCollisionObject->getUserPointer());
			B_TO_G(btResult.m_hitPointWorld, r_info->point);
			B_TO_G(btResult.m_hitNormalWorld, r_info->normal);
			r_info->rid = collision_object->get_self();
			r_info->collider_id = collision_object->get_instance_id();
			r_info->shape = btResult.m_shapeId;
		}
	} else {
		r_closest_safe = 1.0f;
		r_closest_unsafe = 1.0f;
	}

	bulletdelete(bt_convex_shape);
	return true; // Mean success
}

/// Returns the list of contacts pairs in this order: Local contact, other body contact
bool BulletPhysicsDirectSpaceState::collide_shape(RID p_shape, const Transform &p_shape_xform, float p_margin, Vector3 *r_results, int p_result_max, int &r_result_count, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	if (p_result_max <= 0) {
		return false;
	}

	ShapeBullet *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	btCollisionShape *btShape = shape->create_bt_shape(p_shape_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINT("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return false;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B(p_shape_xform, bt_xform);
	UNSCALE_BT_BASIS(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotContactPairContactResultCallback btQuery(&collision_object, r_results, p_result_max, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	r_result_count = btQuery.m_count;
	bulletdelete(btConvex);

	return btQuery.m_count;
}

bool BulletPhysicsDirectSpaceState::rest_info(RID p_shape, const Transform &p_shape_xform, float p_margin, ShapeRestInfo *r_info, const Set<RID> &p_exclude, uint32_t p_collision_mask, bool p_collide_with_bodies, bool p_collide_with_areas) {
	ShapeBullet *shape = space->get_physics_server()->get_shape_owner()->get(p_shape);
	ERR_FAIL_COND_V(!shape, false);

	btCollisionShape *btShape = shape->create_bt_shape(p_shape_xform.basis.get_scale_abs(), p_margin);
	if (!btShape->isConvex()) {
		bulletdelete(btShape);
		ERR_PRINT("The shape is not a convex shape, then is not supported: shape type: " + itos(shape->get_type()));
		return false;
	}
	btConvexShape *btConvex = static_cast<btConvexShape *>(btShape);

	btTransform bt_xform;
	G_TO_B(p_shape_xform, bt_xform);
	UNSCALE_BT_BASIS(bt_xform);

	btCollisionObject collision_object;
	collision_object.setCollisionShape(btConvex);
	collision_object.setWorldTransform(bt_xform);

	GodotRestInfoContactResultCallback btQuery(&collision_object, r_info, &p_exclude, p_collide_with_bodies, p_collide_with_areas);
	btQuery.m_collisionFilterGroup = 0;
	btQuery.m_collisionFilterMask = p_collision_mask;
	btQuery.m_closestDistanceThreshold = 0;
	space->dynamicsWorld->contactTest(&collision_object, btQuery);

	bulletdelete(btConvex);

	if (btQuery.m_collided) {
		if (btCollisionObject::CO_RIGID_BODY == btQuery.m_rest_info_collision_object->getInternalType()) {
			B_TO_G(static_cast<const btRigidBody *>(btQuery.m_rest_info_collision_object)->getVelocityInLocalPoint(btQuery.m_rest_info_bt_point), r_info->linear_velocity);
		}
		B_TO_G(btQuery.m_rest_info_bt_point, r_info->point);
	}

	return btQuery.m_collided;
}

Vector3 BulletPhysicsDirectSpaceState::get_closest_point_to_object_volume(RID p_object, const Vector3 p_point) const {
	RigidCollisionObjectBullet *rigid_object = space->get_physics_server()->get_rigid_collision_object(p_object);
	ERR_FAIL_COND_V(!rigid_object, Vector3());

	btVector3 out_closest_point(0, 0, 0);
	btScalar out_distance = 1e20;

	btVector3 bt_point;
	G_TO_B(p_point, bt_point);

	btSphereShape point_shape(0.);

	btCollisionShape *shape;
	btConvexShape *convex_shape;
	btTransform child_transform;
	btTransform body_transform(rigid_object->get_bt_collision_object()->getWorldTransform());

	btGjkPairDetector::ClosestPointInput input;
	input.m_transformA.getBasis().setIdentity();
	input.m_transformA.setOrigin(bt_point);

	bool shapes_found = false;

	for (int i = rigid_object->get_shape_count() - 1; 0 <= i; --i) {
		shape = rigid_object->get_bt_shape(i);
		if (shape->isConvex()) {
			child_transform = rigid_object->get_bt_shape_transform(i);
			convex_shape = static_cast<btConvexShape *>(shape);

			input.m_transformB = body_transform * child_transform;

			btPointCollector result;
			btGjkPairDetector gjk_pair_detector(&point_shape, convex_shape, space->gjk_simplex_solver, space->gjk_epa_pen_solver);
			gjk_pair_detector.getClosestPoints(input, result, nullptr);

			if (out_distance > result.m_distance) {
				out_distance = result.m_distance;
				out_closest_point = result.m_pointInWorld;
			}
		}
		shapes_found = true;
	}

	if (shapes_found) {
		Vector3 out;
		B_TO_G(out_closest_point, out);
		return out;
	} else {
		// no shapes found, use distance to origin.
		return rigid_object->get_transform().get_origin();
	}
}

SpaceBullet::SpaceBullet() :
		broadphase(nullptr),
		collisionConfiguration(nullptr),
		dispatcher(nullptr),
		solver(nullptr),
		dynamicsWorld(nullptr),
		soft_body_world_info(nullptr),
		ghostPairCallback(nullptr),
		godotFilterCallback(nullptr),
		gravityDirection(0, -1, 0),
		gravityMagnitude(10),
		linear_damp(0.0),
		angular_damp(0.0),
		contactDebugCount(0),
		delta_time(0.) {
	create_empty_world(GLOBAL_DEF("physics/3d/active_soft_world", true));
	direct_access = memnew(BulletPhysicsDirectSpaceState(this));
}

SpaceBullet::~SpaceBullet() {
	memdelete(direct_access);
	destroy_world();
}

void SpaceBullet::flush_queries() {
	const btCollisionObjectArray &colObjArray = dynamicsWorld->getCollisionObjectArray();
	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectBullet *>(colObjArray[i]->getUserPointer())->dispatch_callbacks();
	}
}

void SpaceBullet::step(real_t p_delta_time) {
	delta_time = p_delta_time;
	dynamicsWorld->stepSimulation(p_delta_time, 0, 0);
}

void SpaceBullet::set_param(PhysicsServer::AreaParameter p_param, const Variant &p_value) {
	assert(dynamicsWorld);

	switch (p_param) {
		case PhysicsServer::AREA_PARAM_GRAVITY:
			gravityMagnitude = p_value;
			update_gravity();
			break;
		case PhysicsServer::AREA_PARAM_GRAVITY_VECTOR:
			gravityDirection = p_value;
			update_gravity();
			break;
		case PhysicsServer::AREA_PARAM_LINEAR_DAMP:
			linear_damp = p_value;
			break;
		case PhysicsServer::AREA_PARAM_ANGULAR_DAMP:
			angular_damp = p_value;
			break;
		case PhysicsServer::AREA_PARAM_PRIORITY:
			// Priority is always 0, the lower
			break;
		case PhysicsServer::AREA_PARAM_GRAVITY_IS_POINT:
		case PhysicsServer::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
		case PhysicsServer::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
			break;
		default:
			WARN_PRINT("This set parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			break;
	}
}

Variant SpaceBullet::get_param(PhysicsServer::AreaParameter p_param) {
	switch (p_param) {
		case PhysicsServer::AREA_PARAM_GRAVITY:
			return gravityMagnitude;
		case PhysicsServer::AREA_PARAM_GRAVITY_VECTOR:
			return gravityDirection;
		case PhysicsServer::AREA_PARAM_LINEAR_DAMP:
			return linear_damp;
		case PhysicsServer::AREA_PARAM_ANGULAR_DAMP:
			return angular_damp;
		case PhysicsServer::AREA_PARAM_PRIORITY:
			return 0; // Priority is always 0, the lower
		case PhysicsServer::AREA_PARAM_GRAVITY_IS_POINT:
			return false;
		case PhysicsServer::AREA_PARAM_GRAVITY_DISTANCE_SCALE:
			return 0;
		case PhysicsServer::AREA_PARAM_GRAVITY_POINT_ATTENUATION:
			return 0;
		default:
			WARN_PRINT("This get parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			return Variant();
	}
}

void SpaceBullet::set_param(PhysicsServer::SpaceParameter p_param, real_t p_value) {
	switch (p_param) {
		case PhysicsServer::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default:
			WARN_PRINT("This set parameter (" + itos(p_param) + ") is ignored, the SpaceBullet doesn't support it.");
			break;
	}
}

real_t SpaceBullet::get_param(PhysicsServer::SpaceParameter p_param) {
	switch (p_param) {
		case PhysicsServer::SPACE_PARAM_CONTACT_RECYCLE_RADIUS:
		case PhysicsServer::SPACE_PARAM_CONTACT_MAX_SEPARATION:
		case PhysicsServer::SPACE_PARAM_BODY_MAX_ALLOWED_PENETRATION:
		case PhysicsServer::SPACE_PARAM_BODY_LINEAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_SLEEP_THRESHOLD:
		case PhysicsServer::SPACE_PARAM_BODY_TIME_TO_SLEEP:
		case PhysicsServer::SPACE_PARAM_BODY_ANGULAR_VELOCITY_DAMP_RATIO:
		case PhysicsServer::SPACE_PARAM_CONSTRAINT_DEFAULT_BIAS:
		default:
			WARN_PRINT("The SpaceBullet doesn't support this get parameter (" + itos(p_param) + "), 0 is returned.");
			return 0.f;
	}
}

void SpaceBullet::add_area(AreaBullet *p_area) {
	areas.push_back(p_area);
	dynamicsWorld->addCollisionObject(p_area->get_bt_ghost(), p_area->get_collision_layer(), p_area->get_collision_mask());
}

void SpaceBullet::remove_area(AreaBullet *p_area) {
	areas.erase(p_area);
	dynamicsWorld->removeCollisionObject(p_area->get_bt_ghost());
}

void SpaceBullet::reload_collision_filters(AreaBullet *p_area) {
	btGhostObject *ghost_object = p_area->get_bt_ghost();

	btBroadphaseProxy *ghost_proxy = ghost_object->getBroadphaseHandle();
	ghost_proxy->m_collisionFilterGroup = p_area->get_collision_layer();
	ghost_proxy->m_collisionFilterMask = p_area->get_collision_mask();

	dynamicsWorld->refreshBroadphaseProxy(ghost_object);
}

void SpaceBullet::add_rigid_body(RigidBodyBullet *p_body) {
	if (p_body->is_static()) {
		dynamicsWorld->addCollisionObject(p_body->get_bt_rigid_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
	} else {
		dynamicsWorld->addRigidBody(p_body->get_bt_rigid_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
		p_body->scratch_space_override_modificator();
	}
}

void SpaceBullet::remove_rigid_body_constraints(RigidBodyBullet *p_body) {
	btRigidBody *btBody = p_body->get_bt_rigid_body();

	int constraints = btBody->getNumConstraintRefs();
	if (constraints > 0) {
		ERR_PRINT("A body connected to joints was removed.");
		for (int i = 0; i < constraints; i++) {
			dynamicsWorld->removeConstraint(btBody->getConstraintRef(i));
		}
	}
}

void SpaceBullet::remove_rigid_body(RigidBodyBullet *p_body) {
	btRigidBody *btBody = p_body->get_bt_rigid_body();

	if (p_body->is_static()) {
		dynamicsWorld->removeCollisionObject(btBody);
	} else {
		dynamicsWorld->removeRigidBody(btBody);
	}
}

void SpaceBullet::reload_collision_filters(RigidBodyBullet *p_body) {
	btRigidBody *rigid_body = p_body->get_bt_rigid_body();

	btBroadphaseProxy *body_proxy = rigid_body->getBroadphaseProxy();
	body_proxy->m_collisionFilterGroup = p_body->get_collision_layer();
	body_proxy->m_collisionFilterMask = p_body->get_collision_mask();

	dynamicsWorld->refreshBroadphaseProxy(rigid_body);
}

void SpaceBullet::add_soft_body(SoftBodyBullet *p_body) {
	if (is_using_soft_world()) {
		if (p_body->get_bt_soft_body()) {
			p_body->get_bt_soft_body()->m_worldInfo = get_soft_body_world_info();
			static_cast<btSoftRigidDynamicsWorld *>(dynamicsWorld)->addSoftBody(p_body->get_bt_soft_body(), p_body->get_collision_layer(), p_body->get_collision_mask());
		}
	} else {
		ERR_PRINT("This soft body can't be added to non soft world");
	}
}

void SpaceBullet::remove_soft_body(SoftBodyBullet *p_body) {
	if (is_using_soft_world()) {
		if (p_body->get_bt_soft_body()) {
			static_cast<btSoftRigidDynamicsWorld *>(dynamicsWorld)->removeSoftBody(p_body->get_bt_soft_body());
			p_body->get_bt_soft_body()->m_worldInfo = nullptr;
		}
	}
}

void SpaceBullet::reload_collision_filters(SoftBodyBullet *p_body) {
	// This is necessary to change collision filter
	remove_soft_body(p_body);
	add_soft_body(p_body);
}

void SpaceBullet::add_constraint(ConstraintBullet *p_constraint, bool disableCollisionsBetweenLinkedBodies) {
	p_constraint->set_space(this);
	dynamicsWorld->addConstraint(p_constraint->get_bt_constraint(), disableCollisionsBetweenLinkedBodies);
}

void SpaceBullet::remove_constraint(ConstraintBullet *p_constraint) {
	dynamicsWorld->removeConstraint(p_constraint->get_bt_constraint());
}

int SpaceBullet::get_num_collision_objects() const {
	return dynamicsWorld->getNumCollisionObjects();
}

void SpaceBullet::remove_all_collision_objects() {
	for (int i = dynamicsWorld->getNumCollisionObjects() - 1; 0 <= i; --i) {
		btCollisionObject *btObj = dynamicsWorld->getCollisionObjectArray()[i];
		CollisionObjectBullet *colObj = static_cast<CollisionObjectBullet *>(btObj->getUserPointer());
		colObj->set_space(nullptr);
	}
}

void onBulletPreTickCallback(btDynamicsWorld *p_dynamicsWorld, btScalar timeStep) {
	static_cast<SpaceBullet *>(p_dynamicsWorld->getWorldUserInfo())->flush_queries();
}

void onBulletTickCallback(btDynamicsWorld *p_dynamicsWorld, btScalar timeStep) {
	const btCollisionObjectArray &colObjArray = p_dynamicsWorld->getCollisionObjectArray();

	// Notify all Collision objects the collision checker is started
	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectBullet *>(colObjArray[i]->getUserPointer())->on_collision_checker_start();
	}

	SpaceBullet *sb = static_cast<SpaceBullet *>(p_dynamicsWorld->getWorldUserInfo());
	sb->check_ghost_overlaps();
	sb->check_body_collision();

	for (int i = colObjArray.size() - 1; 0 <= i; --i) {
		static_cast<CollisionObjectBullet *>(colObjArray[i]->getUserPointer())->on_collision_checker_end();
	}
}

BulletPhysicsDirectSpaceState *SpaceBullet::get_direct_state() {
	return direct_access;
}

btScalar calculateGodotCombinedRestitution(const btCollisionObject *body0, const btCollisionObject *body1) {
	return CLAMP(body0->getRestitution() + body1->getRestitution(), 0, 1);
}

btScalar calculateGodotCombinedFriction(const btCollisionObject *body0, const btCollisionObject *body1) {
	return ABS(MIN(body0->getFriction(), body1->getFriction()));
}

void SpaceBullet::create_empty_world(bool p_create_soft_world) {
	gjk_epa_pen_solver = bulletnew(btGjkEpaPenetrationDepthSolver);
	gjk_simplex_solver = bulletnew(btVoronoiSimplexSolver);

	void *world_mem;
	if (p_create_soft_world) {
		world_mem = malloc(sizeof(btSoftRigidDynamicsWorld));
	} else {
		world_mem = malloc(sizeof(btDiscreteDynamicsWorld));
	}

	ERR_FAIL_COND_MSG(!world_mem, "Out of memory.");

	if (p_create_soft_world) {
		collisionConfiguration = bulletnew(GodotSoftCollisionConfiguration(static_cast<btDiscreteDynamicsWorld *>(world_mem)));
	} else {
		collisionConfiguration = bulletnew(GodotCollisionConfiguration(static_cast<btDiscreteDynamicsWorld *>(world_mem)));
	}

	dispatcher = bulletnew(GodotCollisionDispatcher(collisionConfiguration));
	broadphase = bulletnew(btDbvtBroadphase);
	solver = bulletnew(btSequentialImpulseConstraintSolver);

	if (p_create_soft_world) {
		dynamicsWorld = new (world_mem) btSoftRigidDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
		soft_body_world_info = bulletnew(btSoftBodyWorldInfo);
	} else {
		dynamicsWorld = new (world_mem) btDiscreteDynamicsWorld(dispatcher, broadphase, solver, collisionConfiguration);
	}

	ghostPairCallback = bulletnew(btGhostPairCallback);
	godotFilterCallback = bulletnew(GodotFilterCallback);
	gCalculateCombinedRestitutionCallback = &calculateGodotCombinedRestitution;
	gCalculateCombinedFrictionCallback = &calculateGodotCombinedFriction;
	gContactAddedCallback = &godotContactAddedCallback;

	dynamicsWorld->setWorldUserInfo(this);

	dynamicsWorld->setInternalTickCallback(onBulletPreTickCallback, this, true);
	dynamicsWorld->setInternalTickCallback(onBulletTickCallback, this, false);
	dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(ghostPairCallback); // Setup ghost check
	dynamicsWorld->getPairCache()->setOverlapFilterCallback(godotFilterCallback);

	if (soft_body_world_info) {
		soft_body_world_info->m_broadphase = broadphase;
		soft_body_world_info->m_dispatcher = dispatcher;
		soft_body_world_info->m_sparsesdf.Initialize();
	}

	update_gravity();
}

void SpaceBullet::destroy_world() {
	/// The world elements (like: Collision Objects, Constraints, Shapes) are managed by godot

	dynamicsWorld->getBroadphase()->getOverlappingPairCache()->setInternalGhostPairCallback(nullptr);
	dynamicsWorld->getPairCache()->setOverlapFilterCallback(nullptr);

	bulletdelete(ghostPairCallback);
	bulletdelete(godotFilterCallback);

	// Deallocate world
	dynamicsWorld->~btDiscreteDynamicsWorld();
	free(dynamicsWorld);
	dynamicsWorld = nullptr;

	bulletdelete(solver);
	bulletdelete(broadphase);
	bulletdelete(dispatcher);
	bulletdelete(collisionConfiguration);
	bulletdelete(soft_body_world_info);
	bulletdelete(gjk_simplex_solver);
	bulletdelete(gjk_epa_pen_solver);
}

void SpaceBullet::check_ghost_overlaps() {
	// For each area
	for (int area_idx = 0; area_idx < areas.size(); area_idx++) {
		AreaBullet *area = areas[area_idx];
		if (!area->is_monitoring()) {
			continue;
		}

		btGhostObject *bt_ghost = area->get_bt_ghost();
		const btTransform &area_transform = area->get_transform__bullet();
		const btVector3 &area_scale(area->get_bt_body_scale());

		// Mark all current overlapping shapes dirty.
		area->mark_all_overlaps_dirty();

		// Broadphase
		const btAlignedObjectArray<btCollisionObject *> overlapping_pairs = bt_ghost->getOverlappingPairs();
		// Narrowphase
		for (int pair_idx = 0; pair_idx < overlapping_pairs.size(); pair_idx++) {
			btCollisionObject *other_bt_collision_object = overlapping_pairs[pair_idx];
			RigidCollisionObjectBullet *other_object = static_cast<RigidCollisionObjectBullet *>(other_bt_collision_object->getUserPointer());
			const btTransform &other_transform = other_object->get_transform__bullet();
			const btVector3 &other_scale(other_object->get_bt_body_scale());

			if (!area->is_updated() && !other_object->is_updated()) {
				area->mark_object_overlaps_inside(other_object);
				continue;
			}

			if (other_bt_collision_object->getUserIndex() == CollisionObjectBullet::TYPE_AREA) {
				if (!static_cast<AreaBullet *>(other_bt_collision_object->getUserPointer())->is_monitorable()) {
					continue;
				}
			} else if (other_bt_collision_object->getUserIndex() != CollisionObjectBullet::TYPE_RIGID_BODY) {
				continue;
			}

			// For each area shape
			for (int our_shape_id = 0; our_shape_id < area->get_shape_count(); our_shape_id++) {
				btCollisionShape *area_shape = area->get_bt_shape(our_shape_id);
				if (!area_shape->isConvex()) {
					continue;
				}
				btConvexShape *area_convex_shape = static_cast<btConvexShape *>(area_shape);

				btTransform area_shape_transform(area->get_bt_shape_transform(our_shape_id));
				area_shape_transform.getOrigin() *= area_scale;
				btGjkPairDetector::ClosestPointInput gjk_input;
				gjk_input.m_transformA = area_transform * area_shape_transform;

				// For each other object shape
				for (int other_shape_id = 0; other_shape_id < other_object->get_shape_count(); other_shape_id++) {
					btCollisionShape *other_shape = other_object->get_bt_shape(other_shape_id);
					btTransform other_shape_transform(other_object->get_bt_shape_transform(other_shape_id));
					other_shape_transform.getOrigin() *= other_scale;
					gjk_input.m_transformB = other_transform * other_shape_transform;

					if (other_shape->isConvex()) {
						btPointCollector result;
						btGjkPairDetector gjk_pair_detector(
								area_convex_shape,
								static_cast<btConvexShape *>(other_shape),
								gjk_simplex_solver,
								gjk_epa_pen_solver);

						gjk_pair_detector.getClosestPoints(gjk_input, result, 0);
						if (result.m_distance <= 0) {
							area->set_overlap(other_object, other_shape_id, our_shape_id);
						}
					} else { // Other shape is not convex.
						btCollisionObjectWrapper obA(NULL, area_convex_shape, bt_ghost, gjk_input.m_transformA, -1, our_shape_id);
						btCollisionObjectWrapper obB(NULL, other_shape, other_bt_collision_object, gjk_input.m_transformB, -1, other_shape_id);
						btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, NULL, BT_CONTACT_POINT_ALGORITHMS);

						if (!algorithm) {
							continue;
						}

						GodotDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
						algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);
						algorithm->~btCollisionAlgorithm();
						dispatcher->freeCollisionAlgorithm(algorithm);

						if (contactPointResult.hasHit()) {
							area->set_overlap(other_object, our_shape_id, other_shape_id);
						}
					}
				} // End for each other object shape
			} // End for each area shape
		} // End for each overlapping pair

		// All overlapping shapes still marked dirty must have exited.
		area->mark_all_dirty_overlaps_as_exit();
	} // End for each area
}

void SpaceBullet::check_body_collision() {
#ifdef DEBUG_ENABLED
	reset_debug_contact_count();
#endif

	const int numManifolds = dynamicsWorld->getDispatcher()->getNumManifolds();
	for (int i = 0; i < numManifolds; ++i) {
		btPersistentManifold *contactManifold = dynamicsWorld->getDispatcher()->getManifoldByIndexInternal(i);

		// I know this static cast is a bit risky. But I'm checking its type just after it.
		// This allow me to avoid a lot of other cast and checks
		RigidBodyBullet *bodyA = static_cast<RigidBodyBullet *>(contactManifold->getBody0()->getUserPointer());
		RigidBodyBullet *bodyB = static_cast<RigidBodyBullet *>(contactManifold->getBody1()->getUserPointer());

		if (CollisionObjectBullet::TYPE_RIGID_BODY == bodyA->getType() && CollisionObjectBullet::TYPE_RIGID_BODY == bodyB->getType()) {
			if (!bodyA->can_add_collision() && !bodyB->can_add_collision()) {
				continue;
			}

			const int numContacts = contactManifold->getNumContacts();

			/// Since I don't need report all contacts for these objects,
			/// So report only the first
#define REPORT_ALL_CONTACTS 0
#if REPORT_ALL_CONTACTS
			for (int j = 0; j < numContacts; j++) {
				btManifoldPoint &pt = contactManifold->getContactPoint(j);
#else
			if (numContacts) {
				btManifoldPoint &pt = contactManifold->getContactPoint(0);
#endif
				if (
						pt.getDistance() < 0.0 ||
						bodyA->was_colliding(bodyB) ||
						bodyB->was_colliding(bodyA)) {
					Vector3 collisionWorldPosition;
					Vector3 collisionLocalPosition;
					Vector3 normalOnB;
					float appliedImpulse = pt.m_appliedImpulse;
					B_TO_G(pt.m_normalWorldOnB, normalOnB);

					// The pt.m_index only contains the shape index when more than one collision shape is used
					// and only if the collision shape is not a concave collision shape.
					// A value of -1 in pt.m_partId indicates the pt.m_index is a shape index.
					int shape_index_a = 0;
					if (bodyA->get_shape_count() > 1 && pt.m_partId0 == -1) {
						shape_index_a = pt.m_index0;
					}
					int shape_index_b = 0;
					if (bodyB->get_shape_count() > 1 && pt.m_partId1 == -1) {
						shape_index_b = pt.m_index1;
					}

					if (bodyA->can_add_collision()) {
						B_TO_G(pt.getPositionWorldOnB(), collisionWorldPosition);
						/// pt.m_localPointB Doesn't report the exact point in local space
						B_TO_G(pt.getPositionWorldOnB() - contactManifold->getBody1()->getWorldTransform().getOrigin(), collisionLocalPosition);
						bodyA->add_collision_object(bodyB, collisionWorldPosition, collisionLocalPosition, normalOnB, appliedImpulse, shape_index_b, shape_index_a);
					}
					if (bodyB->can_add_collision()) {
						B_TO_G(pt.getPositionWorldOnA(), collisionWorldPosition);
						/// pt.m_localPointA Doesn't report the exact point in local space
						B_TO_G(pt.getPositionWorldOnA() - contactManifold->getBody0()->getWorldTransform().getOrigin(), collisionLocalPosition);
						bodyB->add_collision_object(bodyA, collisionWorldPosition, collisionLocalPosition, normalOnB * -1, appliedImpulse * -1, shape_index_a, shape_index_b);
					}

#ifdef DEBUG_ENABLED
					if (is_debugging_contacts()) {
						add_debug_contact(collisionWorldPosition);
					}
#endif
				}
			}
		}
	}
}

void SpaceBullet::update_gravity() {
	btVector3 btGravity;
	G_TO_B(gravityDirection * gravityMagnitude, btGravity);
	//dynamicsWorld->setGravity(btGravity);
	dynamicsWorld->setGravity(btVector3(0, 0, 0));
	if (soft_body_world_info) {
		soft_body_world_info->m_gravity = btGravity;
	}
}

/// IMPORTANT: Please don't turn it ON this is not managed correctly!!
/// I'm leaving this here just for future tests.
/// Debug motion and normal vector drawing
#define debug_test_motion 0

#define RECOVERING_MOVEMENT_SCALE 0.4
#define RECOVERING_MOVEMENT_CYCLES 4

#if debug_test_motion

#include "scene/3d/immediate_geometry.h"

static ImmediateGeometry *motionVec(NULL);
static ImmediateGeometry *normalLine(NULL);
static Ref<SpatialMaterial> red_mat;
static Ref<SpatialMaterial> blue_mat;
#endif

#if USE_OLD_GODOT_BULLET_PHYSICS // Godot's bullet physics (with my tweaks)
bool SpaceBullet::test_body_motion(RigidBodyBullet *p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, PhysicsServer::MotionResult *r_result, bool p_exclude_raycast_shapes, const Set<RID> &p_exclude) {

	int test_thing = 0;

#ifdef BT_USE_DOUBLE_PRECISION
	test_thing = 2;
#else
	test_thing = 1;
#endif
#if debug_test_motion
	/// Yes I know this is not good, but I've used it as fast debugging hack.
	/// I'm leaving it here just for speedup the other eventual debugs
	if (!normalLine) {
		motionVec = memnew(ImmediateGeometry);
		normalLine = memnew(ImmediateGeometry);
		SceneTree::get_singleton()->get_current_scene()->add_child(motionVec);
		SceneTree::get_singleton()->get_current_scene()->add_child(normalLine);

		motionVec->set_as_toplevel(true);
		normalLine->set_as_toplevel(true);

		red_mat = Ref<SpatialMaterial>(memnew(SpatialMaterial));
		red_mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
		red_mat->set_line_width(20.0);
		red_mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
		red_mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		red_mat->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
		red_mat->set_albedo(Color(1, 0, 0, 1));
		motionVec->set_material_override(red_mat);

		blue_mat = Ref<SpatialMaterial>(memnew(SpatialMaterial));
		blue_mat->set_flag(SpatialMaterial::FLAG_UNSHADED, true);
		blue_mat->set_line_width(20.0);
		blue_mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
		blue_mat->set_flag(SpatialMaterial::FLAG_ALBEDO_FROM_VERTEX_COLOR, true);
		blue_mat->set_flag(SpatialMaterial::FLAG_SRGB_VERTEX_COLOR, true);
		blue_mat->set_albedo(Color(0, 0, 1, 1));
		normalLine->set_material_override(blue_mat);
	}
#endif

	btTransform body_transform;
	G_TO_B(p_from, body_transform);
	UNSCALE_BT_BASIS(body_transform);

	if (!p_body->get_kinematic_utilities()) {
		p_body->init_kinematic_utilities();
	}

	btVector3 initial_recover_motion(0, 0, 0);
	{ /// Phase one - multi shapes depenetration using margin
		for (int t(RECOVERING_MOVEMENT_CYCLES); 0 < t; --t) {
			if (!recover_from_penetration(p_body, body_transform, RECOVERING_MOVEMENT_SCALE, p_infinite_inertia, initial_recover_motion, nullptr, p_exclude)) {
				break;
			}
		}
		// Add recover movement in order to make it safe
		body_transform.getOrigin() += initial_recover_motion;
	}

	btVector3 motion;
	G_TO_B(p_motion, motion);
	real_t total_length = motion.length();
	bool has_collision = false;
	real_t unsafe_fraction = 1.0;
	real_t safe_fraction = 1.0;
	{
		// Phase two - sweep test, from a secure position without margin

		const int shape_count(p_body->get_shape_count());

#if debug_test_motion
		Vector3 sup_line;
		B_TO_G(body_safe_position.getOrigin(), sup_line);
		motionVec->clear();
		motionVec->begin(Mesh::PRIMITIVE_LINES, NULL);
		motionVec->add_vertex(sup_line);
		motionVec->add_vertex(sup_line + p_motion * 10);
		motionVec->end();
#endif

		for (int shIndex = 0; shIndex < shape_count; ++shIndex) {
			if (p_body->is_shape_disabled(shIndex)) {
				continue;
			}

			if (!p_body->get_bt_shape(shIndex)->isConvex()) {
				// Skip no convex shape
				continue;
			}

			if (p_exclude_raycast_shapes && p_body->get_bt_shape(shIndex)->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
				// Skip rayshape in order to implement custom separation process
				continue;
			}

			btConvexShape *convex_shape_test(static_cast<btConvexShape *>(p_body->get_bt_shape(shIndex)));

			btTransform shape_world_from = body_transform * p_body->get_kinematic_utilities()->shapes[shIndex].transform;

			btTransform shape_world_to(shape_world_from);
			shape_world_to.getOrigin() += motion;

			if ((shape_world_to.getOrigin() - shape_world_from.getOrigin()).fuzzyZero()) {
				motion = btVector3(0, 0, 0);
				break;
			}

			GodotKinClosestConvexResultCallback btResult(shape_world_from.getOrigin(), shape_world_to.getOrigin(), p_body, p_infinite_inertia, &p_exclude);
			btResult.m_collisionFilterGroup = p_body->get_collision_layer();
			btResult.m_collisionFilterMask = p_body->get_collision_mask();

			dynamicsWorld->convexSweepTest(convex_shape_test, shape_world_from, shape_world_to, btResult, dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration);

			if (btResult.hasHit()) {
				if (total_length > CMP_EPSILON) {
					real_t hit_fraction = btResult.m_closestHitFraction * motion.length() / total_length;
					if (hit_fraction < unsafe_fraction) {
						unsafe_fraction = hit_fraction;
						real_t margin = p_body->get_kinematic_utilities()->safe_margin;
						safe_fraction = MAX(hit_fraction - (1 - ((total_length - margin) / total_length)), 0);
					}
				}

				/// Since for each sweep test I fix the motion of new shapes in base the recover result,
				/// if another shape will hit something it means that has a deepest penetration respect the previous shape
				motion *= btResult.m_closestHitFraction;
				/// jitspoe - fix case where collision happens but we don't get any results returned.
				has_collision = true;

				if (r_result) {
					const btRigidBody *btRigid = static_cast<const btRigidBody *>(btResult.m_hitCollisionObject);
					CollisionObjectBullet *collisionObject = static_cast<CollisionObjectBullet *>(btRigid->getUserPointer());

					B_TO_G(motion, r_result->remainder); // is the remaining movements
					r_result->remainder = p_motion - r_result->remainder;

					B_TO_G(btResult.m_hitPointWorld, r_result->collision_point);
					B_TO_G(btResult.m_hitNormalWorld, r_result->collision_normal);
					//B_TO_G(btRigid->getVelocityInLocalPoint(r_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity); // It calculates velocity at point and assign it using special function Bullet_to_Godot
					r_result->collider = collisionObject->get_self();
					r_result->collider_id = collisionObject->get_instance_id();
					//r_result->collider_shape = r_recover_result.other_compound_shape_index;
					//r_result->collision_local_shape = r_recover_result.local_shape_most_recovered;
				}
			}
		}

		body_transform.getOrigin() += motion;
	}

	bool has_penetration = false;

	{ /// Phase three - contact test with margin

		btVector3 __rec(0, 0, 0);
		RecoverResult r_recover_result;

		has_penetration = recover_from_penetration(p_body, body_transform, 1, p_infinite_inertia, __rec, &r_recover_result, p_exclude);

		// Parse results
		if (r_result) {
			B_TO_G(motion + initial_recover_motion + __rec, r_result->motion);

			if (has_penetration) {
				const btRigidBody *btRigid = static_cast<const btRigidBody *>(r_recover_result.other_collision_object);
				CollisionObjectBullet *collisionObject = static_cast<CollisionObjectBullet *>(btRigid->getUserPointer());

				B_TO_G(motion, r_result->remainder); // is the remaining movements
				r_result->remainder = p_motion - r_result->remainder;

				B_TO_G(r_recover_result.pointWorld, r_result->collision_point);
				B_TO_G(r_recover_result.normal, r_result->collision_normal);
				B_TO_G(btRigid->getVelocityInLocalPoint(r_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity); // It calculates velocity at point and assign it using special function Bullet_to_Godot
				r_result->collider = collisionObject->get_self();
				r_result->collider_id = collisionObject->get_instance_id();
				r_result->collider_shape = r_recover_result.other_compound_shape_index;
				r_result->collision_local_shape = r_recover_result.local_shape_most_recovered;
				r_result->collision_depth = Math::abs(r_recover_result.penetration_distance);
				r_result->collision_safe_fraction = safe_fraction;
				r_result->collision_unsafe_fraction = unsafe_fraction;

#if debug_test_motion
				Vector3 sup_line2;
				B_TO_G(motion, sup_line2);
				normalLine->clear();
				normalLine->begin(Mesh::PRIMITIVE_LINES, NULL);
				normalLine->add_vertex(r_result->collision_point);
				normalLine->add_vertex(r_result->collision_point + r_result->collision_normal * 10);
				normalLine->end();
#endif
			} else if (!has_collision) {
				r_result->remainder = Vector3();
			}
		}
	}

	return has_penetration || has_collision;
}
#else //  !USE_OLD_GODOT_BULLET_PHYSICS // No margin / Quake-style collision
bool SpaceBullet::test_body_motion(RigidBodyBullet *p_body, const Transform &p_from, const Vector3 &p_motion, bool p_infinite_inertia, PhysicsServer::MotionResult *r_result, bool p_exclude_raycast_shapes) {

	btTransform body_transform;
	G_TO_B(p_from, body_transform);
	UNSCALE_BT_BASIS(body_transform);
	btTransform start_transform(body_transform);
	btVector3 motion;
	G_TO_B(p_motion, motion);
	bool has_collision = false;
	bool needs_iteration = true;
	bool needs_unstuck = false;
	int iterations_left = 3; // Max of 3 iterations
	real_t unstuck_margin = 0.001; // TODO: use margin settings?
	btVector3 unstuck_offset(0.0, 0.0, 0.0);
	real_t allowed_penetration = 0.0; // dynamicsWorld->getDispatchInfo().m_allowedCcdPenetration

next_iteration:
	// Do a sweep.  If something hits without movement, back out along normal and try again.
	while (needs_iteration) {
		needs_iteration = false;
		const int shape_count(p_body->get_shape_count());

		for (int shIndex = 0; shIndex < shape_count; ++shIndex) {
			if (p_body->is_shape_disabled(shIndex)) {
				continue;
			}

			if (!p_body->get_bt_shape(shIndex)->isConvex()) {
				// Skip no convex shape
				continue;
			}

			if (p_exclude_raycast_shapes && p_body->get_bt_shape(shIndex)->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
				// Skip rayshape in order to implement custom separation process
				continue;
			}

			btConvexShape *convex_shape_test(static_cast<btConvexShape *>(p_body->get_bt_shape(shIndex)));

			if (needs_unstuck) {
				// I thought maybe if we moved OUT of collision, it wouldn't have an immediate hit, but apparently it does, so we'll have to unsafely back out :|
#if 1
				btTransform shape_unstuck_from = start_transform * p_body->get_kinematic_utilities()->shapes[shIndex].transform;
				btTransform shape_unstuck_to(shape_unstuck_from);
				shape_unstuck_to.getOrigin() += unstuck_offset;
				GodotKinClosestConvexResultCallback btResult(shape_unstuck_from.getOrigin(), shape_unstuck_to.getOrigin(), p_body, p_infinite_inertia);
				btResult.m_collisionFilterGroup = p_body->get_collision_layer();
				btResult.m_collisionFilterMask = p_body->get_collision_mask();
				convex_sweep_test(dynamicsWorld, convex_shape_test, shape_unstuck_from, shape_unstuck_to, btResult, allowed_penetration);
				body_transform.getOrigin() = start_transform.getOrigin() + btResult.m_closestHitFraction * unstuck_offset;
#else
				//shape_world_from.getOrigin() += unstuck_offset;
				body_transform.getOrigin() += unstuck_offset;
#endif
			}

			btTransform shape_world_from = body_transform * p_body->get_kinematic_utilities()->shapes[shIndex].transform;
			btTransform shape_world_to(shape_world_from);
			shape_world_to.getOrigin() += motion;
			GodotKinClosestConvexResultCallback btResult(shape_world_from.getOrigin(), shape_world_to.getOrigin(), p_body, p_infinite_inertia);
			btResult.m_collisionFilterGroup = p_body->get_collision_layer();
			btResult.m_collisionFilterMask = p_body->get_collision_mask();

			convex_sweep_test(dynamicsWorld, convex_shape_test, shape_world_from, shape_world_to, btResult, allowed_penetration);

			if (btResult.hasHit()) {
				// If we get stuck immediately moving close to parallel to a surface, back up a little bit and try again.
				if (btResult.m_closestHitFraction == 0.0 && iterations_left > 0 && motion.normalized().dot(btResult.m_hitNormalWorld) > -0.01) {
					// Stuck immediately.  Try to move out a bit.
					--iterations_left;
					needs_iteration = true;
					needs_unstuck = true;
					unstuck_offset = btResult.m_hitNormalWorld * unstuck_margin;
					goto next_iteration;
				} else {
					/// Since for each sweep test I fix the motion of new shapes in base the recover result,
					/// if another shape will hit something it means that has a deepest penetration respect the previous shape
					motion *= btResult.m_closestHitFraction;
					/// jitspoe - fix case where collision happens but we don't get any results returned.
					has_collision = true;

					if (r_result) {
						const btRigidBody *btRigid = static_cast<const btRigidBody *>(btResult.m_hitCollisionObject);
						CollisionObjectBullet *collisionObject = static_cast<CollisionObjectBullet *>(btRigid->getUserPointer());

						B_TO_G(motion, r_result->remainder); // is the remaining movements
						r_result->remainder = p_motion - r_result->remainder;

						B_TO_G(btResult.m_hitPointWorld, r_result->collision_point);
						B_TO_G(btResult.m_hitNormalWorld, r_result->collision_normal);
						//B_TO_G(btRigid->getVelocityInLocalPoint(r_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity); // It calculates velocity at point and assign it using special function Bullet_to_Godot
						r_result->collider = collisionObject->get_self();
						r_result->collider_id = collisionObject->get_instance_id();
						//r_result->collider_shape = r_recover_result.other_compound_shape_index;
						//r_result->collision_local_shape = r_recover_result.local_shape_most_recovered;
					}
				}
			}
			// TODO: Move back by unstuck offset.
		}

		body_transform.getOrigin() += motion;
		//next_iteration:
	}

	if (needs_unstuck) { // Move back by unstuck amount to stop stuff from floating.
		btTransform correct_unstuck_to(body_transform);
		correct_unstuck_to.getOrigin() -= unstuck_offset;
		GodotKinClosestConvexResultCallback btResult(body_transform.getOrigin(), correct_unstuck_to.getOrigin(), p_body, p_infinite_inertia);
		btResult.m_collisionFilterGroup = p_body->get_collision_layer();
		btResult.m_collisionFilterMask = p_body->get_collision_mask();
		int shIndex = 0; // TODO: Handle multiple shapes in one object.
		btConvexShape *convex_shape_test(static_cast<btConvexShape *>(p_body->get_bt_shape(shIndex)));
		convex_sweep_test(dynamicsWorld, convex_shape_test, body_transform, correct_unstuck_to, btResult, allowed_penetration);
		body_transform.getOrigin() -= btResult.m_closestHitFraction * unstuck_offset;
	}

	if (r_result) {

		if (!has_collision) {
			r_result->remainder = Vector3();
		}

		motion = body_transform.getOrigin() - start_transform.getOrigin();
		B_TO_G(motion, r_result->motion);
	}

	return has_collision;
}
#endif

struct btSingleSweepCallback : public btBroadphaseRayCallback {
	btTransform m_convexFromTrans;
	btTransform m_convexToTrans;
	btVector3 m_hitNormal;
	const btCollisionWorld *m_world;
	btCollisionWorld::ConvexResultCallback &m_resultCallback;
	btScalar m_allowedCcdPenetration;
	const btConvexShape *m_castShape;

	btSingleSweepCallback(const btConvexShape *castShape, const btTransform &convexFromTrans, const btTransform &convexToTrans, const btCollisionWorld *world, btCollisionWorld::ConvexResultCallback &resultCallback, btScalar allowedPenetration) :
			m_convexFromTrans(convexFromTrans),
			m_convexToTrans(convexToTrans),
			m_world(world),
			m_resultCallback(resultCallback),
			m_allowedCcdPenetration(allowedPenetration),
			m_castShape(castShape) {
		btVector3 unnormalizedRayDir = (m_convexToTrans.getOrigin() - m_convexFromTrans.getOrigin());
		btVector3 rayDir = unnormalizedRayDir.normalized();
		///what about division by zero? --> just set rayDirection[i] to INF/BT_LARGE_FLOAT
		m_rayDirectionInverse[0] = rayDir[0] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[0];
		m_rayDirectionInverse[1] = rayDir[1] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[1];
		m_rayDirectionInverse[2] = rayDir[2] == btScalar(0.0) ? btScalar(BT_LARGE_FLOAT) : btScalar(1.0) / rayDir[2];
		m_signs[0] = m_rayDirectionInverse[0] < 0.0;
		m_signs[1] = m_rayDirectionInverse[1] < 0.0;
		m_signs[2] = m_rayDirectionInverse[2] < 0.0;

		m_lambda_max = rayDir.dot(unnormalizedRayDir);
	}

	inline double get_plane_dist(btVector4 plane) { // TODO: Precalculate and cache this?
		double dist = /*-*/plane[3]; // Not sure why these are stored as negative.
		btBoxShape *box_shape = nullptr;
		if (m_castShape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
			box_shape = (btBoxShape*)m_castShape;
		}/*
		if (m_castShape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
			btCompoundShape *compound_shape = (btCompoundShape *)m_castShape;
			btCollisionShape *child_shape = compound_shape->getChildShape(0);
			if (child_shape)
			if ()
		}*/
		if (box_shape) {
			btVector3 half_extents = box_shape->getHalfExtentsWithoutMargin();
			dist += abs(plane[0] * half_extents[0]);
			dist += abs(plane[1] * half_extents[1]);
			dist += abs(plane[2] * half_extents[2]);
		}

		return dist;
	}

	bool process_point_cloud(btCollisionObject* collision_object, const btVector3* points, int num_cloud_points, btTransform world_transform, const btVector3 relative_start, const btVector3 relative_end) {
		//btAlignedObjectArray<btVector3> sum_points;
		int num_caster_points = 1;
		//btAlignedObjectArray<btVector3> caster_points;
		Vector<Vector3> caster_points;
		if (m_castShape->getShapeType() == BOX_SHAPE_PROXYTYPE) {
			num_caster_points = 8;
			btBoxShape* box_shape = (btBoxShape*)m_castShape;
			btVector3 half_extents = box_shape->getHalfExtentsWithoutMargin();
			//caster_points.reserve(8);
			// TODO: Reserve with godot style vectors
			// TODO: Apply rotation.

			caster_points.push_back(Vector3(half_extents[0], half_extents[1], half_extents[2]));
			caster_points.push_back(Vector3(half_extents[0], half_extents[1], -half_extents[2]));
			caster_points.push_back(Vector3(half_extents[0], -half_extents[1], half_extents[2]));
			caster_points.push_back(Vector3(half_extents[0], -half_extents[1], -half_extents[2]));
			caster_points.push_back(Vector3(-half_extents[0], half_extents[1], half_extents[2]));
			caster_points.push_back(Vector3(-half_extents[0], half_extents[1], -half_extents[2]));
			caster_points.push_back(Vector3(-half_extents[0], -half_extents[1], half_extents[2]));
			caster_points.push_back(Vector3(-half_extents[0], -half_extents[1], -half_extents[2]));
		}
		else {
			caster_points.push_back(Vector3(0.0, 0.0, 0.0));
		}

		int num_points = num_cloud_points * num_caster_points;
		Vector<Vector3> sum_points;
		sum_points.resize(num_points);
		//sum_points.reserve(num_points);

		// Create Minkowski sum.
		// For every vertex in the potential collision object, add every vertex in the moving object
		for (int i = 0; i < num_cloud_points; ++i) {
			btVector3 world_space_vertex = world_transform.getBasis() * points[i];
			Vector3 world_space_vertex_g(world_space_vertex[0], world_space_vertex[1], world_space_vertex[2]);
			for (int j = 0; j < num_caster_points; ++j) {
				//sum_points.push_back(world_space_vertex + caster_points[j]);
				sum_points.write[i * num_caster_points + j] = world_space_vertex_g + caster_points[j];
			}
		}
		//btAlignedObjectArray<btVector4> planes;

		Geometry::MeshData md;

		Vector<Vector3> varr;
		Error err = QuickHull::build(sum_points, md);
		// todo: This is super slow.  Try something else.  quickhull?
		// Convert these into planes.
		/*for (int i_0 = 0; i_0 < num_points; ++i_0) {
			for (int i_1 = i_0 + 1; i_1 < num_points; ++i_1) {
				for (int i_2 = i_1 + 1; i_2 < num_points; ++i_2) {
					btVector3 p0 = sum_points[i_0];
					btVector3 p1 = sum_points[i_1];
					btVector3 p2 = sum_points[i_2];
					btVector3 plane_normal = (p1 - p0).cross(p2 - p0);
					if (plane_normal.length2() < CMP_EPSILON) {
						continue; // Bad plane -- 3 points in a row
					}
					plane_normal.normalize();
					btScalar plane_dist = p0.dot(plane_normal);

					int points_under = 0;
					int points_over = 0;
					for (int i_other = 0; i_other < num_points; ++i_other) {
						// Not sure if it's worth checking that i_other != i_0 through i_2.
						btVector3 other_point = sum_points[i_other];
						btScalar d = other_point.dot(plane_normal);
						if (d - plane_dist < -CMP_EPSILON) {
							++points_under;
						}
						else if (d - plane_dist > CMP_EPSILON) {
							++points_over;
						}
					}
					if (points_over > 0 && points_under > 0) { // Bad plane. Ignore
						continue;
					}
					if (points_over > 0) {
						// Need to invert the plane.
						plane_normal = -plane_normal;
						plane_dist = -plane_dist;
					}
					btVector4 plane;
					plane[0] = plane_normal[0];
					plane[1] = plane_normal[1];
					plane[2] = plane_normal[2];
					plane[3] = plane_dist;
					bool exists = false;
					for (int i_plane = 0; i_plane < planes.size(); ++i_plane) {
						// Make sure we don't have this plane already.
						if (planes[i_plane].distance2(plane) < CMP_EPSILON2) {
							exists = true;
							break;
						}
					}
					if (!exists) {
						planes.push_back(plane);
					}
				}
			}
		}*/
		// Loop through all the planes and check for colloisions.
		int num_planes = md.faces.size();//planes.size();
		Vector3 relative_start_g(relative_start[0], relative_start[1], relative_start[2]);
		Vector3 relative_end_g(relative_end[0], relative_end[1], relative_end[2]);

		// Start solid check:
		bool start_solid = true;

		for (int i = 0; i < num_planes; ++i) {
			Vector3 normal = md.faces[i].plane.normal;
			double start_height = normal.dot(relative_start_g);
			if (start_height >= 0.0) {
				start_solid = false;
				break;
			}
		}

		for (int i = 0; i < num_planes; ++i) {
			//btVector4 plane = planes[i];
			//btVector3 normal = plane;
			//normal[3] = 0.0; // not sure if this is needed, but I don't trust a vector4 to vector3 assignment to clear out the w component.
			Vector3 normal = md.faces[i].plane.normal;
			//normal = world_transform.getBasis() * normal; // TODO: This isn't correct.  Need to apply to the verts themselves. (already done)
			// check if we cross the plane.  If we do, check if we're inside all the other planes.  That, my friends, is a collision!
			// is the point outside of the plane?
			//double plane_dist = get_plane_dist(plane);
			//double plane_dist = plane[3];
			double plane_dist = md.faces[i].plane.d;
			double start_height = normal.dot(relative_start_g);
			double start_from_plane = start_height - plane_dist;

			//if (start_dist_from_plane >= -0.004) { // may need an epsilon check here.  TODO: Add player extents here.
			if (true) { //(start_height > 0.0) { // TODO: Add player extents.  Might not be good if collision is offset.
				double end_height = normal.dot(relative_end_g);
				// is the target point INSIDE the plane.
				if (end_height < start_height) { // Only collide if we're moving TOWARD the plane
					double end_from_plane = end_height - plane_dist;
					if (end_from_plane < 0.0) { // Are we past the plane?
						double movement_along_normal = start_height - end_height;
						double fraction = start_from_plane / movement_along_normal;
						if (fraction < 0.0) {
							fraction = 0.0;
						}
						btVector3 movement_vector = relative_end - relative_start;

						// Get potential collision point.
						btVector3 local_impact_point = relative_start + movement_vector * fraction;
						Vector3 local_impact_point_g;
						B_TO_G(local_impact_point, local_impact_point_g);
						// Check all other planes to see if this is a collision point
						bool inside_other_planes = true;
						for (int j = 0; j < num_planes; ++j) {
							if (j != i) {
								//btVector4 other_plane = planes[j];
								//btVector3 other_normal = other_plane;
								//other_normal[3] = 0;
								Vector3 other_normal = md.faces[j].plane.normal;
								//other_normal = world_transform.getBasis() * other_normal;
								//if (other_normal.dot(local_impact_point_g) > get_plane_dist(btVector4(other_normal[0], other_normal[1], other_normal[2], other_plane[3])) - 0.005) { // TODO: Add caster extents.
								double dot = other_normal.dot(local_impact_point_g);
								//double plane_dist = get_plane_dist(btVector4(other_normal[0], other_normal[1], other_normal[2], md.faces[j].plane.d)) - 0.005; // This is for if we don't do the Minkowski sum.
								double plane_dist = md.faces[j].plane.d - PLANE_EPSILON_CHECK;
								if (dot > plane_dist) { // TODO: Add caster extents.
									inside_other_planes = false;
									break;
								}
							}
						}

						if (inside_other_planes) { // legit collision
							btVector3 normal_b;
							G_TO_B(normal, normal_b);
							btVector3 world_impact_point = local_impact_point + world_transform.getOrigin() - normal_b * get_plane_dist(btVector4(normal[0], normal[1], normal[2], 0.0));
							if (fraction < m_resultCallback.m_closestHitFraction) {
								btCollisionWorld::LocalConvexResult localConvexResult(
									collision_object,
									0,
									normal_b,
									world_impact_point,
									fraction);

								bool normalInWorldSpace = true;
								m_resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
								return true;
							}
						}
					}
				}
			}
		}
		// TODO: Make this generic between box and convex shapes.
		// TODO: Finish this.
		return false;
	}

	bool process_shape(btCollisionObject *collision_object, btCollisionShape *collision_shape, btTransform world_transform) {

		if (collision_shape->getShapeType() == COMPOUND_SHAPE_PROXYTYPE) {
			btCompoundShape *compound_shape = (btCompoundShape *)collision_shape;
			int num_shapes = compound_shape->getNumChildShapes();
			auto list = compound_shape->getChildList();
			for (int i = 0; i < num_shapes; ++i) {
				process_shape(collision_object, list[i].m_childShape, world_transform * compound_shape->getChildTransform(i)); // Is this the correct matrix order?
			}
		}
		else {
			btVector3 relative_start = m_convexFromTrans.getOrigin() - world_transform.getOrigin();
			btVector3 relative_end = m_convexToTrans.getOrigin() - world_transform.getOrigin();

			if (collision_shape->getShapeType() == BOX_SHAPE_PROXYTYPE) { // Just worry about box shapes for now.  Planes don't work for the other stuff.
				//auto world_transform = collision_object->getWorldTransform() ;
				btBoxShape *box_shape = (btBoxShape *)collision_shape;
				//box_shape->getPlane(plane_normal, plane_support, i);
				/*int num_planes = box_shape->getNumPlanes(); // this should always be 6, but just to be safe.
				for (int i = 0; i < num_planes; ++i)
				{
					btVector3 plane_normal;
					btVector3 plane_support; // unused
					box_shape->getPlane(plane_normal, plane_support, i);

				}*/

				for (int i = 0; i < 6; ++i) {
					btVector4 plane;
					btVector3 normal;
					box_shape->getPlaneEquation(plane, i); // return normal + offset.
					plane[3] = -plane[3]; // Not sure why this is negative?
					normal = plane;
					normal[3] = 0;
					normal = world_transform.getBasis() * normal;
					// check if we cross the plane.  If we do, check if we're inside all the other planes.  That, my friends, is a collision!
					// is the point outside of the plane?
					double plane_dist = get_plane_dist(btVector4(normal[0], normal[1], normal[2], plane[3]));
					double start_height = normal.dot(relative_start);
					double start_from_plane = start_height - plane_dist;

					//if (start_dist_from_plane >= -0.004) { // may need an epsilon check here.  TODO: Add player extents here.
					if (true) { //(start_height > 0.0) { // TODO: Add player extents.  Might not be good if collision is offset.
						double end_height = normal.dot(relative_end);
						// is the target point INSIDE the plane.
						if (end_height < start_height) { // Only collide if we're moving TOWARD the plane
							double end_from_plane = end_height - plane_dist;
							if (end_from_plane < 0.0) { // Are we past the plane?
								double movement_along_normal = start_height - end_height;
								double fraction = start_from_plane / movement_along_normal;
								if (fraction < 0.0) {
									fraction = 0.0;
								}
								btVector3 movement_vector = relative_end - relative_start;

								// Get potential collision point.
								btVector3 local_impact_point = relative_start + movement_vector * fraction;
								// Check all other planes to see if this is a collision point
								bool inside_other_planes = true;
								for (int j = 0; j < 6; ++j) {
									if (j != i) {
										btVector4 other_plane;
										box_shape->getPlaneEquation(other_plane, j);
										other_plane[3] = -other_plane[3]; // Not sure why this is negative.
										btVector3 other_normal = other_plane;
										other_normal[3] = 0.0;
										other_normal = world_transform.getBasis() * other_normal;
										if (other_normal.dot(local_impact_point) > get_plane_dist(btVector4(other_normal[0], other_normal[1], other_normal[2], other_plane[3])) - 0.005) { // TODO: Add caster extents.
											inside_other_planes = false;
											break;
										}
									}
								}

								if (inside_other_planes) { // legit collision
									btVector3 world_impact_point = local_impact_point + world_transform.getOrigin() - normal * get_plane_dist(btVector4(normal[0], normal[1], normal[2], 0.0));
									if (fraction < m_resultCallback.m_closestHitFraction) {
										btCollisionWorld::LocalConvexResult localConvexResult(
											collision_object,
											0,
											normal,
											world_impact_point,
											fraction);

										bool normalInWorldSpace = true;
										m_resultCallback.addSingleResult(localConvexResult, normalInWorldSpace);
										return true;
									}
								}
							}
						}
					}
				}
			}
			else if (collision_shape->getShapeType() == CONVEX_HULL_SHAPE_PROXYTYPE) { // Seems this isn't used at all -- godot uses point clouds.
				// Sadly, the polyhedron is null, so we can't get those sweet, juicy faces.
				//btConvexHullShape *convex_shape = (btConvexHullShape *)collision_shape;
				//auto polyhedron = convex_shape->getConvexPolyhedron();
				/*btConvexPointCloudShape
					getNumPoints
					getUnscaledPoints
					btConvexHullShape *convex_shape = (btConvexHullShape *)collision_shape;
				const btAlignedObjectArray<btFace> *faces = &(convex_shape->getConvexPolyhedron()->m_faces);
				//int num_planes = convex_shape->getConvexPolyhedron()->m_faces.size();//convex_shape->getNumPlanes();
				int num_planes = faces->size();
				for (int plane_index = 0; plane_index < num_planes; ++plane_index) {

				}*/
			}
			else if (collision_shape->getShapeType() == CONVEX_POINT_CLOUD_SHAPE_PROXYTYPE) { // Most godot objects use this
				btConvexPointCloudShape *convex_shape = (btConvexPointCloudShape *)collision_shape;
				// Sadly, the polyhedron is null, so we can't get those sweet, juicy faces.
				//auto polyhedron = convex_shape->getConvexPolyhedron();
				int num_points = convex_shape->getNumPoints();
				const btVector3 *unscaled_points = convex_shape->getUnscaledPoints();
				return process_point_cloud(collision_object, unscaled_points, num_points, world_transform, relative_start, relative_end);
			}
			else { // Not box shape
				m_world->objectQuerySingle(m_castShape, m_convexFromTrans, m_convexToTrans, collision_object, collision_shape, collision_object->getWorldTransform(), m_resultCallback, m_allowedCcdPenetration);
			}
		}
		return false;
	}

	virtual bool process(const btBroadphaseProxy *proxy) {
		///terminate further convex sweep tests, once the closestHitFraction reached zero
		if (m_resultCallback.m_closestHitFraction == btScalar(0.f))
			return false;

		btCollisionObject *collisionObject = (btCollisionObject *)proxy->m_clientObject;

		//only perform raycast if filterMask matches
		if (m_resultCallback.needsCollision(collisionObject->getBroadphaseHandle())) {
#if USE_OLD_GODOT_BULLET_PHYSICS // Old bullet behavior
			m_world->objectQuerySingle(m_castShape, m_convexFromTrans, m_convexToTrans, collisionObject, collisionObject->getCollisionShape(), collisionObject->getWorldTransform(), m_resultCallback, m_allowedCcdPenetration);
#else // !USE_OLD_GODOT_BULLET_PHYSICS
			btCollisionShape *collision_shape = collisionObject->getCollisionShape();
			btTransform test_transform = collisionObject->getWorldTransform(); // bad transform when passed in directly?  Testing.
			process_shape(collisionObject, collision_shape, test_transform);
#endif
		}
		return true;
	}
};

void SpaceBullet::convex_sweep_test(btCollisionWorld *p_world, btConvexShape *p_cast_shape, const btTransform &p_convex_from_world, const btTransform &p_convex_to_world, btCollisionWorld::ConvexResultCallback &p_result_callback, btScalar p_allowed_ccd_penetration) const {
	btTransform convex_from_trans, convexToTrans;
	convex_from_trans = p_convex_from_world;
	convexToTrans = p_convex_to_world;
	btVector3 cast_shape_aabb_min, cast_shape_aabb_max;
	/* Compute AABB that encompasses angular movement */
	{
		btVector3 linVel, angVel;
		btTransformUtil::calculateVelocity(convex_from_trans, convexToTrans, 1.0f, linVel, angVel);
		btVector3 zeroLinVel;
		zeroLinVel.setValue(0, 0, 0);
		btTransform R;
		R.setIdentity();
		R.setRotation(convex_from_trans.getRotation());
		p_cast_shape->calculateTemporalAabb(R, zeroLinVel, angVel, 1.0f, cast_shape_aabb_min, cast_shape_aabb_max);
	}

	p_allowed_ccd_penetration = 0.0;
	btSingleSweepCallback convexCB(p_cast_shape, p_convex_from_world, p_convex_to_world, p_world, p_result_callback, p_allowed_ccd_penetration);
	p_world->getBroadphase()->rayTest(convex_from_trans.getOrigin(), convexToTrans.getOrigin(), convexCB, cast_shape_aabb_min, cast_shape_aabb_max);
}

int SpaceBullet::test_ray_separation(RigidBodyBullet *p_body, const Transform &p_transform, bool p_infinite_inertia, Vector3 &r_recover_motion, PhysicsServer::SeparationResult *r_results, int p_result_max, float p_margin) {
	btTransform body_transform;
	G_TO_B(p_transform, body_transform);
	UNSCALE_BT_BASIS(body_transform);

	if (!p_body->get_kinematic_utilities()) {
		p_body->init_kinematic_utilities();
	}

	btVector3 recover_motion(0, 0, 0);

	int rays_found = 0;
	int rays_found_this_round = 0;

	for (int t(RECOVERING_MOVEMENT_CYCLES); 0 < t; --t) {
		PhysicsServer::SeparationResult *next_results = &r_results[rays_found];
		rays_found_this_round = recover_from_penetration_ray(p_body, body_transform, RECOVERING_MOVEMENT_SCALE, p_infinite_inertia, p_result_max - rays_found, recover_motion, next_results);

		rays_found += rays_found_this_round;
		if (rays_found_this_round == 0) {
			body_transform.getOrigin() += recover_motion;
			break;
		}
	}

	B_TO_G(recover_motion, r_recover_motion);
	return rays_found;
}

struct RecoverPenetrationBroadPhaseCallback : public btBroadphaseAabbCallback {
private:
	btDbvtVolume bounds;

	const btCollisionObject *self_collision_object;
	uint32_t collision_layer;
	uint32_t collision_mask;

	struct CompoundLeafCallback : btDbvt::ICollide {
	private:
		RecoverPenetrationBroadPhaseCallback *parent_callback;
		btCollisionObject *collision_object;

	public:
		CompoundLeafCallback(RecoverPenetrationBroadPhaseCallback *p_parent_callback, btCollisionObject *p_collision_object) :
				parent_callback(p_parent_callback),
				collision_object(p_collision_object) {
		}

		void Process(const btDbvtNode *leaf) {
			BroadphaseResult result;
			result.collision_object = collision_object;
			result.compound_child_index = leaf->dataAsInt;
			parent_callback->results.push_back(result);
		}
	};

public:
	struct BroadphaseResult {
		btCollisionObject *collision_object;
		int compound_child_index;
	};

	Vector<BroadphaseResult> results;

public:
	RecoverPenetrationBroadPhaseCallback(const btCollisionObject *p_self_collision_object, uint32_t p_collision_layer, uint32_t p_collision_mask, btVector3 p_aabb_min, btVector3 p_aabb_max) :
			self_collision_object(p_self_collision_object),
			collision_layer(p_collision_layer),
			collision_mask(p_collision_mask) {
		bounds = btDbvtVolume::FromMM(p_aabb_min, p_aabb_max);
	}

	virtual ~RecoverPenetrationBroadPhaseCallback() {}

	virtual bool process(const btBroadphaseProxy *proxy) {
		btCollisionObject *co = static_cast<btCollisionObject *>(proxy->m_clientObject);
		if (co->getInternalType() <= btCollisionObject::CO_RIGID_BODY) {
			if (self_collision_object != proxy->m_clientObject && GodotFilterCallback::test_collision_filters(collision_layer, collision_mask, proxy->m_collisionFilterGroup, proxy->m_collisionFilterMask)) {
				if (co->getCollisionShape()->isCompound()) {
					const btCompoundShape *cs = static_cast<btCompoundShape *>(co->getCollisionShape());

					if (cs->getNumChildShapes() > 1) {
						const btDbvt *tree = cs->getDynamicAabbTree();
						ERR_FAIL_COND_V(tree == nullptr, true);

						// Transform bounds into compound shape local space
						const btTransform other_in_compound_space = co->getWorldTransform().inverse();
						const btMatrix3x3 abs_b = other_in_compound_space.getBasis().absolute();
						const btVector3 local_center = other_in_compound_space(bounds.Center());
						const btVector3 local_extent = bounds.Extents().dot3(abs_b[0], abs_b[1], abs_b[2]);
						const btVector3 local_aabb_min = local_center - local_extent;
						const btVector3 local_aabb_max = local_center + local_extent;
						const btDbvtVolume local_bounds = btDbvtVolume::FromMM(local_aabb_min, local_aabb_max);

						// Test collision against compound child shapes using its AABB tree
						CompoundLeafCallback compound_leaf_callback(this, co);
						tree->collideTV(tree->m_root, local_bounds, compound_leaf_callback);
					} else {
						// If there's only a single child shape then there's no need to search any more, we know which child overlaps
						BroadphaseResult result;
						result.collision_object = co;
						result.compound_child_index = 0;
						results.push_back(result);
					}
				} else {
					BroadphaseResult result;
					result.collision_object = co;
					result.compound_child_index = -1;
					results.push_back(result);
				}
				return true;
			}
		}
		return false;
	}
};

bool SpaceBullet::recover_from_penetration(RigidBodyBullet *p_body, const btTransform &p_body_position, btScalar p_recover_movement_scale, bool p_infinite_inertia, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result, const Set<RID> &p_exclude) {
	// Calculate the cumulative AABB of all shapes of the kinematic body
	btVector3 aabb_min, aabb_max;
	bool shapes_found = false;

	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
			// Skip rayshape in order to implement custom separation process
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		btVector3 shape_aabb_min, shape_aabb_max;
		kin_shape.shape->getAabb(shape_transform, shape_aabb_min, shape_aabb_max);

		if (!shapes_found) {
			aabb_min = shape_aabb_min;
			aabb_max = shape_aabb_max;
			shapes_found = true;
		} else {
			aabb_min.setX((aabb_min.x() < shape_aabb_min.x()) ? aabb_min.x() : shape_aabb_min.x());
			aabb_min.setY((aabb_min.y() < shape_aabb_min.y()) ? aabb_min.y() : shape_aabb_min.y());
			aabb_min.setZ((aabb_min.z() < shape_aabb_min.z()) ? aabb_min.z() : shape_aabb_min.z());

			aabb_max.setX((aabb_max.x() > shape_aabb_max.x()) ? aabb_max.x() : shape_aabb_max.x());
			aabb_max.setY((aabb_max.y() > shape_aabb_max.y()) ? aabb_max.y() : shape_aabb_max.y());
			aabb_max.setZ((aabb_max.z() > shape_aabb_max.z()) ? aabb_max.z() : shape_aabb_max.z());
		}
	}

	// If there are no shapes then there is no penetration either
	if (!shapes_found) {
		return false;
	}

	// Perform broadphase test
	RecoverPenetrationBroadPhaseCallback recover_broad_result(p_body->get_bt_collision_object(), p_body->get_collision_layer(), p_body->get_collision_mask(), aabb_min, aabb_max);
	dynamicsWorld->getBroadphase()->aabbTest(aabb_min, aabb_max, recover_broad_result);

	bool penetration = false;

	// Perform narrowphase per shape
	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() == CUSTOM_CONVEX_SHAPE_TYPE) {
			// Skip rayshape in order to implement custom separation process
			continue;
		}

		if (kin_shape.shape->getShapeType() == EMPTY_SHAPE_PROXYTYPE) {
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		for (int i = recover_broad_result.results.size() - 1; 0 <= i; --i) {
			btCollisionObject *otherObject = recover_broad_result.results[i].collision_object;

			CollisionObjectBullet *gObj = static_cast<CollisionObjectBullet *>(otherObject->getUserPointer());
			if (p_exclude.has(gObj->get_self())) {
				continue;
			}

			if (p_infinite_inertia && !otherObject->isStaticOrKinematicObject()) {
				otherObject->activate(); // Force activation of hitten rigid, soft body
				continue;
			} else if (!p_body->get_bt_collision_object()->checkCollideWith(otherObject) || !otherObject->checkCollideWith(p_body->get_bt_collision_object())) {
				continue;
			}

			if (otherObject->getCollisionShape()->isCompound()) {
				const btCompoundShape *cs = static_cast<const btCompoundShape *>(otherObject->getCollisionShape());
				if (cs->getNumChildShapes() == 0) {
					continue; // No shapes to depenetrate from.
				}
				int shape_idx = recover_broad_result.results[i].compound_child_index;
				ERR_FAIL_COND_V(shape_idx < 0 || shape_idx >= cs->getNumChildShapes(), false);

				if (cs->getChildShape(shape_idx)->isConvex()) {
					if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(cs->getChildShape(shape_idx)), otherObject, kinIndex, shape_idx, shape_transform, otherObject->getWorldTransform() * cs->getChildTransform(shape_idx), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
						penetration = true;
					}
				} else {
					if (RFP_convex_world_test(kin_shape.shape, cs->getChildShape(shape_idx), p_body->get_bt_collision_object(), otherObject, kinIndex, shape_idx, shape_transform, otherObject->getWorldTransform() * cs->getChildTransform(shape_idx), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
						penetration = true;
					}
				}
			} else if (otherObject->getCollisionShape()->isConvex()) { /// Execute GJK test against object shape
				if (RFP_convex_convex_test(kin_shape.shape, static_cast<const btConvexShape *>(otherObject->getCollisionShape()), otherObject, kinIndex, 0, shape_transform, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
					penetration = true;
				}
			} else {
				if (RFP_convex_world_test(kin_shape.shape, otherObject->getCollisionShape(), p_body->get_bt_collision_object(), otherObject, kinIndex, 0, shape_transform, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, r_recover_result)) {
					penetration = true;
				}
			}
		}
	}

	return penetration;
}

bool SpaceBullet::RFP_convex_convex_test(const btConvexShape *p_shapeA, const btConvexShape *p_shapeB, btCollisionObject *p_objectB, int p_shapeId_A, int p_shapeId_B, const btTransform &p_transformA, const btTransform &p_transformB, btScalar p_recover_movement_scale, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result) {
	// Initialize GJK input
	btGjkPairDetector::ClosestPointInput gjk_input;
	gjk_input.m_transformA = p_transformA;
	// Avoid repeat penetrations
	gjk_input.m_transformA.getOrigin() += r_delta_recover_movement;
	gjk_input.m_transformB = p_transformB;

	// Perform GJK test
	btPointCollector result;
	btGjkPairDetector gjk_pair_detector(p_shapeA, p_shapeB, gjk_simplex_solver, gjk_epa_pen_solver);
	gjk_pair_detector.getClosestPoints(gjk_input, result, nullptr);
	if (0 > result.m_distance) {
		// Has penetration
		r_delta_recover_movement += result.m_normalOnBInWorld * (result.m_distance * -1 * p_recover_movement_scale);

		if (r_recover_result) {
			if (result.m_distance < r_recover_result->penetration_distance) {
				r_recover_result->hasPenetration = true;
				r_recover_result->local_shape_most_recovered = p_shapeId_A;
				r_recover_result->other_collision_object = p_objectB;
				r_recover_result->other_compound_shape_index = p_shapeId_B;
				r_recover_result->penetration_distance = result.m_distance;
				r_recover_result->pointWorld = result.m_pointInWorld;
				r_recover_result->normal = result.m_normalOnBInWorld;
			}
		}
		return true;
	}
	return false;
}

bool SpaceBullet::RFP_convex_world_test(const btConvexShape *p_shapeA, const btCollisionShape *p_shapeB, btCollisionObject *p_objectA, btCollisionObject *p_objectB, int p_shapeId_A, int p_shapeId_B, const btTransform &p_transformA, const btTransform &p_transformB, btScalar p_recover_movement_scale, btVector3 &r_delta_recover_movement, RecoverResult *r_recover_result) {
	/// Contact test

	btTransform tA(p_transformA);
	// Avoid repeat penetrations
	tA.getOrigin() += r_delta_recover_movement;

	btCollisionObjectWrapper obA(nullptr, p_shapeA, p_objectA, tA, -1, p_shapeId_A);
	btCollisionObjectWrapper obB(nullptr, p_shapeB, p_objectB, p_transformB, -1, p_shapeId_B);

	btCollisionAlgorithm *algorithm = dispatcher->findAlgorithm(&obA, &obB, nullptr, BT_CONTACT_POINT_ALGORITHMS);
	if (algorithm) {
		GodotDeepPenetrationContactResultCallback contactPointResult(&obA, &obB);
		//discrete collision detection query
		algorithm->processCollision(&obA, &obB, dynamicsWorld->getDispatchInfo(), &contactPointResult);

		algorithm->~btCollisionAlgorithm();
		dispatcher->freeCollisionAlgorithm(algorithm);

		if (contactPointResult.hasHit()) {
			r_delta_recover_movement += contactPointResult.m_pointNormalWorld * (contactPointResult.m_penetration_distance * -1 * p_recover_movement_scale);
			if (r_recover_result) {
				if (contactPointResult.m_penetration_distance < r_recover_result->penetration_distance) {
					r_recover_result->hasPenetration = true;
					r_recover_result->local_shape_most_recovered = p_shapeId_A;
					r_recover_result->other_collision_object = p_objectB;
					r_recover_result->other_compound_shape_index = p_shapeId_B;
					r_recover_result->penetration_distance = contactPointResult.m_penetration_distance;
					r_recover_result->pointWorld = contactPointResult.m_pointWorld;
					r_recover_result->normal = contactPointResult.m_pointNormalWorld;
				}
			}
			return true;
		}
	}
	return false;
}

int SpaceBullet::add_separation_result(PhysicsServer::SeparationResult *r_result, const SpaceBullet::RecoverResult &p_recover_result, int p_shape_id, const btCollisionObject *p_other_object) const {
	// optimize results (ignore non-colliding)
	if (p_recover_result.penetration_distance < 0.0) {
		const btRigidBody *btRigid = static_cast<const btRigidBody *>(p_other_object);
		CollisionObjectBullet *collisionObject = static_cast<CollisionObjectBullet *>(p_other_object->getUserPointer());

		r_result->collision_depth = p_recover_result.penetration_distance;
		B_TO_G(p_recover_result.pointWorld, r_result->collision_point);
		B_TO_G(p_recover_result.normal, r_result->collision_normal);
		B_TO_G(btRigid->getVelocityInLocalPoint(p_recover_result.pointWorld - btRigid->getWorldTransform().getOrigin()), r_result->collider_velocity);
		r_result->collision_local_shape = p_shape_id;
		r_result->collider_id = collisionObject->get_instance_id();
		r_result->collider = collisionObject->get_self();
		r_result->collider_shape = p_recover_result.other_compound_shape_index;

		return 1;
	} else {
		return 0;
	}
}

int SpaceBullet::recover_from_penetration_ray(RigidBodyBullet *p_body, const btTransform &p_body_position, btScalar p_recover_movement_scale, bool p_infinite_inertia, int p_result_max, btVector3 &r_delta_recover_movement, PhysicsServer::SeparationResult *r_results) {
	// Calculate the cumulative AABB of all shapes of the kinematic body
	btVector3 aabb_min, aabb_max;
	bool shapes_found = false;

	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE) {
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		btVector3 shape_aabb_min, shape_aabb_max;
		kin_shape.shape->getAabb(shape_transform, shape_aabb_min, shape_aabb_max);

		if (!shapes_found) {
			aabb_min = shape_aabb_min;
			aabb_max = shape_aabb_max;
			shapes_found = true;
		} else {
			aabb_min.setX((aabb_min.x() < shape_aabb_min.x()) ? aabb_min.x() : shape_aabb_min.x());
			aabb_min.setY((aabb_min.y() < shape_aabb_min.y()) ? aabb_min.y() : shape_aabb_min.y());
			aabb_min.setZ((aabb_min.z() < shape_aabb_min.z()) ? aabb_min.z() : shape_aabb_min.z());

			aabb_max.setX((aabb_max.x() > shape_aabb_max.x()) ? aabb_max.x() : shape_aabb_max.x());
			aabb_max.setY((aabb_max.y() > shape_aabb_max.y()) ? aabb_max.y() : shape_aabb_max.y());
			aabb_max.setZ((aabb_max.z() > shape_aabb_max.z()) ? aabb_max.z() : shape_aabb_max.z());
		}
	}

	// If there are no shapes then there is no penetration either
	if (!shapes_found) {
		return 0;
	}

	// Perform broadphase test
	RecoverPenetrationBroadPhaseCallback recover_broad_result(p_body->get_bt_collision_object(), p_body->get_collision_layer(), p_body->get_collision_mask(), aabb_min, aabb_max);
	dynamicsWorld->getBroadphase()->aabbTest(aabb_min, aabb_max, recover_broad_result);

	int ray_count = 0;

	// Perform narrowphase per shape
	for (int kinIndex = p_body->get_kinematic_utilities()->shapes.size() - 1; 0 <= kinIndex; --kinIndex) {
		if (ray_count >= p_result_max) {
			break;
		}

		const RigidBodyBullet::KinematicShape &kin_shape(p_body->get_kinematic_utilities()->shapes[kinIndex]);
		if (!kin_shape.is_active()) {
			continue;
		}

		if (kin_shape.shape->getShapeType() != CUSTOM_CONVEX_SHAPE_TYPE) {
			continue;
		}

		btTransform shape_transform = p_body_position * kin_shape.transform;
		shape_transform.getOrigin() += r_delta_recover_movement;

		for (int i = recover_broad_result.results.size() - 1; 0 <= i; --i) {
			btCollisionObject *otherObject = recover_broad_result.results[i].collision_object;
			if (p_infinite_inertia && !otherObject->isStaticOrKinematicObject()) {
				otherObject->activate(); // Force activation of hitten rigid, soft body
				continue;
			} else if (!p_body->get_bt_collision_object()->checkCollideWith(otherObject) || !otherObject->checkCollideWith(p_body->get_bt_collision_object())) {
				continue;
			}

			if (otherObject->getCollisionShape()->isCompound()) {
				const btCompoundShape *cs = static_cast<const btCompoundShape *>(otherObject->getCollisionShape());
				int shape_idx = recover_broad_result.results[i].compound_child_index;
				ERR_FAIL_COND_V(shape_idx < 0 || shape_idx >= cs->getNumChildShapes(), false);

				RecoverResult recover_result;
				if (RFP_convex_world_test(kin_shape.shape, cs->getChildShape(shape_idx), p_body->get_bt_collision_object(), otherObject, kinIndex, shape_idx, shape_transform, otherObject->getWorldTransform() * cs->getChildTransform(shape_idx), p_recover_movement_scale, r_delta_recover_movement, &recover_result)) {
					ray_count = add_separation_result(&r_results[ray_count], recover_result, kinIndex, otherObject);
				}
			} else {
				RecoverResult recover_result;
				if (RFP_convex_world_test(kin_shape.shape, otherObject->getCollisionShape(), p_body->get_bt_collision_object(), otherObject, kinIndex, 0, shape_transform, otherObject->getWorldTransform(), p_recover_movement_scale, r_delta_recover_movement, &recover_result)) {
					ray_count = add_separation_result(&r_results[ray_count], recover_result, kinIndex, otherObject);
				}
			}
		}
	}

	return ray_count;
}
