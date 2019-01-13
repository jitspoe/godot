// Copyright (c) 2007-2017 Juan Linietsky, Ariel Manzur.
// Copyright (c) 2014-2017 Godot Engine contributors (cf. AUTHORS.md)

// Permission is hereby granted, free of charge, to any person obtaining a copy
// of this software and associated documentation files (the "Software"), to deal
// in the Software without restriction, including without limitation the rights
// to use, copy, modify, merge, publish, distribute, sublicense, and/or sell
// copies of the Software, and to permit persons to whom the Software is
// furnished to do so, subject to the following conditions:

// The above copyright notice and this permission notice shall be included in all
// copies or substantial portions of the Software.

// THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR
// IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY,
// FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE
// AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER
// LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM,
// OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE
// SOFTWARE.

// -- Godot Engine <https://godotengine.org>

#include "thirdparty/assimp/include/assimp/DefaultLogger.hpp"
#include "thirdparty/assimp/include/assimp/Importer.hpp"
#include "thirdparty/assimp/include/assimp/LogStream.hpp"
#include "thirdparty/assimp/include/assimp/Logger.hpp"
#include "thirdparty/assimp/include/assimp/SceneCombiner.h"
#include "thirdparty/assimp/include/assimp/cexport.h"
#include "thirdparty/assimp/include/assimp/cimport.h"
#include "thirdparty/assimp/include/assimp/matrix4x4.h"
#include "thirdparty/assimp/include/assimp/pbrmaterial.h"
#include "thirdparty/assimp/include/assimp/postprocess.h"
#include "thirdparty/assimp/include/assimp/scene.h"

#include "core/bind/core_bind.h"
#include "editor/editor_file_system.h"
#include "editor/import/resource_importer_scene.h"
#include "editor_scene_importer_asset_import.h"
#include "scene/3d/camera.h"
#include "scene/3d/light.h"
#include "scene/3d/mesh_instance.h"
#include "scene/animation/animation_player.h"
#include "scene/main/node.h"
#include "scene/resources/material.h"
#include "scene/resources/surface_tool.h"
#include "zutil.h"
#include <string>

void EditorSceneImporterAssetImport::get_extensions(List<String> *r_extensions) const {
	r_extensions->push_back("amf"); //crashes
	r_extensions->push_back("3ds"); //hangs
	r_extensions->push_back("ac");
	r_extensions->push_back("ase"); //crashes
	r_extensions->push_back("assbin");
	r_extensions->push_back("assxml");
	r_extensions->push_back("b3d");
	r_extensions->push_back("bvh"); //crashes
	r_extensions->push_back("dxf");
	//Don't shadow the existing gltf importer
	r_extensions->push_back("gltf");
	r_extensions->push_back("glb");
	//Don't shadow the existing collada importer either
	r_extensions->push_back("csm"); //crashes
	r_extensions->push_back("hmp");
	r_extensions->push_back("lwo");
	r_extensions->push_back("lws");
	r_extensions->push_back("md2");
	r_extensions->push_back("md3");
	r_extensions->push_back("md5mesh"); //md5
	r_extensions->push_back("mdc");
	r_extensions->push_back("mdl");
	r_extensions->push_back("nff");
	r_extensions->push_back("ndo");
	r_extensions->push_back("off");
	r_extensions->push_back("ogex"); //opengex //crashes
	r_extensions->push_back("ply");
	r_extensions->push_back("ms3d");
	r_extensions->push_back("cob");
	r_extensions->push_back("blend");
	r_extensions->push_back("xgl");
	r_extensions->push_back("fbx");
	r_extensions->push_back("q3d");
	r_extensions->push_back("q3bsp");
	r_extensions->push_back("raw"); //crashes
	r_extensions->push_back("sib");
	r_extensions->push_back("smd");
	r_extensions->push_back("stl"); //crashes
	r_extensions->push_back("terragen");
	r_extensions->push_back("3d");
	r_extensions->push_back("x"); //crashes
	r_extensions->push_back("x3d");
	r_extensions->push_back("3mf");
	r_extensions->push_back("pmx"); //mmd
}

uint32_t EditorSceneImporterAssetImport::get_import_flags() const {
	return IMPORT_SCENE;
}

AssimpStream::AssimpStream() {
	// empty
}

AssimpStream::~AssimpStream() {
	// empty
}

void AssimpStream::write(const char *message) {
	print_verbose(String("Open Asset Importer: ") + String(message).strip_edges());
}

Node *EditorSceneImporterAssetImport::import_scene(const String &p_path, uint32_t p_flags, int p_bake_fps, List<String> *r_missing_deps, Error *r_err) {
	Assimp::Importer importer;
	std::wstring w_path = ProjectSettings::get_singleton()->globalize_path(p_path).c_str();
	std::string s_path(w_path.begin(), w_path.end());
	importer.SetPropertyBool(AI_CONFIG_PP_FD_REMOVE, true);
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, false);
	importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
	//importer.SetPropertyFloat(AI_CONFIG_PP_DB_THRESHOLD, 1.0f);
	int32_t post_process_Steps = aiProcess_CalcTangentSpace |
								 //aiProcess_FlipUVs |
								 aiProcess_FlipWindingOrder |
								 //aiProcess_DropNormals |
								 aiProcess_GenSmoothNormals |
								 //aiProcess_JoinIdenticalVertices |
								 aiProcess_ImproveCacheLocality |
								 aiProcess_LimitBoneWeights |
								 aiProcess_RemoveRedundantMaterials |
								 aiProcess_SplitLargeMeshes |
								 aiProcess_Triangulate |
								 aiProcess_GenUVCoords |
								 //aiProcess_SortByPType |
								 //aiProcess_FindDegenerates |
								 //aiProcess_FindInvalidData |
								 aiProcess_TransformUVCoords |
								 aiProcess_FindInstances |
								 //aiProcess_FixInfacingNormals |
								 //aiProcess_ValidateDataStructure |
								 aiProcess_OptimizeMeshes |
								 // Optimize graph must be on or many changes will need to be made
								 //aiProcess_OptimizeGraph |
								 //aiProcess_Debone |
								 //aiProcess_EmbedTextures |
								 aiProcess_SplitByBoneCount |
								 0;
	const aiScene *scene = importer.ReadFile(s_path.c_str(),
			post_process_Steps);
	ERR_EXPLAIN(String("Open Asset Importer failed to open: ") + String(importer.GetErrorString()));
	ERR_FAIL_COND_V(scene == NULL, NULL);
	return _generate_scene(p_path, scene, p_flags, p_bake_fps);
}

template <class T>
struct EditorSceneImporterAssetImportInterpolate {

	T lerp(const T &a, const T &b, float c) const {

		return a + (b - a) * c;
	}

	T catmull_rom(const T &p0, const T &p1, const T &p2, const T &p3, float t) {

		float t2 = t * t;
		float t3 = t2 * t;

		return 0.5f * ((2.0f * p1) + (-p0 + p2) * t + (2.0f * p0 - 5.0f * p1 + 4 * p2 - p3) * t2 + (-p0 + 3.0f * p1 - 3.0f * p2 + p3) * t3);
	}

	T bezier(T start, T control_1, T control_2, T end, float t) {
		/* Formula from Wikipedia article on Bezier curves. */
		real_t omt = (1.0 - t);
		real_t omt2 = omt * omt;
		real_t omt3 = omt2 * omt;
		real_t t2 = t * t;
		real_t t3 = t2 * t;

		return start * omt3 + control_1 * omt2 * t * 3.0 + control_2 * omt * t2 * 3.0 + end * t3;
	}
};

//thank you for existing, partial specialization
template <>
struct EditorSceneImporterAssetImportInterpolate<Quat> {

	Quat lerp(const Quat &a, const Quat &b, float c) const {
		ERR_FAIL_COND_V(!a.is_normalized(), Quat());
		ERR_FAIL_COND_V(!b.is_normalized(), Quat());

		return a.slerp(b, c).normalized();
	}

	Quat catmull_rom(const Quat &p0, const Quat &p1, const Quat &p2, const Quat &p3, float c) {
		ERR_FAIL_COND_V(!p1.is_normalized(), Quat());
		ERR_FAIL_COND_V(!p2.is_normalized(), Quat());

		return p1.slerp(p2, c).normalized();
	}

	Quat bezier(Quat start, Quat control_1, Quat control_2, Quat end, float t) {
		ERR_FAIL_COND_V(!start.is_normalized(), Quat());
		ERR_FAIL_COND_V(!end.is_normalized(), Quat());

		return start.slerp(end, t).normalized();
	}
};

template <class T>
T EditorSceneImporterAssetImport::_interpolate_track(const Vector<float> &p_times, const Vector<T> &p_values, float p_time, AssetImportAnimation::Interpolation p_interp) {
	//TODO(Ernest) RESTORE OTHER TYPES
	//could use binary search, worth it?
	int idx = -1;
	for (int i = 0; i < p_times.size(); i++) {
		if (p_times[i] > p_time)
			break;
		idx++;
	}

	EditorSceneImporterAssetImportInterpolate<T> interp;

	switch (p_interp) {
		case AssetImportAnimation::INTERP_LINEAR: {

			if (idx == -1) {
				return p_values[0];
			} else if (idx >= p_times.size() - 1) {
				return p_values[p_times.size() - 1];
			}

			float c = (p_time - p_times[idx]) / (p_times[idx + 1] - p_times[idx]);

			return interp.lerp(p_values[idx], p_values[idx + 1], c);

		} break;
	}

	ERR_FAIL_V(p_values[0]);
}

Spatial *EditorSceneImporterAssetImport::_generate_scene(const String &p_path, const aiScene *scene, const uint32_t p_flags, int p_bake_fps) {
	ERR_FAIL_COND_V(scene == NULL, NULL);
	Spatial *root = memnew(Spatial);

	if (p_path.get_file().get_extension().to_lower() == "fbx") {
		Transform format_xform = _format_xform(p_path, scene);
		format_xform.basis.set_quat_scale(Quat(), format_xform.basis.get_scale());
		root->set_transform(format_xform);
	}
	AnimationPlayer *ap = memnew(AnimationPlayer);
	root->add_child(ap);
	ap->set_owner(root);
	ap->set_name(TTR("AnimationPlayer"));
	Set<String> bone_names;
	Set<String> light_names;
	Set<String> camera_names;
	for (size_t l = 0; l < scene->mNumLights; l++) {
		Light *light = NULL;
		aiLight *ai_light = scene->mLights[l];
		ERR_CONTINUE(ai_light == NULL);
		if (ai_light->mType == aiLightSource_DIRECTIONAL) {
			light = memnew(DirectionalLight);
			Vector3 dir = Vector3(ai_light->mDirection.x, ai_light->mDirection.y, ai_light->mDirection.z);
			dir.normalize();
			Transform xform;
			Quat quat;
			quat.set_euler(dir);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			xform.origin = pos;
			light->set_transform(xform);
		} else if (ai_light->mType == aiLightSource_POINT) {
			light = memnew(OmniLight);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			Transform xform;
			xform.origin = pos;
			light->set_transform(xform);
			// No idea for energy
			light->set_param(Light::PARAM_ATTENUATION, 0.0f);
		} else if (ai_light->mType == aiLightSource_SPOT) {
			light = memnew(SpotLight);
			Vector3 pos = Vector3(ai_light->mPosition.x, ai_light->mPosition.y, ai_light->mPosition.z);
			Transform xform;
			xform.origin = pos;
			Vector3 dir = Vector3(ai_light->mDirection.x, ai_light->mDirection.y, ai_light->mDirection.z);
			dir.normalize();
			Quat quat;
			quat.set_euler(dir);
			xform.basis = quat;
			light->set_transform(xform);
			// No idea for energy
			light->set_param(Light::PARAM_ATTENUATION, 0.0f);
		}
		ERR_CONTINUE(light == NULL);
		light->set_color(Color(ai_light->mColorDiffuse.r, ai_light->mColorDiffuse.g, ai_light->mColorDiffuse.b));
		root->add_child(light);
		light->set_name(_ai_string_to_string(ai_light->mName));
		light->set_owner(root);
		light_names.insert(_ai_string_to_string(scene->mLights[l]->mName));
	}
	for (size_t c = 0; c < scene->mNumCameras; c++) {
		aiCamera *ai_camera = scene->mCameras[c];
		Camera *camera = memnew(Camera);
		float near = ai_camera->mClipPlaneNear;
		if (Math::is_equal_approx(near, 0.0f)) {
			near = 0.1f;
		}
		camera->set_perspective(Math::rad2deg(ai_camera->mHorizontalFOV) * 2.0f, near, ai_camera->mClipPlaneFar);
		Vector3 pos = Vector3(ai_camera->mPosition.x, ai_camera->mPosition.y, ai_camera->mPosition.z);

		Vector3 look_at = Vector3(ai_camera->mLookAt.x, ai_camera->mLookAt.y, ai_camera->mLookAt.z).normalized();
		Quat quat;
		quat.set_euler(look_at);
		Transform xform;
		xform.basis = quat;
		xform.set_origin(pos);
		root->add_child(camera);
		camera->set_transform(xform);
		camera->set_name(_ai_string_to_string(ai_camera->mName));
		camera->set_owner(root);
		camera_names.insert(_ai_string_to_string(scene->mCameras[c]->mName));
	}
	Map<Skeleton *, MeshInstance *> skeletons;
	Map<String, Transform> bone_rests;
	Map<MeshInstance *, String> meshes;
	_generate_node(p_path, scene, scene->mRootNode, root, root, bone_names, light_names, camera_names, skeletons, bone_rests, meshes);

	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		_set_bone_parent(E->key(), root);
	}
	_move_mesh(p_path, scene, root, root, meshes, skeletons);

	Node *skeleton_root = _find_skeleton_root(skeletons, meshes, root);
	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		E->key()->localize_rests();
	}

	for (int i = 0; i < scene->mNumAnimations; i++) {
		_import_animation(p_path, scene, ap, i, p_bake_fps, skeletons, skeleton_root);
	}
	List<StringName> animation_names;
	ap->get_animation_list(&animation_names);
	if (animation_names.empty()) {
		root->remove_child(ap);
	}
	return root;
}

Node *EditorSceneImporterAssetImport::_find_skeleton_root(Map<Skeleton *, MeshInstance *> &skeletons, Map<MeshInstance *, String> &meshes, Spatial *root) {
	Node *armature = NULL;
	for (Map<Skeleton *, MeshInstance *>::Element *E = skeletons.front(); E; E = E->next()) {
		if (meshes.has(E->get())) {
			String name = meshes[E->get()];
			if (name != "") {
				armature = root->find_node(name);

				return armature;
			}
		}
		E->key()->localize_rests();
	}
	return NULL;
}

void EditorSceneImporterAssetImport::_set_bone_parent(Skeleton *s, Node *p_owner) {
	for (size_t j = 0; j < s->get_bone_count(); j++) {
		String bone_name = s->get_bone_name(j);
		int32_t node_parent_index = -1;
		const Node *bone_node = p_owner->find_node(bone_name);
		if (bone_node == NULL) {
			continue;
		}
		if (bone_node != NULL && bone_node->get_parent() != NULL) {
			node_parent_index = s->find_bone(bone_node->get_parent()->get_name());
		}
		//ERR_EXPLAIN(String("Can't find parent bone for ") + bone_node->get_name())
		//ERR_CONTINUE(node_parent_index == -1);
		s->set_bone_parent(j, node_parent_index);
	}
}

void EditorSceneImporterAssetImport::_insert_animation_track(const aiScene *p_scene, const String p_path, int p_bake_fps, Ref<Animation> animation, float ticks_per_second, float length, const Skeleton *sk, size_t i, const aiNodeAnim *track, String node_name, NodePath node_path) {
	if (track->mNumRotationKeys || track->mNumPositionKeys || track->mNumScalingKeys) {
		//make transform track
		int track_idx = animation->get_track_count();
		animation->add_track(Animation::TYPE_TRANSFORM);
		animation->track_set_path(track_idx, node_path);
		//first determine animation length

		for (int i = 0; i < track->mNumRotationKeys; i++) {
			length = MAX(length, track->mRotationKeys[i].mTime / ticks_per_second);
		}
		for (int i = 0; i < track->mNumPositionKeys; i++) {
			length = MAX(length, track->mPositionKeys[i].mTime / ticks_per_second);
		}
		for (int i = 0; i < track->mNumScalingKeys; i++) {
			length = MAX(length, track->mScalingKeys[i].mTime / ticks_per_second);
		}

		// Todo blend keys

		float increment = 1.0 / float(p_bake_fps);
		float time = 0.0;

		Vector3 base_pos;
		Quat base_rot;
		Vector3 base_scale = Vector3(1, 1, 1);

		if (track->mNumRotationKeys != 0) {
			aiQuatKey key = track->mRotationKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			real_t w = key.mValue.w;
			Quat q(x, y, z, w);
			q = q.normalized();
			base_rot = q;
		}

		if (track->mNumPositionKeys != 0) {
			aiVectorKey key = track->mPositionKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_pos = Vector3(x, y, z);
		}

		if (track->mNumScalingKeys != 0) {
			aiVectorKey key = track->mScalingKeys[0];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_scale = Vector3(x, y, z);
		}

		bool last = false;

		Vector<Vector3> pos_values;
		Vector<float> pos_times;
		Vector<Vector3> scale_values;
		Vector<float> scale_times;
		Vector<Quat> rot_values;
		Vector<float> rot_times;

		for (size_t p = 0; p < track->mNumPositionKeys; p++) {
			aiVector3D pos = track->mPositionKeys[p].mValue;
			pos_values.push_back(Vector3(pos.x, pos.y, pos.z));
			pos_times.push_back(track->mPositionKeys[p].mTime / ticks_per_second);
		}

		for (size_t r = 0; r < track->mNumRotationKeys; r++) {
			aiQuaternion quat = track->mRotationKeys[r].mValue;
			rot_values.push_back(Quat(quat.x, quat.y, quat.z, quat.w).normalized());
			rot_times.push_back(track->mRotationKeys[r].mTime / ticks_per_second);
		}

		for (size_t sc = 0; sc < track->mNumScalingKeys; sc++) {
			aiVector3D scale = track->mScalingKeys[sc].mValue;
			scale_values.push_back(Vector3(scale.x, scale.y, scale.z));
			scale_times.push_back(track->mScalingKeys[sc].mTime / ticks_per_second);
		}
		while (true) {
			Vector3 pos = base_pos;
			Quat rot = base_rot;
			Vector3 scale = base_scale;

			if (pos_values.size()) {
				pos = _interpolate_track<Vector3>(pos_times, pos_values, time, AssetImportAnimation::INTERP_LINEAR);
			}

			if (rot_values.size()) {
				rot = _interpolate_track<Quat>(rot_times, rot_values, time, AssetImportAnimation::INTERP_LINEAR);
			}

			if (scale_values.size()) {
				scale = _interpolate_track<Vector3>(scale_times, scale_values, time, AssetImportAnimation::INTERP_LINEAR);
			}

			if (sk != NULL && sk->find_bone(node_name) != -1) {
				Transform xform;
				//xform.basis = Basis(rot);
				//xform.basis.scale(scale);
				xform.basis.set_quat_scale(rot, scale);
				xform.origin = pos;

				int bone = sk->find_bone(node_name);
				Transform rest_xform = sk->get_bone_rest(bone).affine_inverse();
				xform = rest_xform * xform;

				rot = xform.basis.get_rotation_quat();
				scale = xform.basis.get_scale();
				pos = xform.origin;
			}

			Transform format_xform;
			format_xform.basis.set_quat_scale(rot, scale);
			format_xform.origin = pos;

			Transform scale_xform = _format_xform(p_path, p_scene);
			scale_xform.basis.set_quat_scale(Quat(), scale_xform.basis.get_scale());
			format_xform = scale_xform * format_xform;

			rot = format_xform.basis.get_rotation_quat();
			scale = format_xform.basis.get_scale();
			pos = format_xform.origin;

			animation->transform_track_insert_key(track_idx, time, pos, rot, scale);

			if (last) {
				break;
			}
			time += increment;
			if (time >= length) {
				last = true;
				time = length;
			}
		}
	}
}

void EditorSceneImporterAssetImport::_import_animation(const String path, const aiScene *p_scene, AnimationPlayer *ap, int32_t p_index, int p_bake_fps, Map<Skeleton *, MeshInstance *> p_skeletons, Node *p_skeleton_root) {
	String name = "Animation";
	aiAnimation const *anim = NULL;
	if (p_index != -1) {
		anim = p_scene->mAnimations[p_index];
		if (anim->mName.length > 0) {
			name = _ai_raw_string_to_string(anim->mName);
		}
	}

	Ref<Animation> animation;
	animation.instance();
	float length = 0.0f;
	animation->set_name(name);
	float ticks_per_second = p_scene->mAnimations[p_index]->mTicksPerSecond;
	if (path.get_file().get_extension().to_lower() == "glb" || path.get_file().get_extension().to_lower() == "gltf" && Math::is_equal_approx(ticks_per_second, 0.0f)) {
		ticks_per_second = 1000.0f;
	} else if (Math::is_equal_approx(ticks_per_second, 0.0f)) {
		ticks_per_second = 25.0f;
	}
	length = anim->mDuration / ticks_per_second;
	if (anim) {
		Set<String> tracks;
		_get_track_set(p_scene, tracks);

		bool is_found_node = false;
		for (size_t i = 0; i < anim->mNumChannels; i++) {
			const aiNodeAnim *track = anim->mChannels[i];
			String node_name = _ai_string_to_string(track->mNodeName);
			NodePath node_path = node_name;
			bool found_bone = false;

			Vector<String> fbx_pivot_name = node_name.split("_$AssimpFbx$_");
			for (Map<Skeleton *, MeshInstance *>::Element *E = p_skeletons.front(); E; E = E->next()) {
				Skeleton *sk = E->key();
				if (fbx_pivot_name.size() != 1) {
					node_name = fbx_pivot_name[0];
				}
				if (p_skeleton_root != NULL && p_skeleton_root->get_name() == node_name) {
					break;
				}
				if (sk->find_bone(node_name) != -1) {
					const String path = ap->get_owner()->get_path_to(sk);
					if (path.empty()) {
						continue;
					}
					node_path = path + ":" + node_name;

					_insert_animation_track(p_scene, path, p_bake_fps, animation, ticks_per_second, length, sk, i, track, node_name, node_path);
					found_bone = found_bone || true;
				}
			}

			if (found_bone) {
				continue;
			}

			if (fbx_pivot_name.size() != 1) {
				node_name = fbx_pivot_name[0];
			}
			const Node *node = ap->get_owner()->find_node(node_name);
			if (node != NULL) {
				const String path = ap->get_owner()->get_path_to(node);
				ERR_CONTINUE(animation->find_track(path) != -1);
				ERR_EXPLAIN("Can't animate path");
				ERR_CONTINUE(path == String());
				node_path = path;
				if (fbx_pivot_name.size() == 2) {
					String transform_name = fbx_pivot_name[1].to_lower();
					if (transform_name == "scaling") {
						transform_name = "scale";
					}
					node_path = path + ":" + transform_name;
				}
				_insert_animation_track(p_scene, path, p_bake_fps, animation, ticks_per_second, length, NULL, i, track, node_name, node_path);
			}
		}

		for (int i = 0; i < anim->mNumMorphMeshChannels; i++) {
			const aiMeshMorphAnim *anim_mesh = anim->mMorphMeshChannels[i];
			const String prop_name = _ai_string_to_string(anim_mesh->mName);
			const String mesh_name = prop_name.split("*")[0];
			ERR_CONTINUE(prop_name.split("*").size() != 2);
			const int32_t blend_shape_index = prop_name.split("*")[1].to_int();
			const MeshInstance *mesh_instance = Object::cast_to<MeshInstance>(ap->get_owner()->find_node(mesh_name));
			const String path = ap->get_owner()->get_path_to(mesh_instance);
			ERR_EXPLAIN("Can't find mesh in scene");
			ERR_CONTINUE(path == String())
			Ref<Mesh> mesh = mesh_instance->get_mesh();
			ERR_CONTINUE(mesh.is_null());
			//must bake, apologies.
			float increment = 1.0 / float(p_bake_fps);
			float time = 0.0;

			bool last = false;

			for (size_t k = 0; k < anim_mesh->mNumKeys; k++) {
				Map<int32_t, Vector<real_t> > track_values;
				Map<int32_t, Vector<real_t> > track_times;
				for (size_t j = 0; j < anim_mesh->mKeys[k].mNumValuesAndWeights; j++) {
					const String prop = "blend_shapes/" + mesh->get_blend_shape_name(anim_mesh->mKeys[k].mValues[j]);
					const NodePath node_path = String(path) + ":" + prop;
					int32_t track_idx = -1;
					if (animation->find_track(node_path) == -1) {
						track_idx = animation->get_track_count();
						animation->add_track(Animation::TYPE_VALUE);
						animation->track_set_path(track_idx, node_path);
					} else {
						track_idx = animation->find_track(node_path);
					}
					if (track_values.has(track_idx) == false) {
						track_values.insert(track_idx, Vector<real_t>());
					}
					if (track_times.has(track_idx) == false) {
						track_times.insert(track_idx, Vector<real_t>());
					}
					Vector<real_t> values = track_values[track_idx];
					Vector<real_t> times = track_times[track_idx];
					values.push_back(anim_mesh->mKeys[k].mWeights[j]);
					times.push_back(anim_mesh->mKeys[k].mTime);
					track_values[track_idx] = values;
					track_times[track_idx] = times;
				}

				while (true) {
					for (Map<int32_t, Vector<real_t> >::Element *E = track_values.front(); E; E = E->next()) {
						real_t weight = _interpolate_track<real_t>(track_times[E->key()], track_values[E->key()], time, AssetImportAnimation::INTERP_LINEAR);
						animation->track_insert_key(E->key(), time, weight);
					}
					if (last) {
						break;
					}

					time += increment;
					if (time >= length) {
						last = true;
						time = length;
					}
				}
			}
		}
	}
	animation->set_length(length);
	if (animation->get_track_count()) {
		ap->add_animation(name, animation);
	}
}

Transform EditorSceneImporterAssetImport::_get_global_ai_node_transform(const aiScene *p_scene, const aiNode *p_current_node) {
	aiNode const *current_node = p_current_node;
	Transform xform;
	while (current_node != NULL) {
		xform = _extract_ai_matrix_transform(current_node->mTransformation) * xform;
		current_node = current_node->mParent;
	}
	return xform;
}

void EditorSceneImporterAssetImport::_generate_node_bone(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton) {
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];
		for (int j = 0; j < ai_mesh->mNumBones; j++) {
			String bone_name = _ai_string_to_string(ai_mesh->mBones[j]->mName);
			if (p_skeleton->find_bone(bone_name) != -1) {
				continue;
			}
			p_mesh_bones.insert(bone_name, true);
			p_skeleton->add_bone(bone_name);
			int32_t idx = p_skeleton->find_bone(bone_name);
			Transform xform = _extract_ai_matrix_transform(ai_mesh->mBones[j]->mOffsetMatrix);
			p_skeleton->set_bone_rest(idx, xform.affine_inverse());
		}
	}
}

void EditorSceneImporterAssetImport::_generate_node_bone_parents(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton, const MeshInstance *p_mi) {
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];

		for (int j = 0; j < ai_mesh->mNumBones; j++) {
			aiNode *bone_node = p_scene->mRootNode->FindNode(ai_mesh->mBones[j]->mName);
			ERR_CONTINUE(bone_node == NULL);
			aiNode *bone_node_parent = bone_node->mParent;
			while (bone_node_parent != NULL) {
				String bone_parent_name = _ai_string_to_string(bone_node_parent->mName);

				if (bone_parent_name == p_mi->get_name()) {
					break;
				}
				if (p_mi->get_parent() == NULL) {
					break;
				}
				if (bone_parent_name == p_mi->get_parent()->get_name()) {
					break;
				}
				if (p_skeleton->find_bone(bone_parent_name) == -1) {
					p_mesh_bones.insert(bone_parent_name, true);
				}
				bone_node_parent = bone_node_parent->mParent;
			}
		}
	}
}

void EditorSceneImporterAssetImport::_fill_skeleton(const aiScene *p_scene, aiNode *p_node, Spatial *p_current, Node *p_owner, Skeleton *p_skeleton, const Map<String, bool> p_mesh_bones, const Map<String, Transform> &p_bone_rests, Set<String> p_tracks, const String p_skeleton_root) {
	String node_name = _ai_string_to_string(p_node->mName);

	if ((p_mesh_bones.find(node_name) == NULL || p_mesh_bones.find(node_name)->get() == false)) {
		return;
	} else if (node_name != p_skeleton_root && _ai_find_node(p_scene->mRootNode, p_skeleton_root)->FindNode(p_node->mName) && p_mesh_bones.find(node_name) != NULL && p_skeleton->find_bone(node_name) == -1) {
		p_skeleton->add_bone(node_name);
		int32_t idx = p_skeleton->find_bone(node_name);
		Transform xform = _get_global_ai_node_transform(p_scene, p_node);
		p_skeleton->set_bone_rest(idx, xform);
	}

	for (int i = 0; i < p_node->mNumChildren; i++) {
		_fill_skeleton(p_scene, p_node->mChildren[i], p_current, p_owner, p_skeleton, p_mesh_bones, p_bone_rests, p_tracks, p_skeleton_root);
	}
}

void EditorSceneImporterAssetImport::_generate_node(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_parent, Node *p_owner, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names, Map<Skeleton *, MeshInstance *> &r_skeletons, const Map<String, Transform> &p_bone_rests, Map<MeshInstance *, String> &r_mesh_instances) {
	Spatial *child_node = NULL;
	String node_name = _ai_string_to_string(p_node->mName);
	Skeleton *s = NULL;
	aiNode *ai_skeleton_root = NULL;
	if (p_node->mNumMeshes > 0) {
		child_node = memnew(MeshInstance);
		p_parent->add_child(child_node);
		child_node->set_owner(p_owner);
		String name = _gen_unique_name(node_name, p_owner);
		child_node->set_name(name);
		{
			String node_name = p_parent->get_name();
			Map<String, bool> mesh_bones;
			s = memnew(Skeleton);
			_generate_node_bone(p_scene, p_node, mesh_bones, s);
			Set<String> tracks;
			_get_track_set(p_scene, tracks);
			_generate_node_bone_parents(p_scene, p_node, mesh_bones, s, Object::cast_to<MeshInstance>(child_node));

			if (s->get_bone_count() > 0) {
				aiNode *ai_child_node = p_scene->mRootNode;
				String bone_name = s->get_bone_name(0);
				ai_skeleton_root = _ai_find_node(ai_child_node, bone_name);
			}
			if (ai_skeleton_root != NULL) {
				Map<String, bool>::Element *E = mesh_bones.find(_ai_string_to_string(ai_skeleton_root->mName));
				while (ai_skeleton_root && E && ai_skeleton_root->mParent) {
					E = mesh_bones.find(_ai_string_to_string(ai_skeleton_root->mParent->mName));
					if (E == NULL || ai_skeleton_root->mParent->mName == p_scene->mRootNode->mName) {
						break;
					}
					ai_skeleton_root = p_scene->mRootNode->FindNode(ai_skeleton_root->mName)->mParent;
				}
			}
			if (ai_skeleton_root == NULL) {
				ai_skeleton_root = p_scene->mRootNode->FindNode(p_node->mName);
				while (ai_skeleton_root && ai_skeleton_root->mParent && ai_skeleton_root->mParent != p_scene->mRootNode) {
					ai_skeleton_root = p_scene->mRootNode->FindNode(ai_skeleton_root->mName)->mParent;
				}
			}

			if (s->get_bone_count() > 0) {
				_fill_skeleton(p_scene, ai_skeleton_root, child_node, p_owner, s, mesh_bones, p_bone_rests, tracks, _ai_string_to_string(ai_skeleton_root->mName));
				r_skeletons.insert(s, Object::cast_to<MeshInstance>(child_node));
				String skeleton_path = s->get_name();
				Object::cast_to<MeshInstance>(child_node)->set_skeleton_path(skeleton_path);
			}
			if (ai_skeleton_root != NULL) {
				r_mesh_instances.insert(Object::cast_to<MeshInstance>(child_node), _ai_string_to_string(ai_skeleton_root->mName));
			} else {
				r_mesh_instances.insert(Object::cast_to<MeshInstance>(child_node), "");
			}
			_add_mesh_to_mesh_instance(p_node, p_scene, s, p_path, Object::cast_to<MeshInstance>(child_node), p_owner, r_bone_name);
		}
	} else if (p_light_names.has(node_name)) {
		Spatial *light = Object::cast_to<Light>(p_owner->find_node(node_name));
		ERR_FAIL_COND(light == NULL);
		p_parent->add_child(light);
		light->set_owner(p_owner);
		light->get_parent()->remove_child(light);
		String name = _gen_unique_name(node_name, p_owner);
		light->set_name(name);
		child_node = light;
	} else if (p_camera_names.has(node_name)) {
		Spatial *camera = Object::cast_to<Camera>(p_owner->find_node(node_name));
		ERR_FAIL_COND(camera == NULL);
		p_parent->add_child(camera);
		camera->set_owner(p_owner);
		camera->get_parent()->remove_child(camera);
		String name = _gen_unique_name(node_name, p_owner);
		camera->set_name(name);
		child_node = camera;
	} else {
		child_node = memnew(Spatial);
		p_parent->add_child(child_node);
		child_node->set_owner(p_owner);
		String name = _gen_unique_name(node_name, p_owner);
		child_node->set_name(name);
	}

	ERR_FAIL_COND(child_node == NULL);

	MeshInstance *mi = Object::cast_to<MeshInstance>(child_node);
	if (mi != NULL && s != NULL) {
		if (s->get_bone_count() > 0) {
			s->set_name(node_name + TTR("Skeleton"));
			mi->add_child(s);
			s->set_owner(p_owner);
			mi->set_skeleton_path(NodePath(s->get_name()));
		}
	}

	Transform xform = _extract_ai_matrix_transform(p_node->mTransformation);
	child_node->set_transform(xform * child_node->get_transform());
	for (int i = 0; i < p_node->mNumChildren; i++) {
		_generate_node(p_path, p_scene, p_node->mChildren[i], child_node, p_owner, r_bone_name, p_light_names, p_camera_names, r_skeletons, p_bone_rests, r_mesh_instances);
	}
}

aiNode *EditorSceneImporterAssetImport::_ai_find_node(aiNode *ai_child_node, const String bone_name) {

	if (_ai_string_to_string(ai_child_node->mName) == bone_name) {
		return ai_child_node;
	}
	aiNode *target = NULL;
	for (int i = 0; i < ai_child_node->mNumChildren; i++) {

		target = _ai_find_node(ai_child_node->mChildren[i], bone_name);
		if (target != NULL) {
			return target;
		}
	}
	return target;
}

Transform EditorSceneImporterAssetImport::_format_xform(const String p_path, const aiScene *p_scene) {
	String ext = p_path.get_file().get_extension().to_lower();
	if (!(ext == "glb" || ext == "gltf" || ext == "fbx")) {
		return Transform();
	}
	Quat quat;
	quat.set_euler(Vector3(Math::deg2rad(-90.f), 0.0f, 0.0f));
	Transform xform;
	real_t factor = 1.0f;
	if (p_scene->mMetaData != NULL) {
		p_scene->mMetaData->Get("UnitScaleFactor", factor);
	}
	xform.basis.set_quat_scale(quat, Vector3(factor * 0.01f, factor * 0.01f, factor * 0.01f));

	return xform;
}

String EditorSceneImporterAssetImport::_gen_unique_name(String node_name, Node *p_owner) {
	String name;
	int index = 1;
	while (true) {

		name = node_name;
		if (index > 1) {
			name += " " + itos(index);
		}
		if (p_owner->find_node(name) == NULL) {
			break;
		}
		index++;
	}
	return name;
}

void EditorSceneImporterAssetImport::_move_mesh(const String p_path, const aiScene *p_scene, Node *p_current, Node *p_owner, Map<MeshInstance *, String> &p_mesh_instances, Map<Skeleton *, MeshInstance *> &p_skeletons) {

	for (Map<MeshInstance *, String>::Element *E = p_mesh_instances.front(); E; E = E->next()) {
		Spatial *skeleton_root = Object::cast_to<Spatial>(p_owner->find_node(E->get()));
		if (skeleton_root == NULL) {
			continue;
		}

		if (skeleton_root == p_owner) {
			continue;
		}

		Spatial *mesh = E->key();
		if (skeleton_root == mesh) {
			continue;
		}

		bool is_inside_armature = (skeleton_root->is_a_parent_of(E->key())) != NULL;
		if (is_inside_armature) {
			continue;
		}
		for (Map<Skeleton *, MeshInstance *>::Element *F = p_skeletons.front(); F; F = F->next()) {
			if (E->key() != F->get()) {
				continue;
			}
			Node *mesh_bone_root = NULL;
			for (size_t i = 0; i < F->key()->get_bone_count(); i++) {
				if (F->key()->get_bone_parent(i) == -1) {
					mesh_bone_root = p_owner->find_node(F->key()->get_bone_name(i));
					break;
				}
			}
			if (mesh_bone_root != NULL) {
				mesh->get_parent()->remove_child(mesh);
				mesh_bone_root->add_child(mesh);
				mesh->set_owner(p_owner);
				Transform skeleton_root_parent_global_xform = _get_global_ai_node_transform(p_scene, _ai_find_node(p_scene->mRootNode, mesh_bone_root->get_name()));
				mesh->set_transform(skeleton_root_parent_global_xform.affine_inverse() * mesh->get_transform());
			}
			F->key()->get_parent()->remove_child(F->key());
			mesh->add_child(F->key());
			F->key()->set_owner(p_owner);
			NodePath skeleton_path = String(F->get()->get_path_to(p_owner)) + "/" + p_owner->get_path_to(F->key());
			F->get()->set_skeleton_path(String(F->key()->get_name()));
			if (p_path.get_file().get_extension().to_lower() == "glb" || p_path.get_file().get_extension().to_lower() == "gltf") {
				Transform xform = mesh->get_transform().scaled(Vector3(0.5f, 0.5f, 0.5f));
				//Where does this come from?
				xform.basis.set_quat_scale(Quat(), xform.basis.get_scale());
				F->key()->set_transform(xform);
			}
		}
	}
}

void EditorSceneImporterAssetImport::_get_track_set(const aiScene *p_scene, Set<String> &tracks) {
	for (size_t i = 0; i < p_scene->mNumAnimations; i++) {
		for (size_t j = 0; j < p_scene->mAnimations[i]->mNumChannels; j++) {
			aiString ai_name = p_scene->mAnimations[i]->mChannels[j]->mNodeName;
			String name = _ai_string_to_string(ai_name);
			tracks.insert(name);
		}
	}
}

void EditorSceneImporterAssetImport::_add_mesh_to_mesh_instance(const aiNode *p_node, const aiScene *p_scene, Skeleton *s, const String &p_path, MeshInstance *p_mesh_instance, Node *p_owner, Set<String> &r_bone_name) {
	Ref<ArrayMesh> mesh;
	mesh.instance();
	bool has_uvs = false;
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];

		Ref<SurfaceTool> st;
		st.instance();
		st->begin(Mesh::PRIMITIVE_TRIANGLES);
		Map<uint32_t, Vector<float> > vertex_weight;
		Map<uint32_t, Vector<String> > vertex_bone_name;
		for (size_t b = 0; b < ai_mesh->mNumBones; b++) {
			aiBone *bone = ai_mesh->mBones[b];
			for (size_t w = 0; w < bone->mNumWeights; w++) {
				String name = _ai_string_to_string(bone->mName);
				aiVertexWeight ai_weights = bone->mWeights[w];
				uint32_t vertexId = ai_weights.mVertexId;
				Map<uint32_t, Vector<float> >::Element *result = vertex_weight.find(vertexId);
				Vector<float> weights;
				if (result != NULL) {
					weights.append_array(result->value());
				}
				weights.push_back(ai_weights.mWeight);
				if (vertex_weight.has(vertexId)) {
					vertex_weight[vertexId] = weights;
				} else {
					vertex_weight.insert(vertexId, weights);
				}
				Map<uint32_t, Vector<String> >::Element *bone_result = vertex_bone_name.find(vertexId);
				Vector<String> bone_names;
				if (bone_result != NULL) {
					bone_names.append_array(bone_result->value());
				}
				bone_names.push_back(name);
				if (vertex_bone_name.has(vertexId)) {
					vertex_bone_name[vertexId] = bone_names;
				} else {
					vertex_bone_name.insert(vertexId, bone_names);
				}
			}
		}
		for (size_t j = 0; j < ai_mesh->mNumFaces; j++) {
			const aiFace face = ai_mesh->mFaces[j];
			for (size_t k = 0; k < face.mNumIndices; k++) {
				unsigned int index = face.mIndices[k];
				if (ai_mesh->HasTextureCoords(0)) {
					has_uvs = true;
					st->add_uv(Vector2(ai_mesh->mTextureCoords[0][index].x, 1.0f - ai_mesh->mTextureCoords[0][index].y));
				}
				if (ai_mesh->HasTextureCoords(1)) {
					has_uvs = true;
					st->add_uv2(Vector2(ai_mesh->mTextureCoords[1][index].x, 1.0f - ai_mesh->mTextureCoords[1][index].y));
				}
				if (ai_mesh->HasVertexColors(0)) {
					Color color = Color(ai_mesh->mColors[0]->r, ai_mesh->mColors[0]->g, ai_mesh->mColors[0]->b, ai_mesh->mColors[0]->a);
					st->add_color(color);
				}
				if (ai_mesh->mNormals != NULL) {
					const aiVector3D normals = ai_mesh->mNormals[index];
					const Vector3 godot_normal = Vector3(normals.x, normals.y, normals.z);
					st->add_normal(godot_normal);
					if (ai_mesh->HasTangentsAndBitangents()) {
						const aiVector3D tangents = ai_mesh->mTangents[index];
						const Vector3 godot_tangent = Vector3(tangents.x, tangents.y, tangents.z);
						const aiVector3D bitangent = ai_mesh->mBitangents[index];
						const Vector3 godot_bitangent = Vector3(bitangent.x, bitangent.y, bitangent.z);
						float d = godot_normal.cross(godot_tangent).dot(godot_bitangent) > 0.0f ? 1.0f : -1.0f;
						st->add_tangent(Plane(tangents.x, tangents.y, tangents.z, d));
					}
				}

				if (s != NULL && s->get_bone_count() > 0) {
					Map<uint32_t, Vector<String> >::Element *I = vertex_bone_name.find(index);
					Vector<int32_t> bones;

					if (I != NULL) {
						Vector<String> bone_names;
						bone_names.append_array(I->value());
						for (size_t f = 0; f < bone_names.size(); f++) {
							int32_t bone = s->find_bone(bone_names[f]);
							ERR_EXPLAIN("Asset Importer: Mesh can't find bone " + bone_names[f]);
							ERR_FAIL_COND(bone == -1);
							bones.push_back(bone);
						}
						if (s->get_bone_count()) {
							int32_t add = CLAMP(VS::ARRAY_WEIGHTS_SIZE - bones.size(), 0, VS::ARRAY_WEIGHTS_SIZE);
							for (size_t f = 0; f < add; f++) {
								bones.push_back(s->get_bone_count() - 1);
							}
						}
						st->add_bones(bones);
						Map<uint32_t, Vector<float> >::Element *E = vertex_weight.find(index);
						Vector<float> weights;
						if (E != NULL) {
							weights = E->value();
							if (weights.size() < VS::ARRAY_WEIGHTS_SIZE) {
								int32_t add = CLAMP(VS::ARRAY_WEIGHTS_SIZE - weights.size(), 0, VS::ARRAY_WEIGHTS_SIZE);
								for (size_t f = 0; f < add; f++) {
									weights.push_back(0.0f);
								}
							}
						}
						ERR_CONTINUE(weights.size() == 0);
						st->add_weights(weights);
					}
				}
				const aiVector3D pos = ai_mesh->mVertices[index];
				Vector3 godot_pos = Vector3(pos.x, pos.y, pos.z);
				st->add_vertex(godot_pos);
			}
		}
		if (ai_mesh->mNumAnimMeshes == 0) {
			st->index();
			if (ai_mesh->HasTangentsAndBitangents() == false && has_uvs) {
				st->generate_tangents();
			}
		}

		aiMaterial *ai_material = p_scene->mMaterials[ai_mesh->mMaterialIndex];
		Ref<SpatialMaterial> mat;
		mat.instance();
		mat->set_name(_ai_string_to_string(ai_material->GetName()));

		int32_t mat_two_sided = 0;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_TWOSIDED, mat_two_sided)) {
			if (mat_two_sided > 0) {
				mat->set_cull_mode(SpatialMaterial::CULL_DISABLED);
			}
		}

		const String mesh_name = _ai_string_to_string(ai_mesh->mName);
		aiString mat_name;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_NAME, mat_name)) {
			mat->set_name(_ai_string_to_string(mat_name));
		}
		aiColor3D clr_diffuse;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_COLOR_DIFFUSE, clr_diffuse)) {
			mat->set_albedo(Color(clr_diffuse.r, clr_diffuse.g, clr_diffuse.b));
		}
		aiColor4D pbr_base_color;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_FACTOR, pbr_base_color)) {
			mat->set_albedo(Color(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a));
		}
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_PBSMETALLICROUGHNESS_BASE_COLOR_FACTOR, pbr_base_color)) {
			mat->set_albedo(Color(pbr_base_color.r, pbr_base_color.g, pbr_base_color.b, pbr_base_color.a));
		}

		float pbr_metallic = 0.0f;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLIC_FACTOR, pbr_metallic)) {
			mat->set_metallic(pbr_metallic);
		}
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_PBSMETALLICROUGHNESS_METALLIC_FACTOR, pbr_metallic)) {
			mat->set_metallic(pbr_metallic);
		}

		float pbr_roughness = 0.0f;
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_ROUGHNESS_FACTOR, pbr_roughness)) {
			mat->set_roughness(pbr_roughness);
		}
		if (AI_SUCCESS == ai_material->Get(AI_MATKEY_FBX_PBSMETALLICROUGHNESS_ROUGHNESS_FACTOR, pbr_roughness)) {
			mat->set_roughness(pbr_roughness);
		}

		aiTextureType tex_normal = aiTextureType_NORMALS;
		{
			aiString ai_filename = aiString();
			String filename = "";
			aiTextureMapMode map_mode[2];

			if (ai_material->GetTexture(tex_normal, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode) == AI_SUCCESS) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = ResourceLoader::load(path, "Texture");
					if (texture != NULL) {
						if (map_mode != NULL) {
							_set_texture_mapping_mode(map_mode, texture);
						}
						mat->set_feature(SpatialMaterial::Feature::FEATURE_NORMAL_MAPPING, true);
						mat->set_texture(SpatialMaterial::TEXTURE_NORMAL, texture);
					}
				}
			}
		}

		aiTextureType tex_fbx_maya_normal = aiTextureType_NORMALS;
		{
			aiString ai_filename = aiString();
			String filename = "";
			aiTextureMapMode map_mode[2];

			if (ai_material->GetTexture(tex_fbx_maya_normal, 1, &ai_filename, NULL, NULL, NULL, NULL, map_mode) == AI_SUCCESS) {
				filename = _ai_raw_string_to_string(ai_filename);
				String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
				bool found = false;
				_find_texture_path(p_path, path, found);
				if (found) {
					Ref<Texture> texture = ResourceLoader::load(path, "Texture");
					if (texture != NULL) {
						if (map_mode != NULL) {
							_set_texture_mapping_mode(map_mode, texture);
						}
						mat->set_feature(SpatialMaterial::Feature::FEATURE_NORMAL_MAPPING, true);
						mat->set_texture(SpatialMaterial::TEXTURE_NORMAL, texture);
					}
				}
			}
		}

		aiTextureType tex_emissive = aiTextureType_EMISSIVE;
		{
			if (ai_material->GetTextureCount(tex_emissive) > 0) {

				aiString ai_filename = aiString();
				String filename = "";
				aiTextureMapMode map_mode[2];

				if (ai_material->GetTexture(tex_emissive, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode) == AI_SUCCESS) {
					filename = _ai_raw_string_to_string(ai_filename);
					String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
					bool found = false;
					_find_texture_path(p_path, path, found);
					if (found) {
						Ref<Texture> texture = ResourceLoader::load(path, "Texture");
						if (texture != NULL) {
							_set_texture_mapping_mode(map_mode, texture);
							mat->set_texture(SpatialMaterial::TEXTURE_EMISSION, texture);
						}
					}
				}
			}
		}

		aiTextureType tex_albedo = aiTextureType_DIFFUSE;
		{
			if (ai_material->GetTextureCount(tex_albedo) > 0) {

				aiString ai_filename = aiString();
				String filename = "";
				aiTextureMapMode map_mode[2];
				if (ai_material->GetTexture(tex_albedo, 0, &ai_filename, NULL, NULL, NULL, NULL, map_mode) == AI_SUCCESS) {
					filename = _ai_raw_string_to_string(ai_filename);
					String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
					bool found = false;
					_find_texture_path(p_path, path, found);
					if (found) {
						Ref<Texture> texture = ResourceLoader::load(path, "Texture");
						if (texture != NULL) {
							if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
								_set_texture_mapping_mode(map_mode, texture);
								mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
								mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
							}
							mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
						}
					}
				}
			}
		}

		aiString tex_gltf_base_color_path = aiString();
		aiTextureMapMode map_mode[2];
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_BASE_COLOR_TEXTURE, &tex_gltf_base_color_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_gltf_base_color_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = ResourceLoader::load(path, "Texture");
				_find_texture_path(p_path, path, found);
				aiTextureMapMode *map_mode = NULL;
				if (texture != NULL) {
					if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
						_set_texture_mapping_mode(map_mode, texture);
						mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
						mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
					}
					mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
				}
			}
		}

		aiString tex_fbx_pbs_base_color_path = aiString();
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_FBX_PBSMETALLICROUGNESS_BASE_COLOR_TEXTURE, &tex_fbx_pbs_base_color_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_fbx_pbs_base_color_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = ResourceLoader::load(path, "Texture");
				_find_texture_path(p_path, path, found);
				aiTextureMapMode *map_mode = NULL;
				if (texture != NULL) {
					if (texture->get_data()->detect_alpha() == Image::ALPHA_BLEND) {
						_set_texture_mapping_mode(map_mode, texture);
						mat->set_feature(SpatialMaterial::FEATURE_TRANSPARENT, true);
						mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
					}
					mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
				}
			}
		}

		aiString tex_gltf_pbr_metallicroughness_path;
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_GLTF_PBRMETALLICROUGHNESS_METALLICROUGHNESS_TEXTURE, &tex_gltf_pbr_metallicroughness_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_gltf_pbr_metallicroughness_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = ResourceLoader::load(path, "Texture");
				if (texture != NULL) {
					_set_texture_mapping_mode(map_mode, texture);
					mat->set_texture(SpatialMaterial::TEXTURE_METALLIC, texture);
					mat->set_metallic_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_BLUE);
					mat->set_texture(SpatialMaterial::TEXTURE_ROUGHNESS, texture);
					mat->set_roughness_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GREEN);
				}
			}
		}

		aiString tex_fbx_pbs_metallic_path;
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_FBX_PBSMETALLICROUGHNESS_METALLIC_TEXTURE, &tex_fbx_pbs_metallic_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_fbx_pbs_metallic_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = ResourceLoader::load(path, "Texture");
				if (texture != NULL) {
					_set_texture_mapping_mode(map_mode, texture);
					mat->set_texture(SpatialMaterial::TEXTURE_METALLIC, texture);
					mat->set_metallic_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
				}
			}
		}

		aiString tex_fbx_pbs_rough_path;
		if (AI_SUCCESS == ai_material->GetTexture(AI_MATKEY_FBX_PBSMETALLICROUGHNESS_ROUGHNESS_TEXTURE, &tex_fbx_pbs_rough_path, NULL, NULL, NULL, NULL, map_mode)) {
			String filename = _ai_raw_string_to_string(tex_fbx_pbs_rough_path);
			String path = p_path.get_base_dir() + "/" + filename.replace("\\", "/");
			bool found = false;
			_find_texture_path(p_path, path, found);
			if (found) {
				Ref<Texture> texture = ResourceLoader::load(path, "Texture");
				if (texture != NULL) {
					_set_texture_mapping_mode(map_mode, texture);
					mat->set_texture(SpatialMaterial::TEXTURE_ROUGHNESS, texture);
					mat->set_roughness_texture_channel(SpatialMaterial::TEXTURE_CHANNEL_GRAYSCALE);
				}
			}
		}

		Array array_mesh = st->commit_to_arrays();
		Array morphs;
		Mesh::PrimitiveType primitive = Mesh::PRIMITIVE_TRIANGLES;

		for (int i = 0; i < ai_mesh->mNumAnimMeshes; i++) {

			String ai_anim_mesh_name = _ai_string_to_string(ai_mesh->mAnimMeshes[i]->mName);
			mesh->set_blend_shape_mode(Mesh::BLEND_SHAPE_MODE_NORMALIZED);
			if (ai_anim_mesh_name.empty()) {
				ai_anim_mesh_name = String("morph_") + itos(i);
			}
			mesh->add_blend_shape(ai_anim_mesh_name);

			Array array_copy;
			array_copy.resize(Mesh::ARRAY_MAX);

			for (int l = 0; l < Mesh::ARRAY_MAX; l++) {
				array_copy[l] = array_mesh[l];
			}

			const uint32_t num_vertices = ai_mesh->mAnimMeshes[i]->mNumVertices;

			array_copy[Mesh::ARRAY_INDEX] = Variant();
			if (ai_mesh->mAnimMeshes[i]->HasPositions()) {
				PoolVector3Array vertices;
				vertices.resize(num_vertices);
				for (int l = 0; l < num_vertices; l++) {
					const aiVector3D pos = ai_mesh->mAnimMeshes[i]->mVertices[l];
					Vector3 positions = Vector3(pos.x, pos.y, -pos.z);
					PoolVector3Array::Write w = vertices.write();
					w[l] = positions;
				}
				PoolVector3Array original_vertices = array_copy[Mesh::ARRAY_VERTEX];

				for (size_t k = 0; k < vertices.size(); k++) {
					PoolVector3Array::Write w = vertices.write();
					Vector3 temp = w[k]; 
					w[k] = vertices[vertices.size() - 1 - k];
					w[vertices.size() - 1 - k] = temp;
				}

				for (int l = 0; l < vertices.size(); l++) {
					PoolVector3Array::Write w = original_vertices.write();
					w[l] = vertices[l];
				}
				array_copy[Mesh::ARRAY_VERTEX] = original_vertices;
			}

			//if (ai_mesh->mAnimMeshes[i]->HasVertexColors(0)) {
			//	PoolColorArray colors = array_copy[Mesh::ARRAY_COLOR];
			//	//colors.resize(num_vertices);
			//	for (int l = 0; l < num_vertices / 2; l++) {
			//		const aiColor4D ai_color = ai_mesh->mAnimMeshes[i]->mColors[0][num_vertices - 1 - l];
			//		Color color = Color(ai_color.r, ai_color.g, ai_color.b, ai_color.a);
			//		PoolColorArray::Write w = colors.write();
			//		w[l] = color;
			//	}
			//	array_copy[Mesh::ARRAY_COLOR] = colors;
			//}

			//if (ai_mesh->mAnimMeshes[i]->HasNormals()) {
			//	PoolVector3Array normals = array_copy[Mesh::ARRAY_NORMAL];
			//	//normals.resize(num_vertices);
			//	for (int l = 0; l < num_vertices / 2; l++) {
			//		const aiVector3D normal = ai_mesh->mAnimMeshes[i]->mNormals[num_vertices - 1 - l];
			//		Vector3 godot_normals = Vector3(normal.x, normal.y, normal.z);
			//		PoolVector3Array::Write w = normals.write();
			//		w[l] = godot_normals;
			//	}
			//	array_copy[Mesh::ARRAY_NORMAL] = normals;
			//}
			//if (ai_mesh->mAnimMeshes[i]->HasTangentsAndBitangents()) {
			//	PoolColorArray tangents = array_copy[Mesh::ARRAY_TANGENT];
			//	//tangents.resize(num_vertices);
			//	for (int l = 0; l < num_vertices / 2; l++) {
			//		const aiVector3D normals = ai_mesh->mAnimMeshes[i]->mNormals[num_vertices - 1 - l];
			//		const Vector3 godot_normal = Vector3(normals.x, normals.y, normals.z);
			//		const aiVector3D tangent = ai_mesh->mAnimMeshes[i]->mTangents[num_vertices - 1 - l];
			//		const Vector3 godot_tangent = Vector3(tangent.x, tangent.y, tangent.z);
			//		const aiVector3D bitangent = ai_mesh->mAnimMeshes[i]->mBitangents[num_vertices - 1 - l];
			//		const Vector3 godot_bitangent = Vector3(bitangent.x, bitangent.y, bitangent.z);
			//		float d = godot_normal.cross(godot_tangent).dot(godot_bitangent) > 0.0f ? 1.0f : -1.0f;

			//		Color plane_tangent = Color(tangent.x, tangent.y, tangent.z, d);
			//		PoolColorArray::Write w = tangents.write();
			//		w[l] = plane_tangent;
			//	}
			//	array_copy[Mesh::ARRAY_TANGENT] = tangents;
			//}
			morphs.push_back(array_copy);
		}

		mesh->add_surface_from_arrays(primitive, array_mesh, morphs);
		mesh->surface_set_material(i, mat);
		mesh->surface_set_name(i, _ai_string_to_string(ai_mesh->mName));
		print_line(String("Created mesh ") + _ai_string_to_string(ai_mesh->mName) + " " + itos(mesh_idx + 1) + " of " + itos(p_scene->mNumMeshes));
	}
	p_mesh_instance->set_mesh(mesh);
}

void EditorSceneImporterAssetImport::_set_texture_mapping_mode(aiTextureMapMode *map_mode, Ref<Texture> texture) {
	ERR_FAIL_COND(map_mode == NULL);
	aiTextureMapMode tex_mode = aiTextureMapMode::aiTextureMapMode_Wrap;
	//for (size_t i = 0; i < 3; i++) {
	tex_mode = map_mode[0];
	//}
	int32_t flags = Texture::FLAGS_DEFAULT;
	if (tex_mode == aiTextureMapMode_Wrap) {
		//Default
	} else if (tex_mode == aiTextureMapMode_Clamp) {
		flags = flags & ~Texture::FLAG_REPEAT;
	} else if (tex_mode == aiTextureMapMode_Mirror) {
		flags = flags | Texture::FLAG_MIRRORED_REPEAT;
	}
	texture->set_flags(flags);
}

void EditorSceneImporterAssetImport::_find_texture_path(const String &r_p_path, String &r_path, bool &r_found) {

	_Directory dir;

	Vector<String> exts;
	exts.push_back(".jpg");
	exts.push_back(".jpeg");
	exts.push_back(".png");
	exts.push_back(".exr");
	exts.push_back(".tga");
	exts.push_back(".dds");

	if (dir.file_exists(r_p_path.get_base_dir() + r_path)) {
		r_path = r_p_path.get_base_dir() + r_path;
		r_found = true;
		return;
	}

	for (size_t i = 0; i < exts.size(); i++) {
		if (r_found) {
			return;
		}
		if (r_found == false) {
			_find_texture_path(r_p_path, dir, r_path, r_found, exts[i]);
		}
	}
}

void EditorSceneImporterAssetImport::_find_texture_path(const String &p_path, _Directory &dir, String &path, bool &found, String extension) {
	String name = path.get_basename() + extension;
	if (dir.file_exists(name)) {
		found = true;
		path = name;
		return;
	}
	String name_ignore_sub_directory = p_path.get_base_dir() + "/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_ignore_sub_directory)) {
		found = true;
		path = name_ignore_sub_directory;
		return;
	}

	String name_find_texture_sub_directory = p_path.get_base_dir() + "/textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_sub_directory)) {
		found = true;
		path = name_find_texture_sub_directory;
		return;
	}
	String name_find_texture_upper_sub_directory = p_path.get_base_dir() + "/Textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_upper_sub_directory)) {
		found = true;
		path = name_find_texture_upper_sub_directory;
		return;
	}
	String name_find_texture_outside_sub_directory = p_path.get_base_dir() + "/../textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_texture_outside_sub_directory)) {
		found = true;
		path = name_find_texture_outside_sub_directory;
		return;
	}

	String name_find_upper_texture_outside_sub_directory = p_path.get_base_dir() + "/../Textures/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_find_upper_texture_outside_sub_directory)) {
		found = true;
		path = name_find_upper_texture_outside_sub_directory;
		return;
	}
}

String EditorSceneImporterAssetImport::_ai_string_to_string(const aiString p_string) {
	Vector<char> raw_name;
	raw_name.resize(p_string.length);
	memcpy(raw_name.ptrw(), p_string.C_Str(), p_string.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	if (name.find(":") != -1) {
		String replaced_name = name.replace(":", "_");
		print_verbose("Replacing " + name + " containing : with " + replaced_name);
		name = replaced_name;
	}
	if (name.find(".") != -1) {
		String replaced_name = name.replace(".", "");
		print_verbose("Replacing " + name + " containing . with " + replaced_name);
		name = replaced_name;
	}
	return name;
}

String EditorSceneImporterAssetImport::_ai_raw_string_to_string(const aiString p_string) {
	Vector<char> raw_name;
	raw_name.resize(p_string.length);
	memcpy(raw_name.ptrw(), p_string.C_Str(), p_string.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	return name;
}

Ref<Animation> EditorSceneImporterAssetImport::import_animation(const String &p_path, uint32_t p_flags, int p_bake_fps) {
	return Ref<Animation>();
}

const Transform EditorSceneImporterAssetImport::_extract_ai_matrix_transform(const aiMatrix4x4 p_matrix) {
	aiMatrix4x4 matrix = p_matrix;
	Transform xform;
	xform.set(matrix.a1, matrix.b1, matrix.c1, matrix.a2, matrix.b2, matrix.c2, matrix.a3, matrix.b3, matrix.c3, matrix.a4, matrix.b4, matrix.c4);
	xform.basis.inverse();
	xform.basis.transpose();
	xform.orthonormalize();
	return xform;
}
