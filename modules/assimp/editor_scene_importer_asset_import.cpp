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

#include "thirdparty/assimp/include/assimp/Importer.hpp"
#include "thirdparty/assimp/include/assimp/SceneCombiner.h"
#include "thirdparty/assimp/include/assimp/cexport.h"
#include "thirdparty/assimp/include/assimp/cimport.h"
#include "thirdparty/assimp/include/assimp/matrix4x4.h"
#include "thirdparty/assimp/include/assimp/postprocess.h"
#include "thirdparty/assimp/include/assimp/scene.h"
#include <thirdparty/assimp/include/assimp/DefaultLogger.hpp>
#include <thirdparty/assimp/include/assimp/LogStream.hpp>
#include <thirdparty/assimp/include/assimp/Logger.hpp>

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
	importer.SetPropertyBool(AI_CONFIG_IMPORT_FBX_PRESERVE_PIVOTS, true);
	importer.SetPropertyInteger(AI_CONFIG_PP_SBP_REMOVE, aiPrimitiveType_LINE | aiPrimitiveType_POINT);
	//importer.SetPropertyFloat(AI_CONFIG_PP_DB_THRESHOLD, 1.0f);
	int32_t post_process_Steps = aiProcess_CalcTangentSpace |
								 //aiProcess_FlipUVs |
								 //aiProcess_FlipWindingOrder |
								 aiProcess_GenSmoothNormals |
								 aiProcess_JoinIdenticalVertices |
								 aiProcess_ImproveCacheLocality |
								 aiProcess_LimitBoneWeights |
								 aiProcess_RemoveRedundantMaterials |
								 aiProcess_SplitLargeMeshes |
								 aiProcess_Triangulate |
								 aiProcess_GenUVCoords |
								 //aiProcess_SortByPType |
								 aiProcess_FindDegenerates |
								 aiProcess_FindInvalidData |
								 aiProcess_TransformUVCoords |
								 aiProcess_FindInstances |
								 aiProcess_FixInfacingNormals |
								 //aiProcess_ValidateDataStructure |
								 aiProcess_OptimizeMeshes |
								 //aiProcess_OptimizeGraph |
								 //aiProcess_Debone |
								 aiProcess_EmbedTextures |
								 aiProcess_SplitByBoneCount |
								 0;
	const aiScene *scene = importer.ReadFile(s_path.c_str(),
			post_process_Steps);
	ERR_EXPLAIN(String("Open Asset Importer failed to open: ") + String(importer.GetErrorString()));
	Node *node = _generate_scene(p_path, scene, p_flags, p_bake_fps);
	return node;
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
	//TODO RESTORE OTHER TYPES
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

	Spatial *root = memnew(Spatial);
	//For all cameras
	//ERR_FAIL_INDEX(i, state.cameras.size());
	//Camera *camera = memnew(Camera);

	//if (c.perspective) {
	//	camera->set_perspective(c.fov_size, c.znear, c.znear);
	//} else {
	//	camera->set_orthogonal(c.fov_size, c.znear, c.znear);
	//}

	// For all lights
	//Light *light = memnew(Light);
	//ERR_FAIL_INDEX(i, lights.size());

	Vector3 scale = Vector3(100.0f, 100.0f, 100.0f);
	ERR_FAIL_COND_V(scene == NULL, NULL);
	if (scene->mMetaData != NULL) {
		float unit_scale_factor = 1.0f;
		scene->mMetaData->Get("UnitScaleFactor", unit_scale_factor);
		scale = Vector3(unit_scale_factor, unit_scale_factor, unit_scale_factor) * scale;
	}
	scale = Vector3(1.0f, 1.0f, 1.0f) / scale;
	if (p_path.get_extension() == String("fbx")) {
		//root->set_rotation_degrees(Vector3(90.0f, 0.f, 0.0f));
		root->scale(scale);
	}
	Vector<Skeleton *> skeletons;
	Set<String> bone_names;
	Set<String> light_names;
	Set<String> camera_names;
	for (size_t l = 0; l < scene->mNumLights; l++) {
		light_names.insert(_ai_string_to_string(scene->mLights[l]->mName));
	}
	for (size_t c = 0; c < scene->mNumCameras; c++) {
		camera_names.insert(_ai_string_to_string(scene->mCameras[c]->mName));
	}

	Skeleton *s = memnew(Skeleton);
	skeletons.push_back(s);

	_generate_node_bone(p_path, scene, scene->mRootNode, root, skeletons, bone_names, light_names, camera_names);
	_generate_node(p_path, scene, scene->mRootNode, root, root, skeletons, bone_names, light_names, camera_names);
	_add_armature_transform_mi(p_path, scene, root, root, skeletons, bone_names);

	for (size_t i = 0; i < skeletons.size(); i++) {
		for (size_t j = 0; j < skeletons[i]->get_bone_count(); j++) {
			String bone_name = skeletons[i]->get_bone_name(j);
			int32_t node_parent_index = -1;
			const aiNode *bone_node = scene->mRootNode->FindNode(_string_to_ai_string(bone_name));
			if (bone_node != NULL && bone_node->mParent != NULL) {
				node_parent_index = skeletons[i]->find_bone(_ai_string_to_string(bone_node->mParent->mName));
			}
			skeletons[i]->set_bone_parent(j, node_parent_index);
		}
		skeletons[i]->localize_rests();
	}

	const bool is_clear_bones = false;
	if (is_clear_bones) {
		for (size_t i = 0; i < scene->mNumMeshes; i++) {
			for (size_t j = 0; j < scene->mMeshes[i]->mNumBones; j++) {
				Node *node = root->find_node(_ai_string_to_string(scene->mMeshes[i]->mBones[j]->mName));
				if (node != NULL) {
					node->get_parent()->remove_child(node);
				}
			}
		}
	}

	AnimationPlayer *ap = memnew(AnimationPlayer);
	root->add_child(ap);
	ap->set_owner(root);
	ap->set_name(TTR("AnimationPlayer"));

	if (scene->mNumAnimations == 0) {
		_import_animation(scene, ap, -1, p_bake_fps, skeletons);
	} else {
		for (int i = 0; i < scene->mNumAnimations; i++) {
			_import_animation(scene, ap, i, p_bake_fps, skeletons);
		}
	}
	List<StringName> animation_names;
	ap->get_animation_list(&animation_names);
	if (animation_names.size() == 0) {
		root->remove_child(ap);
	}

	return root;
}

Spatial *EditorSceneImporterAssetImport::_find_armature(const aiScene *scene, const Skeleton *s, const Set<String> bone_names) {

	aiNode *current_bone = scene->mRootNode->FindNode(_string_to_ai_string(s->get_bone_name(s->get_bone_count() - 1)));
	aiNode *armature_node = NULL;
	while (current_bone != NULL && current_bone->mParent != NULL && bone_names.has(_ai_string_to_string(current_bone->mParent->mName))) {
		armature_node = current_bone;
		current_bone = scene->mRootNode->FindNode(current_bone->mName)->mParent;
	}
	if (armature_node == NULL) {
		return NULL;
	}

	return Object::cast_to<Spatial>(s->get_owner()->find_node(_ai_string_to_string(armature_node->mName)));
}

Transform EditorSceneImporterAssetImport::_get_armature_xform(const aiScene *scene, const Skeleton *s, const Set<String> bone_names, const Spatial *root, const Spatial *p_mesh_instance) {

	Spatial *armature_node = _find_armature(scene, s, bone_names);

	if (armature_node == NULL) {
		return Transform();
	}
	return _get_global_ai_node_transform(scene, scene->mRootNode->FindNode(_string_to_ai_string(armature_node->get_name())));
}

aiString EditorSceneImporterAssetImport::_string_to_ai_string(String bone_name) {
	//https://stackoverflow.com/a/12903901/381724
	//https://godotengine.org/qa/18552/gdnative-convert-godot-string-to-const-char

	std::wstring ws = bone_name.c_str();
	std::string s = std::string(ws.begin(), ws.end());
	aiString string;
	string.Set(s.c_str());
	return string;
}

void EditorSceneImporterAssetImport::_insert_animation_track(const aiScene *p_scene, int p_bake_fps, Ref<Animation> animation, float ticks_per_second, float length, Skeleton *sk, size_t i, const aiNodeAnim *track, String node_name, NodePath node_path) {
	if (track->mNumRotationKeys || track->mNumPositionKeys || track->mNumScalingKeys) {
		//make transform track
		int track_idx = animation->get_track_count();
		animation->add_track(Animation::TYPE_TRANSFORM);
		animation->track_set_path(track_idx, node_path);
		//first determine animation length

		float increment = 1.0 / float(p_bake_fps);
		float time = 0.0;

		Vector3 base_pos;
		Quat base_rot;
		Vector3 base_scale = Vector3(1, 1, 1);

		if (!track->mNumRotationKeys) {
			aiQuatKey key = track->mRotationKeys[i];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			real_t w = key.mValue.w;
			Quat q(x, y, z, w);
			base_rot = _rot_convert_to_godot(q);
		}

		if (!track->mNumPositionKeys) {
			aiVectorKey key = track->mPositionKeys[i];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_pos = _vec3_convert_to_godot(Vector3(x, y, z));
		}

		if (!track->mNumScalingKeys) {
			aiVectorKey key = track->mScalingKeys[i];
			real_t x = key.mValue.x;
			real_t y = key.mValue.y;
			real_t z = key.mValue.z;
			base_scale = _vec3_convert_to_godot(Vector3(x, y, z));
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
			pos_values.push_back(_vec3_convert_to_godot(Vector3(pos.x, pos.y, pos.z)));
			pos_times.push_back(track->mPositionKeys[p].mTime / ticks_per_second);
		}

		for (size_t r = 0; r < track->mNumRotationKeys; r++) {
			aiQuaternion quat = track->mRotationKeys[r].mValue;
			rot_values.push_back(_rot_convert_to_godot(Quat(quat.x, quat.y, quat.z, quat.w)));
			rot_times.push_back(track->mRotationKeys[r].mTime / ticks_per_second);
		}

		for (size_t sc = 0; sc < track->mNumScalingKeys; sc++) {
			aiVector3D scale = track->mScalingKeys[sc].mValue;
			scale_values.push_back(_vec3_convert_to_godot(Vector3(scale.x, scale.y, scale.z)));
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

			if (sk != NULL && sk->find_bone(node_name) != -1 /*&& is_part_of_split_mesh(p_bone_split_names, node_name) == false*/) {
				Transform xform;
				//xform.basis = Basis(rot);
				//xform.basis.scale(scale);
				xform.basis.set_quat_scale(rot, scale);
				xform.origin = pos;

				int bone = sk->find_bone(node_name);
				xform = sk->get_bone_rest(bone).affine_inverse() * xform;

				rot = xform.basis.get_rotation_quat();
				rot.normalize();
				scale = xform.basis.get_scale();
				pos = xform.origin;
			}
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

bool EditorSceneImporterAssetImport::is_part_of_split_mesh(Set<String> &p_bone_split_names, String node_name) {
	return p_bone_split_names.find(node_name) != NULL;
}

void EditorSceneImporterAssetImport::_import_animation(const aiScene *p_scene, AnimationPlayer *ap, int32_t p_index, int p_bake_fps, Vector<Skeleton *> skeletons) {
	String name = "Animation";
	aiAnimation const *anim = NULL;
	if (p_index != -1) {
		anim = p_scene->mAnimations[p_index];
		if (anim->mName.length > 0) {
			name = _ai_string_to_string(anim->mName);
		}
	}

	Ref<Animation> animation;
	animation.instance();
	float ticks_per_second = 25.0f;
	float length = 0.0f;
	if (p_index != -1) {
		animation->set_name(name);
		ticks_per_second = p_scene->mAnimations[p_index]->mTicksPerSecond != 0 ?
								   p_scene->mAnimations[0]->mTicksPerSecond :
								   25.0f;
		length = anim->mDuration / ticks_per_second;
	}
	if (anim) {
		for (size_t i = 0; i < anim->mNumChannels; i++) {
			if (skeletons.size()) {
				for (size_t si = 0; si < skeletons.size(); si++) {
					const aiNodeAnim *track = anim->mChannels[i];
					String path;
					String node_name = _ai_string_to_string(track->mNodeName);
					Skeleton *sk = skeletons[si];
					//need to find the path
					NodePath node_path = node_name;
					bool is_bone_found = false;
					Vector<String> bone_names;
					for (size_t k = 0; k < sk->get_bone_count(); k++) {
						if (sk->get_bone_name(k).begins_with(node_name)) {
							node_name = sk->get_bone_name(k);
							break;
						}
					}
					if (sk->find_bone(node_name) != -1) {
						is_bone_found = true;
					}
					if (is_bone_found) {
						path = ap->get_owner()->get_path_to(sk);
						ERR_EXPLAIN("Can't find bone to animate");
						ERR_CONTINUE(path == String())
						node_path = path + ":" + node_name;
					} else {
						Node *node = ap->get_owner()->find_node(node_name);
						if (node == NULL) {
							continue;
						}
						path = ap->get_owner()->get_path_to(node);
						if (path == String()) {
							continue;
						}
						node_path = path;
					}

					_insert_animation_track(p_scene, p_bake_fps, animation, ticks_per_second, length, sk, i, track, node_name, node_path);
				}
			} else {
				const aiNodeAnim *track = anim->mChannels[i];
				String path;
				String node_name = _ai_string_to_string(track->mNodeName);
				//need to find the path
				NodePath node_path = node_name;

				Node *node = ap->get_owner()->find_node(node_name);
				if (node == NULL) {
					continue;
				}
				path = ap->get_owner()->get_path_to(node);
				if (path == String()) {
					continue;
				}
				node_path = path;

				_insert_animation_track(p_scene, p_bake_fps, animation, ticks_per_second, length, NULL, i, track, node_name, node_path);
			}
		}
	}
	if (false) {
		for (int i = 0; i < anim->mNumMeshChannels; i++) {
			const aiMeshAnim *anim_mesh = anim->mMeshChannels[i];
			String prop_name = _ai_string_to_string(anim_mesh->mName);
			String prop = "blend_shapes/" + String(prop_name);
			String path = ap->get_owner()->get_path_to(ap->get_owner()->find_node(prop_name));
			ERR_EXPLAIN("Can't find mesh in scene");
			ERR_CONTINUE(path == String())
			String node_path = String(node_path) + ":" + prop;

			int track_idx = animation->get_track_count();
			animation->add_track(Animation::TYPE_VALUE);
			animation->track_set_path(track_idx, node_path);

			//must bake, apologies.
			float increment = 1.0 / float(p_bake_fps);
			float time = 0.0;

			bool last = false;

			Vector<int32_t> values;
			Vector<float> times;
			for (size_t i = 0; i < anim_mesh->mNumKeys; i++) {
				uint32_t value = anim_mesh->mKeys[i].mValue;
				//mAnimMeshes[*value];
				// TODO Add vertex animation shapes
				values.push_back(value);
				times.push_back(anim_mesh->mKeys[i].mTime);
			}

			while (true) {

				_interpolate_track<int32_t>(times, values, time, AssetImportAnimation::INTERP_LINEAR);
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
		for (int i = 0; i < anim->mNumMorphMeshChannels; i++) {
			const aiMeshMorphAnim *morph_mesh = anim->mMorphMeshChannels[i];
			String prop_name = _ai_string_to_string(morph_mesh->mName);
			String prop = String(prop_name);
			String path = ap->get_owner()->get_path_to(ap->get_owner()->find_node(prop_name));
			ERR_EXPLAIN("Can't find mesh in scene");
			ERR_CONTINUE(path == String())
			String node_path = String(node_path) + ":" + prop;

			int track_idx = animation->get_track_count();
			animation->add_track(Animation::TYPE_VALUE);
			animation->track_set_path(track_idx, node_path);

			//must bake, apologies.
			float increment = 1.0 / float(p_bake_fps);
			float time = 0.0;

			bool last = false;

			Vector<int32_t> values;
			Vector<float> times;
			for (size_t i = 0; i < morph_mesh->mNumKeys; i++) {
				uint32_t *value = morph_mesh->mKeys[i].mValues;
				// TODO Add blend shapes
				values.push_back(*value);
				times.push_back(morph_mesh->mKeys[i].mTime);
			}
			while (true) {
				_interpolate_track<int32_t>(times, values, time, AssetImportAnimation::INTERP_LINEAR);
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
		xform.orthonormalize();
	}
	return xform;
}

void EditorSceneImporterAssetImport::_generate_node_bone(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_owner, Vector<Skeleton *> &p_skeletons, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names) {
	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		for (size_t k = 0; k < p_skeletons.size(); k++) {
			Skeleton *s = p_skeletons[k];
			const unsigned int mesh_idx = p_node->mMeshes[i];
			const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];
			for (int j = 0; j < ai_mesh->mNumBones; j++) {
				String bone_name = _ai_string_to_string(ai_mesh->mBones[j]->mName);
				bool has_bone = s->find_bone(bone_name) != -1;
				if (has_bone) {
					continue;
				}
				s->add_bone(bone_name);
				r_bone_name.insert(bone_name);
			}
			for (int l = 0; l < ai_mesh->mNumBones; l++) {
				String bone_name = _ai_string_to_string(ai_mesh->mBones[l]->mName);
				int32_t bone_idx = s->find_bone(bone_name);
				ERR_EXPLAIN("Asset Importer: " + bone_name + " bone not found");
				ERR_CONTINUE(bone_idx == -1);
				Transform bone_offset = _extract_ai_matrix_transform(ai_mesh->mBones[l]->mOffsetMatrix).affine_inverse();
				s->set_bone_rest(bone_idx, bone_offset);
			}
			p_skeletons.write[k] = s;
		}
	}

	bool enable_hack = true;
	if (enable_hack) {
		for (size_t k = 0; k < p_skeletons.size(); k++) {
			Skeleton *s = p_skeletons[k];
			String name = _ai_string_to_string(p_node->mName);
			bool can_create_bone = name != _ai_string_to_string(p_scene->mRootNode->mName) && p_node->mNumChildren > 0 && p_node->mNumMeshes == 0 && p_camera_names.has(name) == false && p_light_names.has(name) == false;
			bool is_armature = p_node->mParent == p_scene->mRootNode;
			if (is_armature) {
				for (int i = 0; i < p_node->mNumChildren; i++) {
					if (s->find_bone(_ai_string_to_string(p_node->mChildren[i]->mName))) {
						is_armature = true;
						break;
					}
					is_armature = false;
				}
			}
			if ((can_create_bone && r_bone_name.find(name) == false) || is_armature) {
				s->add_bone(name);
				r_bone_name.insert(name);
				int32_t idx = s->find_bone(name);
				Transform bone_offset = _get_global_ai_node_transform(p_scene, p_node);
				s->set_bone_rest(idx, bone_offset);
			}
			p_skeletons.write[k] = s;
		}
	}

	for (int i = 0; i < p_node->mNumChildren; i++) {
		_generate_node_bone(p_path, p_scene, p_node->mChildren[i], p_owner, p_skeletons, r_bone_name, p_light_names, p_camera_names);
	}
}

void EditorSceneImporterAssetImport::_add_armature_transform_mi(const String p_path, const aiScene *p_scene, Node *current, Node *p_owner, Vector<Skeleton *> &p_skeletons, Set<String> &r_bone_name) {
	for (size_t k = 0; k < p_skeletons.size(); k++) {
		MeshInstance *mi = Object::cast_to<MeshInstance>(current);
		if (mi != NULL) {
			Skeleton *s = p_skeletons[k];
			String path = String(mi->get_path_to(p_owner)) + "/" + String(p_owner->get_path_to(s));
			mi->set_skeleton_path(path);
			Spatial *armature = _find_armature(p_scene, s, r_bone_name);
			aiNode *other_nodes = p_scene->mRootNode->FindNode(_string_to_ai_string(current->get_name()));

			Quat fbx_quat;
			if (p_path.get_extension() == String("fbx")) {
				fbx_quat.set_euler(Vector3(Math::deg2rad(-90.0f), 0.0f, 0.0f));
			}
			bool is_armature_child = armature->is_a_parent_of(mi) && mi->get_parent() == armature;
			bool is_armature_child_top = mi->get_parent() == armature;
			bool is_ai_root_child_top = current->get_parent()->get_name() == _ai_string_to_string(p_scene->mRootNode->mName);
			Transform armature_xform = _get_armature_xform(p_scene, s, r_bone_name, Object::cast_to<Spatial>(p_owner), mi);
			if (is_armature_child && is_armature_child_top) {
				Transform xform = mi->get_transform();				
				mi->set_transform(armature_xform.affine_inverse() * xform);
			} else if (is_armature_child && is_armature_child_top == false) {
				Transform xform = mi->get_transform(); 
				mi->set_transform(armature_xform.affine_inverse() * xform);
			} else if (is_armature_child == false && is_ai_root_child_top) {
				Transform xform = mi->get_transform();
				xform.basis = Basis();
				mi->set_transform(xform);
			} else if (is_armature_child == false && is_ai_root_child_top == false) {
				Transform xform = mi->get_transform();
				xform.scale(xform.basis.get_scale().inverse());
				mi->set_transform(xform);
			}
		}
	}
	for (int i = 0; i < current->get_child_count(); i++) {
		_add_armature_transform_mi(p_path, p_scene, current->get_child(i), p_owner, p_skeletons, r_bone_name);
	}
}

void EditorSceneImporterAssetImport::_generate_node(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_parent, Node *p_owner, Vector<Skeleton *> &p_skeletons, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names) {
	Spatial *node;
	node = memnew(Spatial);
	Vector<char> raw_name;
	bool has_uvs = false;

	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		for (size_t k = 0; k < p_skeletons.size(); k++) {
			Skeleton *s = p_skeletons[k];
			if (p_skeletons.size() == 1) {
				p_owner->add_child(s);
				s->set_owner(p_owner);
			}
			const unsigned int mesh_idx = p_node->mMeshes[i];
			const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];
			memdelete(node);
			MeshInstance *mi = memnew(MeshInstance);
			node = mi;
			_add_mesh_to_mesh_instance(p_node, p_scene, has_uvs, s, p_path, mi, p_owner, r_bone_name);
			p_skeletons.write[k] = s;
		}
	}

	p_parent->add_child(node);
	node->set_owner(p_owner);
	String node_name = _ai_string_to_string(p_node->mName);
	node->set_name(node_name);
	node->set_transform(_extract_ai_matrix_transform(p_node->mTransformation));

	for (int i = 0; i < p_node->mNumChildren; i++) {
		_generate_node(p_path, p_scene, p_node->mChildren[i], node, p_owner, p_skeletons, r_bone_name, p_light_names, p_camera_names);
	}
}

bool EditorSceneImporterAssetImport::_add_mesh_to_mesh_instance(const aiNode *p_node, const aiScene *p_scene, bool has_uvs, Skeleton *s, const String &p_path, MeshInstance *p_mesh_instance, Node *p_owner, Set<String> &r_bone_name) {
	Ref<ArrayMesh> mesh;

	for (size_t i = 0; i < p_node->mNumMeshes; i++) {
		const unsigned int mesh_idx = p_node->mMeshes[i];
		const aiMesh *ai_mesh = p_scene->mMeshes[mesh_idx];
		p_mesh_instance->set_name(_ai_string_to_string(p_node->mName));
		print_verbose("Open Asset Importer: Creating mesh for: " + p_mesh_instance->get_name());
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
		int32_t surface_flags;
		for (size_t j = 0; j < ai_mesh->mNumFaces; j++) {
			const aiFace face = ai_mesh->mFaces[j];
			Vector<int32_t> tri_order;
			tri_order.resize(3);
			tri_order.write[0] = 0;
			tri_order.write[1] = 2;
			tri_order.write[2] = 1;
			for (size_t k = 0; k < face.mNumIndices; k++) {
				ERR_FAIL_COND_V(k >= tri_order.size(), has_uvs);
				unsigned int index = face.mIndices[tri_order[k]];
				if (ai_mesh->HasTextureCoords(0)) {
					has_uvs = true;
					surface_flags = surface_flags | Mesh::ARRAY_FORMAT_TEX_UV;
					st->add_uv(Vector2(ai_mesh->mTextureCoords[0][index].x, 1.0f - ai_mesh->mTextureCoords[0][index].y));
				}
				if (ai_mesh->HasTextureCoords(1)) {
					has_uvs = true;
					surface_flags = surface_flags | Mesh::ARRAY_FORMAT_TEX_UV2;
					st->add_uv2(Vector2(ai_mesh->mTextureCoords[1][index].x, 1.0f - ai_mesh->mTextureCoords[1][index].y));
				}
				const aiVector3D normals = ai_mesh->mNormals[index];
				const Vector3 godot_normal = _vec3_convert_to_godot(Vector3(normals.x, normals.y, normals.z));
				st->add_normal(godot_normal);
				if (ai_mesh->HasTangentsAndBitangents()) {
					const aiVector3D tangents = ai_mesh->mTangents[index];
					const Vector3 godot_tangent = _vec3_convert_to_godot(Vector3(tangents.x, tangents.y, tangents.z));
					const aiVector3D bitangent = ai_mesh->mBitangents[index];
					const Vector3 godot_bitangent = _vec3_convert_to_godot(Vector3(bitangent.x, bitangent.y, bitangent.z));
					float d = godot_normal.cross(godot_tangent).dot(godot_bitangent) > 0.0f ? 1.0f : -1.0f;
					st->add_tangent(Plane(tangents.x, tangents.y, tangents.z, d));
				}

				Map<uint32_t, Vector<String> >::Element *I = vertex_bone_name.find(index);
				Vector<int32_t> bones;

				if (I != NULL) {
					Vector<String> bone_names;
					bone_names.append_array(I->value());
					for (size_t f = 0; f < bone_names.size(); f++) {
						int32_t bone = s->find_bone(bone_names[f]);
						ERR_EXPLAIN("Asset Importer: Mesh can't find bone " + bone_names[f]);
						ERR_FAIL_COND_V(bone == -1, has_uvs);
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
				} else if (s->get_bone_count() > 0) {
					for (size_t f = 0; f < VS::ARRAY_WEIGHTS_SIZE; f++) {
						String bone_name = s->get_bone_name(s->get_bone_count() - 1);
						int32_t bone = s->find_bone(bone_name);
						ERR_FAIL_COND_V(bone == -1, has_uvs);
						bones.push_back(bone);
					}
					st->add_bones(bones);
					Vector<float> empty_weights;
					for (size_t w = 0; w < VS::ARRAY_WEIGHTS_SIZE; w++) {
						empty_weights.push_back(1.0f / VS::ARRAY_WEIGHTS_SIZE);
					}
					st->add_weights(empty_weights);
				}
				const aiVector3D pos = ai_mesh->mVertices[index];
				Vector3 godot_pos = _vec3_convert_to_godot(Vector3(pos.x, pos.y, pos.z));
				st->add_vertex(godot_pos);
			}
		}

		st->index();
		surface_flags |= Mesh::ARRAY_FORMAT_VERTEX | Mesh::ARRAY_FORMAT_INDEX | Mesh::ARRAY_BONES | Mesh::ARRAY_FORMAT_WEIGHTS;
		if (has_uvs) {
			surface_flags |= Mesh::ARRAY_FORMAT_NORMAL | Mesh::ARRAY_FORMAT_TANGENT;
		}
		if (ai_mesh->HasTangentsAndBitangents() == false) {
			st->generate_tangents();
		}
		aiMaterial *ai_material = p_scene->mMaterials[ai_mesh->mMaterialIndex];
		Ref<SpatialMaterial> mat;
		mat.instance();
		mat->set_name(_ai_string_to_string(ai_material->GetName()));
		Map<String, size_t> properties;
		for (size_t p = 0; p < ai_material->mNumProperties; p++) {
			aiMaterialProperty *property = ai_material->mProperties[p];

			String key = _ai_string_to_string(property->mKey);
			properties.insert(key, p);
		}
		if (properties.has("$clr.transparent")) {
			mat->set_feature(SpatialMaterial::Feature::FEATURE_TRANSPARENT, true);
			mat->set_depth_draw_mode(SpatialMaterial::DepthDrawMode::DEPTH_DRAW_ALPHA_OPAQUE_PREPASS);
		}
		_load_material_type(SpatialMaterial::TEXTURE_ALBEDO, aiTextureType_DIFFUSE, mat, ai_material, p_path);
		_load_material_type(SpatialMaterial::TEXTURE_EMISSION, aiTextureType_EMISSIVE, mat, ai_material, p_path);
		_load_material_type(SpatialMaterial::TEXTURE_NORMAL, aiTextureType_NORMALS, mat, ai_material, p_path);

		mesh = st->commit(mesh);
		mesh->surface_set_material(i, mat);
	}
	p_mesh_instance->set_mesh(mesh);
	//for (int i = 0; i < mesh.blend_weights.size(); i++) {
	//	mi->set("blend_shapes/" + mesh.mesh->get_blend_shape_name(i), mesh.blend_weights[i]);
	//}
	return has_uvs;
}

void EditorSceneImporterAssetImport::_load_material_type(SpatialMaterial::TextureParam p_spatial_material_type, aiTextureType p_texture_type, Ref<SpatialMaterial> p_spatial_material, aiMaterial *p_ai_material, const String &p_path) {
	for (size_t t = 0; t < p_ai_material->GetTextureCount(p_texture_type); t++) {
		if (p_spatial_material_type == SpatialMaterial::TEXTURE_NORMAL) {
			p_spatial_material->set_feature(SpatialMaterial::Feature::FEATURE_NORMAL_MAPPING, true);
		}
		if (p_spatial_material_type == SpatialMaterial::TEXTURE_EMISSION) {
			p_spatial_material->set_feature(SpatialMaterial::Feature::FEATURE_EMISSION, true);
		}
		aiString texture_path;
		p_ai_material->GetTexture(p_texture_type, t, &texture_path);
		String path = p_path.get_base_dir() + "/" + _ai_string_to_string(texture_path).replace("\\", "/");
		_Directory dir;
		bool found = false;
		if (dir.file_exists(p_path.get_base_dir() + path)) {
			found = true;
		}
		if (found == false) {
			found = found || _find_texture_path(p_path, dir, path, found, ".jpg");
		}
		if (found == false) {
			found = found || _find_texture_path(p_path, dir, path, found, ".jpeg");
		}
		if (found == false) {
			found = found || _find_texture_path(p_path, dir, path, found, ".png");
		}
		if (found == false) {
			found = found || _find_texture_path(p_path, dir, path, found, ".exr");
		}
		if (found == false) {
			found = found || _find_texture_path(p_path, dir, path, found, ".tga");
		}
		if (found == false) {
			continue;
		}
		Ref<Texture> texture = ResourceLoader::load(path, "Texture");
		p_spatial_material->set_texture(p_spatial_material_type, texture);
	}
}

bool EditorSceneImporterAssetImport::_find_texture_path(const String &p_path, _Directory &dir, String &path, bool &found, String extension) {
	String name = path.get_basename() + extension;
	if (dir.file_exists(name)) {
		found = true;
		path = name;
		return found;
	}
	String name_ignore_sub_directory = p_path.get_base_dir() + "/" + path.get_file().get_basename() + extension;
	if (dir.file_exists(name_ignore_sub_directory)) {
		found = true;
		path = name_ignore_sub_directory;
		return found;
	}
	return found;
}

String EditorSceneImporterAssetImport::_ai_string_to_string(const aiString p_node) {
	Vector<char> raw_name;
	raw_name.resize(p_node.length);
	memcpy(raw_name.ptrw(), p_node.C_Str(), p_node.length);
	String name;
	name.parse_utf8(raw_name.ptrw(), raw_name.size());
	if (name.find(":") != -1) {
		String replaced_name = name.replace(":", "_");
		print_verbose("Replacing " + name + " containing : with " + replaced_name);
		name = replaced_name;
	}
	return name;
}

Ref<Animation> EditorSceneImporterAssetImport::import_animation(const String &p_path, uint32_t p_flags, int p_bake_fps) {
	return Ref<Animation>();
}

const Transform EditorSceneImporterAssetImport::_extract_ai_matrix_transform(const aiMatrix4x4 p_matrix) {
	aiQuaternion rotation;
	aiVector3t<ai_real> position;
	aiVector3t<ai_real> scaling;
	aiMatrix4x4 matrix = p_matrix;
	matrix.Decompose(scaling, rotation, position);
	Transform xform;
	xform.set_origin(_vec3_convert_to_godot(Vector3(position.x, position.y, position.z)));
	xform.basis.set_quat_scale(_rot_convert_to_godot(Quat(rotation.x, rotation.y, rotation.z, rotation.w)),
			_vec3_convert_to_godot(Vector3(scaling.x, scaling.y, scaling.z)));
	return xform;
}

const Vector3 EditorSceneImporterAssetImport::_vec3_convert_to_godot(Vector3 p_pos) const {
	return p_pos;
}

const Quat EditorSceneImporterAssetImport::_rot_convert_to_godot(Quat p_quat) const {
	return p_quat;
}
