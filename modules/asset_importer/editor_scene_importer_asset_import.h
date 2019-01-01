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

#ifndef EDITOR_SCENE_IMPORTER_ASSET_IMPORT_H
#define EDITOR_SCENE_IMPORTER_ASSET_IMPORT_H

#include "core/bind/core_bind.h"
#include "core/vector.h"
#include "editor/import/resource_importer_scene.h"
#include "scene/3d/mesh_instance.h"
#include "scene/3d/skeleton.h"
#include "scene/3d/spatial.h"
#include "scene/animation/animation_player.h"
#include "scene/resources/animation.h"
#include "scene/resources/surface_tool.h"
#include "thirdparty/assimp/include/assimp/matrix4x4.h"
#include "thirdparty/assimp/include/assimp/scene.h"
#include "thirdparty/assimp/include/assimp/types.h"
#include <thirdparty/assimp/include/assimp/DefaultLogger.hpp>
#include <thirdparty/assimp/include/assimp/LogStream.hpp>
#include <thirdparty/assimp/include/assimp/Logger.hpp>

class AssimpStream : public Assimp::LogStream {
public:
	// Constructor
	AssimpStream();

	// Destructor
	~AssimpStream();
	// Write something using your own functionality
	void write(const char *message);
};

class aiScene;
class aiNode;
class EditorSceneImporterAssetImport : public EditorSceneImporter {
private:
	GDCLASS(EditorSceneImporterAssetImport, EditorSceneImporter);

	struct AssetImportAnimation {
		enum Interpolation {
			INTERP_LINEAR
		};
	};

	Spatial *_generate_scene(const String &p_path, const aiScene *scene, const uint32_t p_flags, int p_bake_fps);

	Node * _find_skeleton_root(Map<Skeleton *, MeshInstance *> &skeletons, Map<MeshInstance *, String> &meshes, Spatial *root);

	void _set_bone_parent(Skeleton *s, const aiScene *scene);
	Transform _get_global_ai_node_transform(const aiScene *p_scene, const aiNode *p_current_node);
	void _generate_node_bone(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton);
	void _generate_node_bone_parents(const aiScene *p_scene, const aiNode *p_node, Map<String, bool> &p_mesh_bones, Skeleton *p_skeleton, const MeshInstance *p_mi);
	void _fill_skeleton(const aiScene *p_scene, const aiNode *p_node, Spatial *p_current, Node *p_owner, Skeleton *p_skeleton, const Map<String, bool> p_mesh_bones, const Map<String, Transform> &p_bone_rests, Set<String> p_tracks, Node* p_armature);
	void _generate_mesh(const String &p_path, const aiScene *p_scene, Node *p_parent, Node *p_owner, Set<String> &r_bone_name, Map<Skeleton *, MeshInstance *> &r_skeletons, const Map<String, Transform> &p_bone_rests, Map<MeshInstance *, String> &r_mesh_instances, Set<String> p_tracks);
	void _add_armature_transform_mi(const String p_path, const aiScene *p_scene, Node *current, Node *p_owner, Skeleton *p_skeleton, const Spatial *p_armature);
	void _generate_node(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_parent, Node *p_owner, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names, Map<Skeleton *, MeshInstance *> &r_skeletons, const Map<String, Transform> &p_bone_rests, Map<MeshInstance*, String> &r_mesh_instances, Set<String> p_tracks);
	String _gen_unique_name(String node_name, Node *p_owner);
	void _move_mesh(const aiScene *p_scene, Node *p_current, Node *p_owner, Map<MeshInstance*, String> &p_mesh_instances, Map<Skeleton *, MeshInstance *> p_skeletons);
	void _get_track_set(const aiScene *p_scene, Set<String> &tracks);
	aiString _string_to_ai_string(String bone_name);
	void _insert_animation_track(const aiScene *p_scene, int p_bake_fps, Ref<Animation> animation, float ticks_per_second, float length, const Skeleton *sk, size_t i, const aiNodeAnim *track, String node_name, NodePath node_path);
	void _add_mesh_to_mesh_instance(const aiNode *p_node, const aiScene *p_scene, Skeleton *s, const String &p_path, MeshInstance *p_mesh_instance, Node *p_owner, Set<String> &r_bone_name);
	Vector3 _get_scale(const aiScene *p_scene);
	void _find_texture_path(const String &p_path, String &path, bool &r_found);
	bool _find_texture_path(const String &p_path, _Directory &dir, String &path, bool &found, String extension);
	String _ai_string_to_string(const aiString p_node);
	void _import_animation(const aiScene *p_scene, AnimationPlayer *ap, int32_t p_index, int p_bake_fps, Map<Skeleton *, MeshInstance *> p_skeletons, Node *armature);
	template <class T>
	T _interpolate_track(const Vector<float> &p_times, const Vector<T> &p_values, float p_time, AssetImportAnimation::Interpolation p_interp);
	const Transform _extract_ai_matrix_transform(const aiMatrix4x4 p_matrix);

public:
	EditorSceneImporterAssetImport() {
		Assimp::DefaultLogger::create("", Assimp::Logger::VERBOSE);
		unsigned int severity = Assimp::Logger::Info | Assimp::Logger::Err | Assimp::Logger::Warn;
		Assimp::DefaultLogger::get()->attachStream(new AssimpStream(), severity);
	}
	~EditorSceneImporterAssetImport() {
		Assimp::DefaultLogger::kill();
	}
	virtual void get_extensions(List<String> *r_extensions) const;
	virtual uint32_t get_import_flags() const;
	virtual Node *import_scene(const String &p_path, uint32_t p_flags, int p_bake_fps, List<String> *r_missing_deps, Error *r_err = NULL);
	virtual Ref<Animation> import_animation(const String &p_path, uint32_t p_flags, int p_bake_fps);
};
#endif
