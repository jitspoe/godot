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
#include "core/bind/core_bind.h"

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

	Transform _get_armature_xform(const aiScene *scene, const Skeleton *s, const Set<String> bone_names, const Spatial *root, const Spatial * p_mesh_instance);

	Transform _get_global_ai_node_transform(const aiScene *p_scene, const aiNode *p_current_node);
	void _generate_node_bone(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_owner, Vector<Skeleton *> &p_skeletons, Set<String> &r_bone_name, Set<String> p_light_names, Set<String> p_camera_names);
	void _generate_node(const String &p_path, const aiScene *p_scene, const aiNode *p_node, Node *p_parent, Node *p_owner, Vector<Skeleton *> &p_skeletons, Set<String> & r_bone_name, Set<String> p_light_names, Set<String> p_camera_names);
	aiString _string_to_ai_string(String bone_name);
	void _insert_animation_track(const aiScene *p_scene, int p_bake_fps, Ref<Animation> animation, float ticks_per_second, float length, Skeleton *sk, size_t i, const aiNodeAnim *track, String node_name, NodePath node_path);
	bool is_part_of_split_mesh(Set<String> &p_bone_split_names, String node_name);
	bool _add_mesh_to_mesh_instance(const aiNode *p_node, const aiScene *p_scene, bool has_uvs, Skeleton *s, const String &p_path, MeshInstance *p_mesh_instance, Node *p_owner, Set<String>& r_bone_name);
	void _load_material_type(SpatialMaterial::TextureParam p_spatial_material_type, aiTextureType p_texture_type, Ref<SpatialMaterial> p_spatial_material, aiMaterial *p_ai_material, const String &p_path);
	bool _find_texture_path(const String &p_path, _Directory &dir, String &path, bool &found, String extension);
	String _ai_string_to_string(const aiString p_node);
	void _import_animation(const aiScene *p_scene, AnimationPlayer *ap, int32_t p_index, int p_bake_fps, Vector<Skeleton *> skeletons);
	template <class T>
	T _interpolate_track(const Vector<float> &p_times, const Vector<T> &p_values, float p_time, AssetImportAnimation::Interpolation p_interp);
	const Vector3 _vec3_convert_to_godot(Vector3 p_pos) const;
	const Quat _rot_convert_to_godot(Quat p_quat) const;
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
