/*************************************************************************/
/*  mesh_merge_material_repack.cpp                                       */
/*************************************************************************/
/*                       This file is part of:                           */
/*                           GODOT ENGINE                                */
/*                      https://godotengine.org                          */
/*************************************************************************/
/* Copyright (c) 2007-2019 Juan Linietsky, Ariel Manzur.                 */
/* Copyright (c) 2014-2019 Godot Engine contributors (cf. AUTHORS.md)    */
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

/*
xatlas
https://github.com/jpcy/xatlas
Copyright (c) 2018 Jonathan Young
Permission is hereby granted, free of charge, to any person obtaining a copy of this software and associated documentation files (the "Software"), to deal in the Software without restriction, including without limitation the rights to use, copy, modify, merge, publish, distribute, sublicense, and/or sell copies of the Software, and to permit persons to whom the Software is furnished to do so, subject to the following conditions:
The above copyright notice and this permission notice shall be included in all copies or substantial portions of the Software.
THE SOFTWARE IS PROVIDED "AS IS", WITHOUT WARRANTY OF ANY KIND, EXPRESS OR IMPLIED, INCLUDING BUT NOT LIMITED TO THE WARRANTIES OF MERCHANTABILITY, FITNESS FOR A PARTICULAR PURPOSE AND NONINFRINGEMENT. IN NO EVENT SHALL THE AUTHORS OR COPYRIGHT HOLDERS BE LIABLE FOR ANY CLAIM, DAMAGES OR OTHER LIABILITY, WHETHER IN AN ACTION OF CONTRACT, TORT OR OTHERWISE, ARISING FROM, OUT OF OR IN CONNECTION WITH THE SOFTWARE OR THE USE OR OTHER DEALINGS IN THE SOFTWARE.
*/
/*
thekla_atlas
https://github.com/Thekla/thekla_atlas
MIT License
Copyright (c) 2013 Thekla, Inc
Copyright NVIDIA Corporation 2006 -- Ignacio Castano <icastano@nvidia.com>
*/

#include "mesh_merge_material_pack.h"
#include "core/math/vector2.h"
#include "core/math/vector3.h"
#include "core/os/os.h"
#include "scene/resources/mesh_data_tool.h"
#include "scene/resources/surface_tool.h"
#include "scene_optimize.h"

bool MeshMergeMaterialRepack::setAtlasTexel(void *param, int x, int y, const Vector3 &bar, const Vector3 &, const Vector3 &, float) {
	SetAtlasTexelArgs *args = (SetAtlasTexelArgs *)param;

	if (!args->sourceTexture.is_valid()) {
		args->atlasData->lock();
		args->atlasData->set_pixel(x, y, Color());
		args->atlasData->unlock();

	} else {
		// Interpolate source UVs using barycentrics.
		const Vector2 sourceUv = args->source_uvs[0] * bar.x + args->source_uvs[1] * bar.y + args->source_uvs[2] * bar.z;
		// Keep coordinates in range of texture dimensions.
		int sx = int(sourceUv.x);
		while (sx < 0) {
			sx += args->sourceTexture->get_width();
		}
		if (sx >= args->sourceTexture->get_width()) {
			sx %= args->sourceTexture->get_width();
		}
		int sy = int(sourceUv.y);
		while (sy < 0) {
			sy += args->sourceTexture->get_height();
		}
		if (sy >= args->sourceTexture->get_height()) {
			sy %= args->sourceTexture->get_height();
		}

		args->sourceTexture->lock();
		const Color color = args->sourceTexture->get_pixel(sx, sy);
		args->sourceTexture->unlock();
		args->atlasData->lock();
		args->atlasData->set_pixel(x, y, color);
		args->atlasData->unlock();

		AtlasLookupTexel lookup = args->atlas_lookup[x + y * args->atlas_width];
		lookup.material_index = args->material_index;
		lookup.x = (uint16_t)sx;
		lookup.y = (uint16_t)sy;
		args->atlas_lookup.write[x + y * args->atlas_width] = lookup;
	}
	return true;
}
void MeshMergeMaterialRepack::_find_all_mesh_instances(Vector<MeshInstance *> &r_items, Node *p_current_node, const Node *p_owner) {
	MeshInstance *mi = Object::cast_to<MeshInstance>(p_current_node);

	if (mi) {
		r_items.push_back(mi);
	}
	for (int32_t i = 0; i < p_current_node->get_child_count(); i++) {
		_find_all_mesh_instances(r_items, p_current_node->get_child(i), p_owner);
	}
}
Node *MeshMergeMaterialRepack::pack(Node *p_root) {
	Vector<MeshInstance *> mesh_items;
	_find_all_mesh_instances(mesh_items, p_root, p_root);
	PoolVector<Ref<Material> > vertex_to_material;
	Vector<Ref<Material> > material_cache;
	Ref<Material> empty_material;
	material_cache.push_back(empty_material);
	map_vertex_to_material(mesh_items, vertex_to_material, material_cache);

	PoolVector2Array uvs;
	Vector<ModelVertex> model_vertices;
	scale_uvs_by_texture_dimension(mesh_items, uvs, vertex_to_material, model_vertices);

	xatlas::SetPrint(printf, true);
	xatlas::Atlas *atlas = xatlas::Create();

	int32_t num_surfaces = 0;

	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.empty()) {
				continue;
			}
			num_surfaces++;
		}
	}
	xatlas::PackOptions pack_options;
	generate_atlas(num_surfaces, uvs, atlas, mesh_items, vertex_to_material, material_cache, pack_options);
	Vector<AtlasLookupTexel> atlas_lookup;
	atlas_lookup.resize(atlas->width * atlas->height);
	Node *root = output(p_root, atlas, mesh_items, vertex_to_material, uvs, model_vertices, p_root->get_name(), pack_options, atlas_lookup);

	xatlas::Destroy(atlas);
	return root;
}

void MeshMergeMaterialRepack::generate_atlas(const int32_t p_num_meshes, PoolVector2Array &r_uvs, xatlas::Atlas *atlas, Vector<MeshInstance *> &r_meshes, PoolVector<Ref<Material> > vertex_to_material, const Vector<Ref<Material> > material_cache,
		xatlas::PackOptions &pack_options) {

	int32_t mesh_first_index = 0;
	for (int32_t i = 0; i < r_meshes.size(); i++) {
		for (int32_t j = 0; j < r_meshes[i]->get_mesh()->get_surface_count(); j++) {
			// Handle blend shapes?
			Array mesh = r_meshes[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.empty()) {
				continue;
			}
			Array indices = mesh[ArrayMesh::ARRAY_INDEX];
			if (indices.empty()) {
				continue;
			}
			xatlas::UvMeshDecl meshDecl;
			meshDecl.vertexCount = r_uvs.size();
			meshDecl.vertexUvData = r_uvs.read().ptr();
			meshDecl.vertexStride = sizeof(Vector2);
			PoolIntArray mesh_indices = mesh[Mesh::ARRAY_INDEX];
			Vector<uint32_t> indexes;
			indexes.resize(mesh_indices.size());
			PoolVector<uint32_t> materials;
			materials.resize(mesh_indices.size());
			for (int32_t k = 0; k < mesh_indices.size(); k++) {
				indexes.write[k] = mesh_indices[k];
				Ref<Material> material = vertex_to_material[mesh_indices[k]];
				if (material.is_valid()) {
					if (material_cache.find(material) != -1) {
						materials.write()[k] = material_cache.find(material);
					}
				} else {
					materials.write()[k] = 0;
				}
			}
			meshDecl.indexCount = indexes.size();
			meshDecl.indexData = indexes.ptr();
			meshDecl.indexFormat = xatlas::IndexFormat::UInt32;
			meshDecl.indexOffset = mesh_first_index;
			meshDecl.faceMaterialData = materials.read().ptr();
			meshDecl.rotateCharts = false;
			xatlas::AddMeshError::Enum error = xatlas::AddUvMesh(atlas, meshDecl);
			if (error != xatlas::AddMeshError::Success) {
				OS::get_singleton()->print("Error adding mesh %d: %s\n", i, xatlas::StringForEnum(error));
				ERR_CONTINUE(error != xatlas::AddMeshError::Success);
			}
			mesh_first_index += vertices.size();
		}
	}
	pack_options.padding = 1;
	// TODO(Ernest) Better texel units
	pack_options.texelsPerUnit = 1.0f;
	xatlas::PackCharts(atlas, pack_options);
}

void MeshMergeMaterialRepack::scale_uvs_by_texture_dimension(Vector<MeshInstance *> &mesh_items, PoolVector2Array &uvs, PoolVector<Ref<Material> > &r_vertex_to_material, Vector<ModelVertex> &r_model_vertices) {
	int64_t num_vertices = 0;
	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.size() == 0) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.size() == 0) {
				continue;
			}
			PoolVector3Array arr = mesh[Mesh::ARRAY_VERTEX];
			num_vertices += arr.size();
		}
	}
	uvs.resize(num_vertices);
	r_model_vertices.resize(num_vertices);
	int32_t first_vertex_index = 0;
	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.size() == 0) {
				continue;
			}
			PoolVector3Array vertex_arr = mesh[Mesh::ARRAY_VERTEX];
			PoolVector3Array normal_arr = mesh[Mesh::ARRAY_NORMAL];
			PoolVector2Array uv_arr = mesh[Mesh::ARRAY_TEX_UV];
			PoolIntArray index_arr = mesh[Mesh::ARRAY_INDEX];
			Transform xform = mesh_items[i]->get_global_transform();
			for (int32_t k = 0; k < vertex_arr.size(); k++) {
				ModelVertex vertex;
				vertex.pos = xform.xform(vertex_arr[k]);
				if (normal_arr.size()) {
					vertex.normal = normal_arr[k];
				}
				if (uv_arr.size()) {
					vertex.uv = uv_arr[k];
				}
				r_model_vertices.write[first_vertex_index + k] = vertex;
			}
			first_vertex_index += vertex_arr.size();
		}
	}
	first_vertex_index = 0;
	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.size() == 0) {
				continue;
			}
			PoolVector3Array arr = mesh[Mesh::ARRAY_VERTEX];
			for (uint32_t k = 0; k < arr.size(); k++) {
				Ref<SpatialMaterial> empty_material;
				empty_material.instance();
				int64_t remapped_vertex = first_vertex_index + k;
				const Ref<SpatialMaterial> material = r_vertex_to_material[remapped_vertex];
				if (material.is_null()) {
					break;
				}
				ERR_CONTINUE(material->get_class_name() != empty_material->get_class_name());
				const Ref<Texture> tex = material->get_texture(SpatialMaterial::TextureParam::TEXTURE_ALBEDO);
				uvs.write()[remapped_vertex] = r_model_vertices[remapped_vertex].uv;
				if (tex.is_valid()) {
					uvs.write()[remapped_vertex].x *= (float)tex->get_width();
					uvs.write()[remapped_vertex].y *= (float)tex->get_height();
				}
			}
			first_vertex_index += arr.size();
		}
	}
}

void MeshMergeMaterialRepack::map_vertex_to_material(Vector<MeshInstance *> mesh_items, PoolVector<Ref<Material> > &vertex_to_material, Vector<Ref<Material> > &material_cache) {
	int64_t num_vertices = 0;
	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.empty()) {
				continue;
			}
			PoolVector3Array arr = mesh[Mesh::ARRAY_VERTEX];
			num_vertices += arr.size();
		}
	}
	vertex_to_material.resize(num_vertices);
	int32_t first_index = 0;
	for (int32_t i = 0; i < mesh_items.size(); i++) {
		for (int32_t j = 0; j < mesh_items[i]->get_mesh()->get_surface_count(); j++) {
			Array mesh = mesh_items[i]->get_mesh()->surface_get_arrays(j);
			if (mesh.empty()) {
				continue;
			}
			Array vertices = mesh[ArrayMesh::ARRAY_VERTEX];
			if (vertices.empty()) {
				continue;
			}
			PoolVector3Array arr = mesh[Mesh::ARRAY_VERTEX];
			for (int32_t k = 0; k < arr.size(); k++) {
				Ref<Material> mat = mesh_items[i]->get_mesh()->surface_get_material(j);
				if (material_cache.find(mat) == -1) {
					material_cache.push_back(mat);
				}
				if (mat.is_valid()) {
					vertex_to_material.write()[k + first_index] = mat;
				} else {
					Ref<SpatialMaterial> new_mat;
					new_mat.instance();
					vertex_to_material.write()[k + first_index] = new_mat;
				}
			}
			first_index += arr.size();
		}
	}
}

Node *MeshMergeMaterialRepack::output(Node *p_root, xatlas::Atlas *atlas, Vector<MeshInstance *> &r_mesh_items, const PoolVector<Ref<Material> > vertex_to_material, const PoolVector2Array uvs, const Vector<ModelVertex> model_vertices, String p_name, const xatlas::PackOptions &pack_options, Vector<AtlasLookupTexel> &atlas_lookup) {
	MeshMergeMaterialRepack::TextureData texture_data;
	Ref<Image> atlas_img_albedo;
	atlas_img_albedo.instance();
	atlas_img_albedo->create(atlas->width, atlas->height, true, Image::FORMAT_RGBA8);
	atlas_img_albedo->fill(Color());
	// Rasterize chart triangles.
	for (uint32_t i = 0; i < atlas->meshCount; i++) {
		const xatlas::Mesh &mesh = atlas->meshes[i];
		for (uint32_t j = 0; j < mesh.chartCount; j++) {

			const xatlas::Chart &chart = mesh.chartArray[j];
			Ref<SpatialMaterial> material = vertex_to_material[chart.indexArray[0]];

			Vector<uint8_t> dest_img;
			if (material.is_null()) {
				continue;
			}
			Ref<Texture> tex = material->get_texture(SpatialMaterial::TEXTURE_ALBEDO);
			if (tex.is_null()) {
				continue;
			}
			//TODO (Ernest) Handle case with color but no texture
			Ref<Image> img = tex->get_data();
			ERR_EXPLAIN("Float textures are not supported yet");
			ERR_CONTINUE(Image::get_format_pixel_size(img->get_format()) > 4);
			if (img->is_compressed()) {
				img->decompress();
			}
			img->convert(Image::FORMAT_RGBA8);
			Ref<ImageTexture> image_texture;
			image_texture.instance();
			image_texture->create_from_image(img);
			material->set_texture(SpatialMaterial::TEXTURE_ALBEDO, image_texture);
			SetAtlasTexelArgs args = {
				atlas_img_albedo,
				img,
				atlas_lookup
			};
			for (uint32_t k = 0; k < chart.indexCount / 3; k++) {
				Vector2 v[3];
				for (uint32_t l = 0; l < 3; l++) {
					const uint32_t index = chart.indexArray[k * 3 + l];
					const xatlas::Vertex &vertex = mesh.vertexArray[index];
					v[l] = Vector2(vertex.uv[0], vertex.uv[1]);
					args.source_uvs[l] = uvs[vertex.xref];
				}
				Triangle tri(v[0], v[1], v[2], Vector3(1, 0, 0), Vector3(0, 1, 0), Vector3(0, 0, 1));
				tri.drawAA(setAtlasTexel, &args);
			}
		}
	}
	if (false && pack_options.padding > 0) {
		// Run a dilate filter on the atlas texture to fill in padding around charts so bilinear filtering doesn't sample empty texels.
		// Sample from the source texture(s).
		//Ref<Image> temp_atlas_texture;
		//temp_atlas_texture.instance();
		//temp_atlas_texture->create(atlas->width, atlas->height, true, Image::FORMAT_RGBA8, atlas_img_albedo->get_data());
		//temp_atlas_texture->fill(Color(0.0f, 0.0f, 0.0f, 1.0f));
		Vector<AtlasLookupTexel> temp_atlas_lookup;
		temp_atlas_lookup.resize(atlas_lookup.size());
		const int sampleXOffsets[] = { -1, 0, 1, -1, 1, -1, 0, 1 };
		const int sampleYOffsets[] = { -1, -1, -1, 0, 0, 1, 1, 1 };
		PoolVector<Ref<Image> > vertex_to_img;
		// There has to be a better way.
		vertex_to_img.resize(vertex_to_material.size());
		for (int32_t i = 0; i < vertex_to_material.size(); i++) {
			Ref<SpatialMaterial> source_material = vertex_to_material[i];
			if (source_material.is_null()) {
				Ref<Image> img;
				img.instance();
				vertex_to_img.write()[i] = img;
				continue;
			}
			Ref<Texture> albedo_texture = source_material->get_texture(SpatialMaterial::TEXTURE_ALBEDO);
			if (albedo_texture.is_null()) {
				Ref<Image> img;
				img.instance();
				vertex_to_img.write()[i] = img;
				continue;
			}
			Ref<Image> img = albedo_texture->get_data();
			vertex_to_img.write()[i] = img;
		}

		for (uint32_t i = 0; i < pack_options.padding; i++) {
			memcpy(temp_atlas_lookup.ptrw(), atlas_lookup.ptr(), atlas_lookup.size() * sizeof(AtlasLookupTexel));
			for (uint32_t y = 0; y < atlas->height; y++) {
				for (uint32_t x = 0; x < atlas->width; x++) {
					//if (temp_atlas_texture->get_pixel(0, 0).a != 0.0f) {
					//	continue; // Alpha != 0, already data here.
					//}
					// Try to sample directly from the source texture.
					// Need to find source texel position by checking surrounding texels in the atlas.
					bool foundSample = false;
					for (uint32_t si = 0; si < 8; si++) {
						const int sx = (int)x + sampleXOffsets[si];
						const int sy = (int)y + sampleYOffsets[si];
						if (sx < 0 || sy < 0 || sx >= (int)atlas->width || sy >= (int)atlas->height) {
							continue; // Sample position is outside of atlas texture.
						}
						const AtlasLookupTexel &lookup = temp_atlas_lookup[sx + sy * (int)atlas->width];
						//if (lookup.materialIndex == UINT16_MAX || textures[lookup.materialIndex] == UINT32_MAX)
						//	continue; // No source data here.
						// This atlas texel has a corresponding position for the source texel.
						// Subtract the sample offset to get the source position.
						Ref<Image> img = vertex_to_img[lookup.material_index];
						if (img.is_null()) {
							continue;
						}
						const int ssx = (int)lookup.x - sampleXOffsets[si];
						const int ssy = (int)lookup.y - sampleYOffsets[si] * -1; // need to flip y?

						if (ssx < 0 || ssy < 0 || ssx >= (int)img->get_width() || ssy >= (int)img->get_height()) {
							continue; // Sample position is outside of source texture.
						}
						// Valid sample.
						img->lock();
						atlas_img_albedo->lock();
						atlas_img_albedo->set_pixel(x, y, img->get_pixel(ssx, ssy));
						atlas_img_albedo->unlock();
						img->unlock();
						AtlasLookupTexel temp_lookup = temp_atlas_lookup.get(x + y * (int)atlas->width);
						temp_lookup.x = (uint16_t)ssx;
						temp_lookup.y = (uint16_t)ssy;
						temp_lookup.material_index = lookup.material_index;
						temp_atlas_lookup.write[x + y * (int)atlas->width] = temp_lookup;
						atlas_lookup.write[x + y * (int)atlas->width] = temp_lookup;
						foundSample = true;
						break;
					}
					if (foundSample)
						continue;
					// Sample up to 8 surrounding texels in the source texture, average their color and assign it to this texel.
					float rgb_sum[4] = { 0.0f, 0.0f, 0.0f, 1.0f }, n = 0;
					for (uint32_t si = 0; si < 8; si++) {
						const int sx = (int)x + sampleXOffsets[si];
						const int sy = (int)y + sampleYOffsets[si];
						if (sx < 0 || sy < 0 || sx >= (int)atlas->width || sy >= (int)atlas->height)
							continue; // Sample position is outside of atlas texture.
						const AtlasLookupTexel &lookup = temp_atlas_lookup[sx + sy * (int)atlas->width];
						Ref<Image> img = vertex_to_img[lookup.material_index];
						const int ssx = (int)lookup.x + sampleXOffsets[si];
						const int ssy = (int)lookup.y + sampleYOffsets[si];

						if (ssx < 0 || ssy < 0 || ssx >= (int)img->get_width() || ssy >= (int)img->get_height()) {
							continue; // Sample position is outside of source texture.
						}

						img->lock();
						Color color = img->get_pixel(ssx, ssy);
						img->unlock();
						rgb_sum[0] += color.r;
						rgb_sum[1] += color.g;
						rgb_sum[2] += color.b;
						rgb_sum[3] += color.a;
						n++;
					}
					if (n != 0) {
						const float invn = 1.0f / (float)n;
						Color color;
						color.r = rgb_sum[0] * invn;
						color.g = rgb_sum[1] * invn;
						color.b = rgb_sum[2] * invn;
						color.a = rgb_sum[3] * invn;
						atlas_img_albedo->lock();
						atlas_img_albedo->set_pixel(x, y, color);
						atlas_img_albedo->unlock();
						continue;
					}
					// Sample up to 8 surrounding texels in the atlas texture, average their color and assign it to this texel.
					rgb_sum[0] = rgb_sum[1] = rgb_sum[2] = 0.0f;
					rgb_sum[3] = 0.0f;
					n = 0;
					for (uint32_t si = 0; si < 8; si++) {
						const int sx = (int)x + sampleXOffsets[si];
						const int sy = (int)y + sampleYOffsets[si];
						if (sx < 0 || sy < 0 || sx >= (int)atlas->width || sy >= (int)atlas->height) {
							continue; // Sample position is outside of atlas texture.
						}
						atlas_img_albedo->lock();
						Color color = atlas_img_albedo->get_pixel(sx, sy);
						atlas_img_albedo->unlock();
						if (color.a == 0.0f)
							continue;
						rgb_sum[0] += color.r;
						rgb_sum[1] += color.g;
						rgb_sum[2] += color.b;
						rgb_sum[3] += color.a;
						n++;
					}
					if (n != 0) {
						const float invn = 1.0f / (float)n;
						Color color;
						color.r = rgb_sum[0] * invn;
						color.g = rgb_sum[1] * invn;
						color.b = rgb_sum[2] * invn;
						color.a = rgb_sum[3] * invn;
						atlas_img_albedo->lock();
						atlas_img_albedo->set_pixel(x, y, color);
						atlas_img_albedo->unlock();
					}
				}
			}
		}
	}
	Spatial *root = memnew(Spatial);
	Ref<SurfaceTool> st_all;
	st_all.instance();
	st_all->begin(Mesh::PRIMITIVE_TRIANGLES);
	for (uint32_t i = 0; i < atlas->meshCount; i++) {
		const xatlas::Mesh &mesh = atlas->meshes[i];
		for (uint32_t v = 0; v < mesh.vertexCount; v++) {
			const xatlas::Vertex &vertex = mesh.vertexArray[v];
			const ModelVertex &sourceVertex = model_vertices[vertex.xref];
			// TODO (Ernest) UV2
			st_all->add_uv(Vector2(vertex.uv[0] / atlas->width, vertex.uv[1] / atlas->height));
			st_all->add_normal(Vector3(sourceVertex.normal.x, sourceVertex.normal.y, sourceVertex.normal.z));
			st_all->add_vertex(Vector3(sourceVertex.pos.x, sourceVertex.pos.y, sourceVertex.pos.z));
		}
		break;
	}
	for (uint32_t i = 0; i < atlas->meshCount; i++) {
		const xatlas::Mesh &mesh = atlas->meshes[i];
		for (uint32_t f = 0; f < mesh.indexCount; f++) {
			const uint32_t index = mesh.indexArray[f];
			st_all->add_index(index);
		}
	}
	Ref<SpatialMaterial> mat;
	mat.instance();
	if (atlas->width != 0 || atlas->height != 0) {
		Ref<ImageTexture> texture;
		texture.instance();
		atlas_img_albedo->compress();
		texture->create_from_image(atlas_img_albedo);
		mat->set_texture(SpatialMaterial::TEXTURE_ALBEDO, texture);
	}
	MeshInstance *mi = memnew(MeshInstance);
	Ref<ArrayMesh> array_mesh = st_all->commit();
	mi->set_mesh(array_mesh);
	mi->set_name(p_name + "Merged");
	array_mesh->surface_set_material(0, mat);
	root->add_child(mi);
	mi->set_owner(root);
	return root;
}

MeshMergeMaterialRepack::Triangle::Triangle(const Vector2 &v0, const Vector2 &v1, const Vector2 &v2, const Vector3 &t0, const Vector3 &t1, const Vector3 &t2) {
	// Init vertices.
	this->v1 = v0;
	this->v2 = v2;
	this->v3 = v1;
	// Set barycentric coordinates.
	this->t1 = t0;
	this->t2 = t2;
	this->t3 = t1;
	// make sure every triangle is front facing.
	flipBackface();
	// Compute deltas.
	computeDeltas();
	computeUnitInwardNormals();
}

bool MeshMergeMaterialRepack::Triangle::computeDeltas() {
	Vector2 e0 = v3 - v1;
	Vector2 e1 = v2 - v1;
	Vector3 de0 = t3 - t1;
	Vector3 de1 = t2 - t1;
	float denom = 1.0f / (e0.y * e1.x - e1.y * e0.x);
	if (!std::isfinite(denom)) {
		return false;
	}
	float lambda1 = -e1.y * denom;
	float lambda2 = e0.y * denom;
	float lambda3 = e1.x * denom;
	float lambda4 = -e0.x * denom;
	dx = de0 * lambda1 + de1 * lambda2;
	dy = de0 * lambda3 + de1 * lambda4;
	return true;
}

void MeshMergeMaterialRepack::Triangle::flipBackface() {
	// check if triangle is backfacing, if so, swap two vertices
	if (((v3.x - v1.x) * (v2.y - v1.y) - (v3.y - v1.y) * (v2.x - v1.x)) < 0) {
		Vector2 hv = v1;
		v1 = v2;
		v2 = hv; // swap pos
		Vector3 ht = t1;
		t1 = t2;
		t2 = ht; // swap tex
	}
}

void MeshMergeMaterialRepack::Triangle::computeUnitInwardNormals() {
	n1 = v1 - v2;
	n1 = Vector2(-n1.y, n1.x);
	n1 = n1 * (1.0f / sqrtf(n1.x * n1.x + n1.y * n1.y));
	n2 = v2 - v3;
	n2 = Vector2(-n2.y, n2.x);
	n2 = n2 * (1.0f / sqrtf(n2.x * n2.x + n2.y * n2.y));
	n3 = v3 - v1;
	n3 = Vector2(-n3.y, n3.x);
	n3 = n3 * (1.0f / sqrtf(n3.x * n3.x + n3.y * n3.y));
}

bool MeshMergeMaterialRepack::Triangle::drawAA(SamplingCallback cb, void *param) {
	const float PX_INSIDE = 1.0f / sqrtf(2.0f);
	const float PX_OUTSIDE = -1.0f / sqrtf(2.0f);
	const float BK_SIZE = 8;
	const float BK_INSIDE = sqrtf(BK_SIZE * BK_SIZE / 2.0f);
	const float BK_OUTSIDE = -sqrtf(BK_SIZE * BK_SIZE / 2.0f);
	float minx, miny, maxx, maxy;
	// Bounding rectangle
	minx = floorf(std::max(std::min(v1.x, std::min(v2.x, v3.x)), 0.0f));
	miny = floorf(std::max(std::min(v1.y, std::min(v2.y, v3.y)), 0.0f));
	maxx = ceilf(std::max(v1.x, std::max(v2.x, v3.x)));
	maxy = ceilf(std::max(v1.y, std::max(v2.y, v3.y)));
	// There's no reason to align the blocks to the viewport, instead we align them to the origin of the triangle bounds.
	minx = floorf(minx);
	miny = floorf(miny);
	//minx = (float)(((int)minx) & (~((int)BK_SIZE - 1))); // align to blocksize (we don't need to worry about blocks partially out of viewport)
	//miny = (float)(((int)miny) & (~((int)BK_SIZE - 1)));
	minx += 0.5;
	miny += 0.5; // sampling at texel centers!
	maxx += 0.5;
	maxy += 0.5;
	// Half-edge constants
	float C1 = n1.x * (-v1.x) + n1.y * (-v1.y);
	float C2 = n2.x * (-v2.x) + n2.y * (-v2.y);
	float C3 = n3.x * (-v3.x) + n3.y * (-v3.y);
	// Loop through blocks
	for (float y0 = miny; y0 <= maxy; y0 += BK_SIZE) {
		for (float x0 = minx; x0 <= maxx; x0 += BK_SIZE) {
			// Corners of block
			float xc = (x0 + (BK_SIZE - 1) / 2.0f);
			float yc = (y0 + (BK_SIZE - 1) / 2.0f);
			// Evaluate half-space functions
			float aC = C1 + n1.x * xc + n1.y * yc;
			float bC = C2 + n2.x * xc + n2.y * yc;
			float cC = C3 + n3.x * xc + n3.y * yc;
			// Skip block when outside an edge
			if ((aC <= BK_OUTSIDE) || (bC <= BK_OUTSIDE) || (cC <= BK_OUTSIDE)) continue;
			// Accept whole block when totally covered
			if ((aC >= BK_INSIDE) && (bC >= BK_INSIDE) && (cC >= BK_INSIDE)) {
				Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);
				for (float y = y0; y < y0 + BK_SIZE; y++) {
					Vector3 tex = texRow;
					for (float x = x0; x < x0 + BK_SIZE; x++) {
						if (!cb(param, (int)x, (int)y, tex, dx, dy, 1.0f)) {
							return false;
						}
						tex += dx;
					}
					texRow += dy;
				}
			} else { // Partially covered block
				float CY1 = C1 + n1.x * x0 + n1.y * y0;
				float CY2 = C2 + n2.x * x0 + n2.y * y0;
				float CY3 = C3 + n3.x * x0 + n3.y * y0;
				Vector3 texRow = t1 + dy * (y0 - v1.y) + dx * (x0 - v1.x);
				for (float y = y0; y < y0 + BK_SIZE; y++) { // @@ This is not clipping to scissor rectangle correctly.
					float CX1 = CY1;
					float CX2 = CY2;
					float CX3 = CY3;
					Vector3 tex = texRow;
					for (float x = x0; x < x0 + BK_SIZE; x++) { // @@ This is not clipping to scissor rectangle correctly.
						if (CX1 >= PX_INSIDE && CX2 >= PX_INSIDE && CX3 >= PX_INSIDE) {
							// pixel completely covered
							Vector3 tex2 = t1 + dx * (x - v1.x) + dy * (y - v1.y);
							if (!cb(param, (int)x, (int)y, tex2, dx, dy, 1.0f)) {
								return false;
							}
						} else if ((CX1 >= PX_OUTSIDE) && (CX2 >= PX_OUTSIDE) && (CX3 >= PX_OUTSIDE)) {
							// triangle partially covers pixel. do clipping.
							ClippedTriangle ct(v1 - Vector2(x, y), v2 - Vector2(x, y), v3 - Vector2(x, y));
							ct.clipAABox(-0.5, -0.5, 0.5, 0.5);
							Vector2 centroid = ct.centroid();
							float area = ct.area();
							if (area > 0.0f) {
								Vector3 texCent = tex - dx * centroid.x - dy * centroid.y;
								//XA_ASSERT(texCent.x >= -0.1f && texCent.x <= 1.1f); // @@ Centroid is not very exact...
								//XA_ASSERT(texCent.y >= -0.1f && texCent.y <= 1.1f);
								//XA_ASSERT(texCent.z >= -0.1f && texCent.z <= 1.1f);
								//Vector3 texCent2 = t1 + dx * (x - v1.x) + dy * (y - v1.y);
								if (!cb(param, (int)x, (int)y, texCent, dx, dy, area)) {
									return false;
								}
							}
						}
						CX1 += n1.x;
						CX2 += n2.x;
						CX3 += n3.x;
						tex += dx;
					}
					CY1 += n1.y;
					CY2 += n2.y;
					CY3 += n3.y;
					texRow += dy;
				}
			}
		}
	}
	return true;
}

MeshMergeMaterialRepack::ClippedTriangle::ClippedTriangle(const Vector2 &a, const Vector2 &b, const Vector2 &c) {
	m_numVertices = 3;
	m_activeVertexBuffer = 0;
	m_verticesA[0] = a;
	m_verticesA[1] = b;
	m_verticesA[2] = c;
	m_vertexBuffers[0] = m_verticesA;
	m_vertexBuffers[1] = m_verticesB;
}

void MeshMergeMaterialRepack::ClippedTriangle::clipHorizontalPlane(float offset, float clipdirection) {
	Vector2 *v = m_vertexBuffers[m_activeVertexBuffer];
	m_activeVertexBuffer ^= 1;
	Vector2 *v2 = m_vertexBuffers[m_activeVertexBuffer];
	v[m_numVertices] = v[0];
	float dy2, dy1 = offset - v[0].y;
	int dy2in, dy1in = clipdirection * dy1 >= 0;
	uint32_t p = 0;
	for (uint32_t k = 0; k < m_numVertices; k++) {
		dy2 = offset - v[k + 1].y;
		dy2in = clipdirection * dy2 >= 0;
		if (dy1in) v2[p++] = v[k];
		if (dy1in + dy2in == 1) { // not both in/out
			float dx = v[k + 1].x - v[k].x;
			float dy = v[k + 1].y - v[k].y;
			v2[p++] = Vector2(v[k].x + dy1 * (dx / dy), offset);
		}
		dy1 = dy2;
		dy1in = dy2in;
	}
	m_numVertices = p;
}

void MeshMergeMaterialRepack::ClippedTriangle::clipVerticalPlane(float offset, float clipdirection) {
	Vector2 *v = m_vertexBuffers[m_activeVertexBuffer];
	m_activeVertexBuffer ^= 1;
	Vector2 *v2 = m_vertexBuffers[m_activeVertexBuffer];
	v[m_numVertices] = v[0];
	float dx2, dx1 = offset - v[0].x;
	int dx2in, dx1in = clipdirection * dx1 >= 0;
	uint32_t p = 0;
	for (uint32_t k = 0; k < m_numVertices; k++) {
		dx2 = offset - v[k + 1].x;
		dx2in = clipdirection * dx2 >= 0;
		if (dx1in) v2[p++] = v[k];
		if (dx1in + dx2in == 1) { // not both in/out
			float dx = v[k + 1].x - v[k].x;
			float dy = v[k + 1].y - v[k].y;
			v2[p++] = Vector2(offset, v[k].y + dx1 * (dy / dx));
		}
		dx1 = dx2;
		dx1in = dx2in;
	}
	m_numVertices = p;
}

void MeshMergeMaterialRepack::ClippedTriangle::computeAreaCentroid() {
	Vector2 *v = m_vertexBuffers[m_activeVertexBuffer];
	v[m_numVertices] = v[0];
	m_area = 0;
	float centroidx = 0, centroidy = 0;
	for (uint32_t k = 0; k < m_numVertices; k++) {
		// http://local.wasp.uwa.edu.au/~pbourke/geometry/polyarea/
		float f = v[k].x * v[k + 1].y - v[k + 1].x * v[k].y;
		m_area += f;
		centroidx += f * (v[k].x + v[k + 1].x);
		centroidy += f * (v[k].y + v[k + 1].y);
	}
	m_area = 0.5f * fabsf(m_area);
	if (m_area == 0) {
		m_centroid = Vector2(0.0f, 0.0f);
	} else {
		m_centroid = Vector2(centroidx / (6 * m_area), centroidy / (6 * m_area));
	}
}

void MeshMergeMaterialRepack::ClippedTriangle::clipAABox(float x0, float y0, float x1, float y1) {
	clipVerticalPlane(x0, -1);
	clipHorizontalPlane(y0, -1);
	clipVerticalPlane(x1, 1);
	clipHorizontalPlane(y1, 1);
	computeAreaCentroid();
}

Vector2 MeshMergeMaterialRepack::ClippedTriangle::centroid() {
	return m_centroid;
}

float MeshMergeMaterialRepack::ClippedTriangle::area() {
	return m_area;
}
