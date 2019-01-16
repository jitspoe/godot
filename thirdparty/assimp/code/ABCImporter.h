/*
Open Asset Import Library (assimp)
----------------------------------------------------------------------

Copyright (c) 2006-2018, assimp team


All rights reserved.

Redistribution and use of this software in source and binary forms,
with or without modification, are permitted provided that the
following conditions are met:

* Redistributions of source code must retain the above
  copyright notice, this list of conditions and the
  following disclaimer.
r
* Redistributions in binary form must reproduce the above
  copyright notice, this list of conditions and the
  following disclaimer in the documentation and/or other
  materials provided with the distribution.

* Neither the name of the assimp team, nor the names of its
  contributors may be used to endorse or promote products
  derived from this software without specific prior
  written permission of the assimp team.

THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
"AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
(INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.

----------------------------------------------------------------------
*/

//-*****************************************************************************
//
// Copyright (c) 2013,
//  Sony Pictures Imageworks, Inc. and
//  Industrial Light & Magic, a division of Lucasfilm Entertainment Company Ltd.
//
// All rights reserved.
//
// Redistribution and use in source and binary forms, with or without
// modification, are permitted provided that the following conditions are
// met:
// *       Redistributions of source code must retain the above copyright
// notice, this list of conditions and the following disclaimer.
// *       Redistributions in binary form must reproduce the above
// copyright notice, this list of conditions and the following disclaimer
// in the documentation and/or other materials provided with the
// distribution.
// *       Neither the name of Sony Pictures Imageworks, nor
// Industrial Light & Magic nor the names of their contributors may be used
// to endorse or promote products derived from this software without specific
// prior written permission.
//
// THIS SOFTWARE IS PROVIDED BY THE COPYRIGHT HOLDERS AND CONTRIBUTORS
// "AS IS" AND ANY EXPRESS OR IMPLIED WARRANTIES, INCLUDING, BUT NOT
// LIMITED TO, THE IMPLIED WARRANTIES OF MERCHANTABILITY AND FITNESS FOR
// A PARTICULAR PURPOSE ARE DISCLAIMED. IN NO EVENT SHALL THE COPYRIGHT
// OWNER OR CONTRIBUTORS BE LIABLE FOR ANY DIRECT, INDIRECT, INCIDENTAL,
// SPECIAL, EXEMPLARY, OR CONSEQUENTIAL DAMAGES (INCLUDING, BUT NOT
// LIMITED TO, PROCUREMENT OF SUBSTITUTE GOODS OR SERVICES; LOSS OF USE,
// DATA, OR PROFITS; OR BUSINESS INTERRUPTION) HOWEVER CAUSED AND ON ANY
// THEORY OF LIABILITY, WHETHER IN CONTRACT, STRICT LIABILITY, OR TORT
// (INCLUDING NEGLIGENCE OR OTHERWISE) ARISING IN ANY WAY OUT OF THE USE
// OF THIS SOFTWARE, EVEN IF ADVISED OF THE POSSIBILITY OF SUCH DAMAGE.
//
//-*****************************************************************************

#ifndef INCLUDED_AI_ABC_IMPORTER_H
#define INCLUDED_AI_ABC_IMPORTER_H

#include <assimp/BaseImporter.h>
#include <assimp/ai_assert.h>
#include <assimp/scene.h>

#include <Alembic/AbcCollection/All.h>
#include <Alembic/AbcCoreFactory/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcGeom/IPolyMesh.h>
#include <Alembic/AbcMaterial/All.h>

#include <set>
#include <string>

namespace Assimp {

namespace Abc = ::Alembic::Abc;
namespace AbcA = ::Alembic::AbcCoreAbstract;
namespace AbcF = ::Alembic::AbcCoreFactory;
namespace AbcG = ::Alembic::AbcGeom;

class ABCImporter : public BaseImporter {
public:
	bool CanRead(const std::string &pFile, IOSystem *pIOHandler,
			bool checkSig) const;

protected:
	const aiImporterDesc *GetInfo() const;
	void GetExtensionList(std::set<std::string> &extensions);
	void InternReadFile(const std::string &pFile,
			aiScene *pScene, IOSystem *pIOHandler);
	bool is_leaf(AbcG::IObject iObj);
	bool is_leaf(Abc::ICompoundProperty iProp, Abc::PropertyHeader iHeader);
	int index(Abc::ICompoundProperty iProp, Abc::PropertyHeader iHeader);
	void tree(Abc::IScalarProperty iProp, aiScene *pScene, std::string prefix = "");
	void tree(Abc::IArrayProperty iProp, aiScene *pScene, std::string prefix = "");
	void tree(Abc::ICompoundProperty iProp, aiScene *pScene, std::string prefix = "");
	void tree(AbcG::IObject iObj, aiScene *pScene, aiNode *current, bool showProps = false, std::string prefix = "");

private:
	void TransferDataToScene(aiScene *pScene) {
		ai_assert(!pScene->mMeshes);
		ai_assert(!pScene->mNumMeshes);

		// note: the trailing () ensures initialization with NULL - not
		// many C++ users seem to know this, so pointing it out to avoid
		// confusion why this code works.

		if (meshes.size()) {
			pScene->mMeshes = new aiMesh *[meshes.size()]();
			pScene->mNumMeshes = static_cast<unsigned int>(meshes.size());

			std::swap_ranges(meshes.begin(), meshes.end(), pScene->mMeshes);
		}

		if (materials.size()) {
			pScene->mMaterials = new aiMaterial *[materials.size()]();
			pScene->mNumMaterials = static_cast<unsigned int>(materials.size());

			std::swap_ranges(materials.begin(), materials.end(), pScene->mMaterials);
		}

		if (animations.size()) {
			pScene->mAnimations = new aiAnimation *[animations.size()]();
			pScene->mNumAnimations = static_cast<unsigned int>(animations.size());

			std::swap_ranges(animations.begin(), animations.end(), pScene->mAnimations);
		}

		if (lights.size()) {
			pScene->mLights = new aiLight *[lights.size()]();
			pScene->mNumLights = static_cast<unsigned int>(lights.size());

			std::swap_ranges(lights.begin(), lights.end(), pScene->mLights);
		}

		if (cameras.size()) {
			pScene->mCameras = new aiCamera *[cameras.size()]();
			pScene->mNumCameras = static_cast<unsigned int>(cameras.size());

			std::swap_ranges(cameras.begin(), cameras.end(), pScene->mCameras);
		}

		if (textures.size()) {
			pScene->mTextures = new aiTexture *[textures.size()]();
			pScene->mNumTextures = static_cast<unsigned int>(textures.size());

			std::swap_ranges(textures.begin(), textures.end(), pScene->mTextures);
		}
	}
	aiMesh *SetupEmptyMesh(AbcG::IPolyMesh &polymesh, const std::string &facesetName, aiNode *node) {
		// https://github.com/alembic/alembic/blob/8bf5f3494ead3ccb8820fd1768d3f8c764e43fb9/arnold/Procedural/WriteGeo.cpp#L515
		// FBXConverter.cpp

		aiMesh *const out_mesh = new aiMesh();
		meshes.push_back(out_mesh);
		// set name
		std::string name = facesetName;

		if (name.length()) {
			out_mesh->mName.Set(name);
		} else {
			out_mesh->mName = node->mName;
		}
		return out_mesh;
	}

	std::vector<aiMesh *> meshes;
	std::vector<aiMaterial *> materials;
	std::vector<aiAnimation *> animations;
	std::vector<aiLight *> lights;
	std::vector<aiCamera *> cameras;
	std::vector<aiTexture *> textures;
};
} // namespace Assimp
#endif
