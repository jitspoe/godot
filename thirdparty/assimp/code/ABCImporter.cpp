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

#ifndef ASSIMP_BUILD_NO_ABC_IMPORTER

#include "ABCImporter.h"

#include <assimp/MemoryIOWrapper.h>
#include <assimp/StreamReader.h>
#include <assimp/importerdesc.h>
#include <assimp/matrix4x4.h>
#include <assimp/scene.h>
#include <assimp/Importer.hpp>

#include <Alembic/Abc/ErrorHandler.h>
#include <Alembic/AbcCoreAbstract/All.h>
#include <Alembic/AbcCoreOgawa/All.h>
#include <Alembic/AbcGeom/All.h>
#include <Alembic/AbcGeom/IPolyMesh.h>

using namespace Alembic::AbcGeom;

#include <set>

using namespace Assimp;
//using namespace Assimp::Formatter;
//using namespace Assimp::ABC;

#define RESETCOLOR "\033[0m"
#define GREENCOLOR "\033[1;32m"
#define BLUECOLOR "\033[1;34m"
#define BOLD "\033[1m"

namespace {
static const aiImporterDesc desc = {
	"ABC Importer",
	"",
	"",
	"",
	aiImporterFlags_SupportTextFlavour,
	0,
	0,
	0,
	0,
	"abc"
};
}

bool ABCImporter::CanRead(const std::string &pFile, IOSystem *pIOHandler,
		bool checkSig) const {
	const std::string extension = GetExtension(pFile);
	if (extension == "abc") {
		return true;
	}
	if (!extension.length() || checkSig) {
		// no extension given, or we're called a second time because no
		// suitable loader was found yet. This means, we're trying to open
		// the file and look for and hints to identify the file format.
		// #Assimp::BaseImporter provides some utilities:
		//
		// #Assimp::BaseImporter::SearchFileHeaderForToken - for text files.
		// It reads the first lines of the file and does a substring check
		// against a given list of 'magic' strings.
		//
		// #Assimp::BaseImporter::CheckMagicToken - for binary files. It goes
		// to a particular offset in the file and and compares the next words
		// against a given list of 'magic' tokens.
		// These checks MUST be done (even if !checkSig) if the file extension
		// is not exclusive to your format. For example, .xml is very common
		// and (co)used by many formats.
	}
	return false;
}

// -------------------------------------------------------------------------------
// Get list of file extensions handled by this loader
void ABCImporter::GetExtensionList(std::set<std::string> &extensions) {
	extensions.insert("abc");
}

// -------------------------------------------------------------------------------
void ABCImporter::InternReadFile(const std::string &pFile,
		aiScene *pScene, IOSystem *pIOHandler) {
	std::unique_ptr<IOStream> file(pIOHandler->Open(pFile, "rb"));
	// Referenced http://jonmacey.blogspot.com/2011/12/getting-started-with-alembic.html
	IArchive archive(Alembic::AbcCoreOgawa::ReadArchive(), pFile);

	// Check whether we can read from the file
	if (file.get() == NULL) {
		throw DeadlyImportError("Failed to open abc file " + pFile + ".");
	}
	// Based on AbcTree/AbcTree.cpp

	std::vector<std::string> seglist;
	bool opt_all = false;

	// walk object hierarchy and find valid objects
	AbcG::IObject test = archive.getTop();
	AbcG::IObject iObj = test;
	while (test.valid() && seglist.size() > 0) {
		test = test.getChild(seglist.front());
		if (test.valid()) {
			iObj = test;
			seglist.erase(seglist.begin());
		}
	}

	// walk property hierarchy for most recent valid object
	Abc::ICompoundProperty props = iObj.getProperties();
	const Abc::PropertyHeader *header;
	bool found = false;
	for (std::size_t i = 0; i < seglist.size(); ++i) {
		header = props.getPropertyHeader(seglist[i]);
		if (header && header->isCompound()) {
			Abc::ICompoundProperty ptest(props, header->getName());
			if (ptest.valid()) {
				props = ptest;
				found = true;
			}
		} else if (header && header->isSimple()) {
			found = true;
		} else {
			throw DeadlyImportError("Invalid object or property " + seglist[i]);
		}
	}

	// walk the archive tree
	if (found) {
		if (header->isCompound()) {
			tree(props, pScene);
		} else {
			tree(Abc::IScalarProperty(props, header->getName()), pScene);
		}
	} else {
		pScene->mRootNode = new aiNode;
		tree(iObj, pScene, pScene->mRootNode, opt_all);
	}
	TransferDataToScene(pScene);
}

const aiImporterDesc *Assimp::ABCImporter::GetInfo() const {
	return &desc;
}

bool Assimp::ABCImporter::is_leaf(AbcG::IObject iObj) {
	if (!iObj.getParent().valid()) {
		return true;
	}

	Abc::IObject parent = iObj.getParent();
	int numChildren = parent.getNumChildren();

	Abc::IObject test = parent.getChild(numChildren - 1);
	if (test.valid() && test.getName() != iObj.getName()) {
		return false;
	}
	return true;
}

bool Assimp::ABCImporter::is_leaf(Abc::ICompoundProperty iProp, Abc::PropertyHeader iHeader) {
	if (!iProp.valid()) {
		return true;
	}

	int last = iProp.getNumProperties() - 1;
	Abc::PropertyHeader header = iProp.getPropertyHeader(last);
	if (header.getName() == iHeader.getName())
		return true;

	return false;
}

int Assimp::ABCImporter::index(Abc::ICompoundProperty iProp, Abc::PropertyHeader iHeader) {
	for (size_t i = 0; i < iProp.getNumProperties(); i++) {
		Abc::PropertyHeader header = iProp.getPropertyHeader(i);
		if (header.getName() == iHeader.getName()) {
			return i;
		}
	}
	return -1;
}

void Assimp::ABCImporter::tree(Abc::IScalarProperty iProp, aiScene *pScene, std::string prefix) {
	if (iProp.getObject().getFullName() != "/") {
		prefix = prefix + "   ";
	}
	if (is_leaf(iProp.getParent(), iProp.getHeader()) &&
			(iProp.getObject().getNumChildren() == 0 ||
					iProp.getParent().getName() != "")) {
		std::cout << prefix << " `--";
	} else {
		std::cout << prefix << " :--";
		prefix = prefix + " :";
	}

	std::cout << iProp.getName() << "\r" << std::endl;
}

void Assimp::ABCImporter::ConvertMeshSingleMaterial(IPolyMesh polymesh, std::string faceSetName, aiNode *current) {
	aiMesh *const out_mesh = SetupEmptyMesh(polymesh, faceSetName, current);
}

void Assimp::ABCImporter::TransferDataToScene(aiScene *pScene) {
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

aiMesh *Assimp::ABCImporter::SetupEmptyMesh(AbcG::IPolyMesh &polymesh, const std::string &facesetName, aiNode *node) {
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

void Assimp::ABCImporter::tree(Abc::IArrayProperty iProp, aiScene *pScene, std::string prefix) {
	if (iProp.getObject().getFullName() != "/") {
		prefix = prefix + "   ";
	}
	if (is_leaf(iProp.getParent(), iProp.getHeader()) &&
			(iProp.getObject().getNumChildren() == 0 ||
					iProp.getParent().getName() != "")) {
		std::cout << prefix << " `--";
	} else {
		std::cout << prefix << " :--";
		prefix = prefix + " :";
	}

	std::cout << iProp.getName() << "\r" << std::endl;
}

void Assimp::ABCImporter::tree(Abc::ICompoundProperty iProp, aiScene *pScene, std::string prefix) {
	if (iProp.getObject().getFullName() != "/") {
		prefix = prefix + "   ";
	}
	if (is_leaf(iProp.getParent(), iProp.getHeader()) &&
			iProp.getObject().getNumChildren() == 0) {
		std::cout << prefix << " `--";
		prefix = prefix + " ";
	} else {
		if (is_leaf(iProp.getParent(), iProp.getHeader())) {
			std::cout << prefix << " | `--";
			prefix = prefix + " |";
		} else if (iProp.getObject().getNumChildren() == 0) {
			std::cout << prefix << " :--";
			prefix = prefix + " :";
		} else if (is_leaf(iProp, iProp.getHeader())) {
			std::cout << prefix << " | `--";
			prefix = prefix + " |";
		} else {
			std::cout << prefix << " | :--";
			prefix = prefix + " | :";
		}
	}

	std::cout << iProp.getName() << "\r" << std::endl;

	for (size_t i = 0; i < iProp.getNumProperties(); i++) {
		Abc::PropertyHeader header = iProp.getPropertyHeader(i);
		if (header.isScalar()) {
			tree(Abc::IScalarProperty(iProp, header.getName()), pScene, prefix);
		} else if (header.isArray()) {
			tree(Abc::IArrayProperty(iProp, header.getName()), pScene, prefix);
		} else {
			tree(Abc::ICompoundProperty(iProp, header.getName()), pScene, prefix);
		}
	}
}

void Assimp::ABCImporter::tree(AbcG::IObject iObj, aiScene *pScene, aiNode *current, bool showProps, std::string prefix) {
	std::string path = iObj.getFullName();

	if (path == "/") {
		prefix = "";
	} else {
		if (iObj.getParent().getFullName() != "/") {
			prefix = prefix + "   ";
		}
		if (is_leaf(iObj)) {
			std::cout << prefix << " `--";
			prefix = prefix + " ";
		} else {
			std::cout << prefix << " |--";
			prefix = prefix + " |";
		}
	};

	if (showProps) {
		std::cout << GREENCOLOR;
	}

	std::cout << iObj.getName();
	current->mName = iObj.getName();

	if (IXform::matches(iObj.getHeader())) {
		IXform xform(iObj.getParent(), iObj.getHeader().getName());
		IXformSchema &xs = xform.getSchema();
		XformSample xformSample;
		xs.get(xformSample);
		if (xs.getNumOps() > 0) {
			TimeSamplingPtr ts = xs.getTimeSampling();
			size_t numSamples = xs.getNumSamples();
			Abc::M44d abc_mat = xformSample.getMatrix();
			//TODO(Ernest) Fix transform later
			aiMatrix4x4t<ai_real> ai_mat(
					(ai_real)abc_mat[0][0],
					(ai_real)abc_mat[0][1],
					(ai_real)abc_mat[0][2],
					(ai_real)abc_mat[0][3],
					(ai_real)abc_mat[1][0],
					(ai_real)abc_mat[1][1],
					(ai_real)abc_mat[1][2],
					(ai_real)abc_mat[1][3],
					(ai_real)abc_mat[2][0],
					(ai_real)abc_mat[2][1],
					(ai_real)abc_mat[2][2],
					(ai_real)abc_mat[2][3],
					(ai_real)abc_mat[3][0],
					(ai_real)abc_mat[3][1],
					(ai_real)abc_mat[3][2],
					(ai_real)abc_mat[3][3]);
			ai_mat.Transpose();
			current->mTransformation = ai_mat;
		}
	} else if (IPolyMesh::matches(iObj.getHeader())) {
		IPolyMesh polymesh(iObj.getParent(), iObj.getHeader().getName());
		std::string faceSetName;
		//ConvertMeshSingleMaterial(polymesh, faceSetName, current);
	}

	if (iObj.getNumChildren()) {
		const unsigned int numChildren = iObj.getNumChildren();
		aiNode **nodes = new aiNode *[numChildren]();
		for (unsigned int i = 0; i < numChildren; i++) {
			nodes[i] = new aiNode;
		}
		current->mChildren = nodes;
		current->mNumChildren = static_cast<unsigned int>(numChildren);
	}

	if (showProps) {
		std::cout << RESETCOLOR;
	}
	std::cout << "\r" << std::endl;

	// property tree
	if (showProps) {
		Abc::ICompoundProperty props = iObj.getProperties();
		for (size_t i = 0; i < props.getNumProperties(); i++) {
			Abc::PropertyHeader header = props.getPropertyHeader(i);
			if (header.isScalar()) {
				tree(Abc::IScalarProperty(props, header.getName()), pScene, prefix);
			} else if (header.isArray()) {
				tree(Abc::IArrayProperty(props, header.getName()), pScene, prefix);
			} else {
				tree(Abc::ICompoundProperty(props, header.getName()), pScene, prefix);
			}
		}
	}

	// object tree
	for (size_t i = 0; i < iObj.getNumChildren(); i++) {
		tree(AbcG::IObject(iObj, iObj.getChildHeader(i).getName()), pScene, current->mChildren[i], showProps, prefix);
	};
}

#endif
