/******************************************************************************
 * Copyright (c) 2018-2026 openblack developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/openblack/openblack
 *
 * openblack is licensed under the GNU General Public License version 3.
 *******************************************************************************/

#pragma once

#include <map>
#include <unordered_map>
#include <vector>

#include <glm/mat4x4.hpp>

#include "3D/AllMeshes.h"
#include "RenderingSystemCommon.h"

#if !defined(LOCATOR_IMPLEMENTATIONS)
#error "Locator interface implementations should only be included in Locator.cpp, use interface instead."
#endif

namespace openblack::ecs::systems
{

class RenderingSystem final: public RenderingSystemCommon
{
public:
	~RenderingSystem();

private:
	void PrepareDrawDescs(bool drawBoundingBox) override;
	void PrepareDrawUploadUniforms(bool drawBoundingBox) override;

	/// Reusable scratch containers — cleared at the start of each PrepareDrawDescs /
	/// PrepareDrawUploadUniforms call to avoid per-frame heap allocations while
	/// still benefiting from previously reserved capacity.
	std::unordered_map<entt::id_type, std::pair<uint32_t, bool>> _scratchMeshIds;
	std::map<entt::id_type, uint32_t> _scratchUniformOffsets;
};
} // namespace openblack::ecs::systems
