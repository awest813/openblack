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
#include <set>
#include <unordered_map>

#include "3D/AllMeshes.h"
#include "ECS/Components/Temple.h"
#include "RenderingSystemCommon.h"

#if !defined(LOCATOR_IMPLEMENTATIONS)
#error "Locator interface implementations should only be included in Locator.cpp, use interface instead."
#endif

namespace openblack::ecs::systems
{

class RenderingSystemTemple final: public RenderingSystemCommon
{
public:
	~RenderingSystemTemple();

private:
	void PrepareDrawDescs(bool drawBoundingBox) override;
	void PrepareDrawUploadUniforms(bool drawBoundingBox) override;
	std::set<ecs::components::TempleRoom> _loadedRooms;

	/// Reusable scratch containers — cleared at the start of each PrepareDrawDescs /
	/// PrepareDrawUploadUniforms call to avoid per-frame heap allocations while
	/// still benefiting from previously reserved capacity.
	std::unordered_map<entt::id_type, std::pair<uint32_t, bool>> _scratchMeshIds;
	std::map<entt::id_type, uint32_t> _scratchUniformOffsets;
};
} // namespace openblack::ecs::systems
