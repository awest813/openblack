/******************************************************************************
 * Copyright (c) 2018-2026 openblack developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/openblack/openblack
 *
 * openblack is licensed under the GNU General Public License version 3.
 *******************************************************************************/

#define LOCATOR_IMPLEMENTATIONS
#include "MapProduction.h"

#include <glm/gtx/component_wise.hpp>
#include <glm/gtx/norm.hpp>
#include <glm/gtx/vec_swizzle.hpp>
#include <glm/vec3.hpp>

#include "ECS/Components/Fixed.h"
#include "ECS/Components/Mobile.h"
#include "ECS/Components/Transform.h"
#include "ECS/Registry.h"
#include "Locator.h"

using namespace openblack::ecs;
using namespace openblack::ecs::components;

const std::vector<entt::entity>& MapProduction::GetFixedInGridCell(const CellId& cellId) const
{
	return _fixedGrid.at(cellId.x + cellId.y * k_GridSize.x);
}

const std::vector<entt::entity>& MapProduction::GetFixedInGridCell(const glm::vec3& pos) const
{
	const auto cellId = GetGridCell(pos);
	return GetFixedInGridCell(cellId);
}

const std::vector<entt::entity>& MapProduction::GetMobileInGridCell(const CellId& cellId) const
{
	return _mobileGrid.at(cellId.x + cellId.y * k_GridSize.x);
}

const std::vector<entt::entity>& MapProduction::GetMobileInGridCell(const glm::vec3& pos) const
{
	const auto cellId = GetGridCell(pos);
	return GetMobileInGridCell(cellId);
}

void MapProduction::Rebuild()
{
	Clear();
	Build();
}

void MapProduction::RebuildMobile()
{
	ClearMobile();
	BuildMobile();
}

void MapProduction::Clear()
{
	for (auto& g : _fixedGrid)
	{
		g.clear();
	}
	ClearMobile();
}

void MapProduction::ClearMobile()
{
	// Only clear cells that were populated last tick to avoid iterating all
	// 512×512 = 262 144 cells when most of them are empty.
	for (const uint32_t cellIdx : _dirtyMobileCells)
	{
		_mobileGrid[cellIdx].clear();
	}
	_dirtyMobileCells.clear();
}

void MapProduction::Build()
{
	auto& registry = Locator::entitiesRegistry::value();
	registry.Each<const Fixed, const Transform>([this](entt::entity entity, const Fixed& fixed, const Transform& transform) {
		// TODO(bwrsandman): This is only in the case of a square bb underling the bounding circle (x/z) <= 1.4
		const float radius = fixed.boundingRadius * glm::compMax(transform.scale) + 1.0f;
		const auto min = GetGridCell(fixed.boundingCenter - radius);
		const auto max = GetGridCell(fixed.boundingCenter + radius);

		for (uint16_t x = min.x; x < max.x + 1; ++x)
		{
			for (uint16_t y = min.y; y < max.y + 1; ++y)
			{
				const auto cellId = MapProduction::CellId(x, y);
				if (glm::distance2(GetCellCenter(cellId), fixed.boundingCenter) < radius * radius)
				{
					auto& cell = _fixedGrid.at(cellId.x + cellId.y * k_GridSize.x);
					cell.push_back(entity);
				}
			}
		}
	});
	BuildMobile();
}

void MapProduction::BuildMobile()
{
	auto& registry = Locator::entitiesRegistry::value();
	registry.Each<const Mobile, const Transform>(
	    [this](entt::entity entity, [[maybe_unused]] const Mobile& mobile, const Transform& transform) {
		    const auto cellId = GetGridCell(transform.position);
		    const uint32_t cellIdx = cellId.x + cellId.y * k_GridSize.x;
		    auto& cell = _mobileGrid[cellIdx];
		    // Track this cell as dirty so ClearMobile() can skip empty cells.
		    if (cell.empty())
		    {
			    _dirtyMobileCells.push_back(cellIdx);
		    }
		    cell.push_back(entity);
	    });
}
