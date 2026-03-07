/******************************************************************************
 * Copyright (c) 2018-2026 openblack developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/openblack/openblack
 *
 * openblack is licensed under the GNU General Public License version 3.
 *******************************************************************************/

#pragma once

#if !defined(LOCATOR_IMPLEMENTATIONS)
#error "Locator interface implementations should only be included in Locator.cpp, use interface instead."
#endif

#include <vector>

#include "Map.h"

namespace openblack::ecs
{

class MapProduction final: public MapInterface
{
	[[nodiscard]] const std::vector<entt::entity>& GetFixedInGridCell(const CellId& cellId) const override;
	[[nodiscard]] const std::vector<entt::entity>& GetFixedInGridCell(const glm::vec3& pos) const override;
	[[nodiscard]] const std::vector<entt::entity>& GetMobileInGridCell(const CellId& cellId) const override;
	[[nodiscard]] const std::vector<entt::entity>& GetMobileInGridCell(const glm::vec3& pos) const override;

	void Rebuild() override;
	void RebuildMobile() override;

private:
	void Clear() override;
	void ClearMobile() override;
	void Build() override;
	void BuildMobile() override;

	/// Grid cell storage uses vectors for cache-friendly linear iteration.
	/// Fixed entities (obstacles) don't move so their grid is rebuilt only on Rebuild().
	/// The mobile grid is rebuilt every game-logic tick via RebuildMobile().
	std::array<std::vector<entt::entity>, k_GridSize.x * k_GridSize.y> _fixedGrid;
	std::array<std::vector<entt::entity>, k_GridSize.x * k_GridSize.y> _mobileGrid;

	/// Indices of cells that currently contain at least one mobile entity.
	/// Used to avoid iterating all 512×512 = 262 144 cells on every tick.
	std::vector<uint32_t> _dirtyMobileCells;
};

} // namespace openblack::ecs
