/******************************************************************************
 * Copyright (c) 2018-2026 openblack developers
 *
 * For a complete list of all authors, please refer to contributors.md
 * Interested in contributing? Visit https://github.com/openblack/openblack
 *
 * openblack is licensed under the GNU General Public License version 3.
 *******************************************************************************/

#pragma once

#include <cstdint>

#include <array>
#include <vector>

#include <entt/fwd.hpp>
#include <glm/fwd.hpp>
#include <glm/vec2.hpp>

namespace openblack::ecs
{

class MapInterface
{
public:
	using CellId = glm::u16vec2;

	static constexpr float k_PositionToGridFactor = static_cast<float>(0x10000) * 0.1f;
	static constexpr glm::u16vec2 k_GridSize = {0x200, 0x200};

	static CellId GetGridCell(const glm::vec2& pos);
	static CellId GetGridCell(const glm::vec3& pos);
	static glm::vec2 GetCellCenter(const CellId& cellId);

	[[nodiscard]] virtual const std::vector<entt::entity>& GetFixedInGridCell(const CellId& cellId) const = 0;
	[[nodiscard]] virtual const std::vector<entt::entity>& GetFixedInGridCell(const glm::vec3& pos) const = 0;
	[[nodiscard]] virtual const std::vector<entt::entity>& GetMobileInGridCell(const CellId& cellId) const = 0;
	[[nodiscard]] virtual const std::vector<entt::entity>& GetMobileInGridCell(const glm::vec3& pos) const = 0;

	/// Rebuild both the fixed and mobile grids (call on scene load or when entities are added/removed).
	virtual void Rebuild() = 0;
	/// Rebuild only the mobile grid (call every game-logic tick – much cheaper than a full rebuild).
	virtual void RebuildMobile() = 0;

private:
	virtual void Clear() = 0;
	virtual void ClearMobile() = 0;
	virtual void Build() = 0;
	virtual void BuildMobile() = 0;
};

} // namespace openblack::ecs
