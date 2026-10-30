module;

#include "platform/platform.h"

#include <external/config.hpp>
#include <external/nlohmann/json.hpp>

#include "core/systems/schema_macros.h"

module velocity;

import <cmath>;
import <cstddef>;
import <filesystem>;
import <limits>;
import <string_view>;
import <type_traits>;

namespace systems {

namespace {

bool IsFinite(const math::Vector3& value) {
  return std::isfinite(value.x) && std::isfinite(value.y) &&
         std::isfinite(value.z);
}

}  // namespace

Bounds::Data Bounds::Get(std::uintptr_t entity) {
  const auto game_scene_node = memory::Read<std::uintptr_t>(
      entity + SCHEMA("C_BaseEntity", "m_pGameSceneNode"_hash));
  if (!game_scene_node) {
    return {};
  }

  // m_vecOrigin is a quantized network-origin type in current CS2 builds,
  // not a plain vector3. Use the evaluated world-space origin instead.
  const auto origin = memory::Read<math::Vector3>(
      game_scene_node + SCHEMA("CGameSceneNode", "m_vecAbsOrigin"_hash));
  if (!IsFinite(origin)) {
    return {};
  }

  const auto project_box = [](const math::Vector3& mins,
                              const math::Vector3& maxs) -> Data {
    const auto [viewport_w, viewport_h] = xdraw::viewport_size();
    const auto sw = static_cast<float>(viewport_w);
    const auto sh = static_cast<float>(viewport_h);

    Data result{};
    result.min = {FLT_MAX, FLT_MAX};
    result.max = {-FLT_MAX, -FLT_MAX};

    constexpr auto kEdgeMargin = 384.0f;
    auto had_valid_projection = false;

    for (std::size_t i = 0; i < 8; ++i) {
      const math::Vector3 corner{(i & 1) ? maxs.x : mins.x,
                                 (i & 2) ? maxs.y : mins.y,
                                 (i & 4) ? maxs.z : mins.z};

      const auto proj = g_view.ProjectFull(corner);
      if (proj.w <= 0.001f || !std::isfinite(proj.screen.x) ||
          !std::isfinite(proj.screen.y)) {
        continue;
      }

      const auto sx =
          std::clamp(proj.screen.x, -kEdgeMargin, sw + kEdgeMargin);
      const auto sy =
          std::clamp(proj.screen.y, -kEdgeMargin, sh + kEdgeMargin);

      result.min.x = std::min(result.min.x, sx);
      result.min.y = std::min(result.min.y, sy);
      result.max.x = std::max(result.max.x, sx);
      result.max.y = std::max(result.max.y, sy);
      had_valid_projection = true;
    }

    if (!had_valid_projection || result.min.x >= result.max.x ||
        result.min.y >= result.max.y) {
      return {};
    }

    result.valid = true;
    return result;
  };

  auto mins = math::Vector3{};
  auto maxs = math::Vector3{};
  auto have_box{false};

  if (const auto collision = memory::Read<std::uintptr_t>(
          entity + SCHEMA("C_BaseEntity", "m_pCollision"_hash))) {
    const auto local_mins = memory::Read<math::Vector3>(
        collision + SCHEMA("CCollisionProperty", "m_vecMins"_hash));
    const auto local_maxs = memory::Read<math::Vector3>(
        collision + SCHEMA("CCollisionProperty", "m_vecMaxs"_hash));
    const auto extents = local_maxs - local_mins;

    // A zero/absurd hull usually means a stale collision property or a schema
    // offset that did not resolve. Fall through to the skeleton in that case.
    constexpr auto kMaxHullExtent = 512.0f;
    if (IsFinite(local_mins) && IsFinite(local_maxs) &&
        extents.x >= 0.1f && extents.y >= 0.1f && extents.z >= 0.1f &&
        extents.x <= kMaxHullExtent && extents.y <= kMaxHullExtent &&
        extents.z <= kMaxHullExtent) {
      mins = origin + local_mins;
      maxs = origin + local_maxs;
      have_box = true;
    }
  }

  if (!have_box) {
    // Collision data is the primary source, but an invalid hull must not make
    // every bounds-dependent ESP element disappear. Build a hull from the
    // evaluated bones instead.
    const auto skeleton = g_bones.GetSkeleton(entity);
    auto bone_min = math::Vector3{FLT_MAX, FLT_MAX, FLT_MAX};
    auto bone_max = math::Vector3{-FLT_MAX, -FLT_MAX, -FLT_MAX};
    auto bone_count{0};

    for (const auto& bone : skeleton) {
      const auto& position = bone.position;
      if (!IsFinite(position) || position.LengthSqr() < 1.0f) {
        continue;
      }

      bone_min.x = std::min(bone_min.x, position.x);
      bone_min.y = std::min(bone_min.y, position.y);
      bone_min.z = std::min(bone_min.z, position.z);
      bone_max.x = std::max(bone_max.x, position.x);
      bone_max.y = std::max(bone_max.y, position.y);
      bone_max.z = std::max(bone_max.z, position.z);
      ++bone_count;
    }

    if (bone_count >= 3) {
      constexpr auto kPaddingXy = 3.0f;
      constexpr auto kPaddingZ = 2.0f;
      mins = bone_min - math::Vector3{kPaddingXy, kPaddingXy, kPaddingZ};
      maxs = bone_max + math::Vector3{kPaddingXy, kPaddingXy, kPaddingZ};
      have_box = true;
    }
  }

  if (!have_box) {
    // Last resort: the standard standing player hull around the scene-node
    // origin. This keeps ESP usable before collision/bones are evaluated.
    constexpr auto kHalfWidth = 10.0f;
    constexpr auto kHeight = 72.0f;
    mins = origin + math::Vector3{-kHalfWidth, -kHalfWidth, 0.0f};
    maxs = origin + math::Vector3{kHalfWidth, kHalfWidth, kHeight};
  }

  return project_box(mins, maxs);
}

}  // namespace systems
