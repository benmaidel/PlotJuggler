#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstddef>

namespace pj::scene3d::rhi {

/// The GPU-side contract of shaders/arrow.{vert,frag}, shared by every
/// instanced-arrow pass (TF triads, PoseArray triads).
///
/// It lives in one header rather than being repeated per pass because a silent
/// drift between two copies of a std140 block or a vertex stride is exactly the
/// class of bug QRhi does not report — it renders garbage or nothing.
namespace arrow {

/// Resource paths of the baked shader pack (see qt6_add_shaders in the module's
/// CMakeLists.txt).
inline constexpr const char* kVertShader = ":/scene3d_shaders/arrow.vert.qsb";
inline constexpr const char* kFragShader = ":/scene3d_shaders/arrow.frag.qsb";

/// std140 layout of arrow.vert's `ArrowUbo`. Bound to both stages, since the
/// fragment shader must declare the identical block.
struct Ubo {
  float view_proj[16];
  /// Common parent-frame placement applied on top of each instance's model.
  /// Passes whose instances are already world-space write identity here.
  float frame_world[16];
};
static_assert(sizeof(Ubo) == 128, "ArrowUbo must stay a pair of tightly packed mat4s");

/// One instanced arm: a model matrix followed by an RGBA colour. This is also the
/// per-instance vertex stride, and it matches the attribute declarations at
/// locations 2-6 in arrow.vert.
struct Instance {
  float model[16];
  float color[4];
};
static_assert(sizeof(Instance) == 80, "Instance must match the per-instance vertex stride");
static_assert(offsetof(Instance, color) == 64, "colour must follow the four model columns");

/// Byte offset of each vertex attribute within an Instance.
inline constexpr std::size_t kModelColumnStride = 4 * sizeof(float);
inline constexpr std::size_t kColorOffset = offsetof(Instance, color);

}  // namespace arrow

}  // namespace pj::scene3d::rhi
