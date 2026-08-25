#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between a mesh-owning LAYER (which resolves TF and
// bakes per-object world transforms) and a render pass (which uploads and draws).

#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "pj_scene3d_widgets/mesh_data.h"

namespace pj::scene3d {

/// What a draw call resolves to. kMesh looks `mesh_key` up in the sink's keyed
/// store; the rest are procedural primitives built from `mesh_primitives.h`, so both
/// backends draw identical shapes. kPlaceholderCube stands in for a mesh whose async
/// load has not finished — the layer re-emits the draw as kMesh once it has.
enum class MeshGeometryKind {
  kMesh,
  kBox,
  kCylinder,
  kSphere,
  kPlaceholderCube,
};

/// One object to draw. `model` is WORLD-space already: unlike every other layer
/// sink, nothing here carries a frame transform, because a mesh scene spans many TF
/// frames and the layer resolves each one while building the list (see IMeshSink).
struct MeshDrawCall {
  MeshGeometryKind kind{MeshGeometryKind::kPlaceholderCube};
  /// Key into the sink's keyed mesh store; meaningful only for kMesh.
  std::string mesh_key;
  glm::mat4 model{1.0f};
  glm::vec4 color{0.7f, 0.7f, 0.7f, 1.0f};
  /// Take base colour from the mesh's own vertex colours / material rather than
  /// from `color`.
  bool use_vertex_color{true};
};

/// Where a mesh-owning layer sends its geometry and its per-frame draw list.
///
/// Two things make this seam differ from the other layer sinks, both worth knowing
/// before implementing one:
///
/// **Placement is baked, not pushed.** A cloud or an occupancy grid pushes ONE frame
/// transform beside its payload, because applying it on the GPU is far cheaper than
/// re-transforming N points per TF tick. A mesh scene inverts that trade: its draws
/// span many TF frames (one per URDF link), and there are tens of them rather than
/// millions of points, so the layer resolves each link's transform itself and bakes
/// the result into `MeshDrawCall::model`. Draws therefore arrive world-space and this
/// interface needs no placement call at all.
///
/// **Geometry and draws have different lifetimes.** setMeshData() is expensive and
/// rare (an async import completing); the draw list is cheap and re-pushed whenever
/// TF, the model, or the render origin moves. Implementations should treat a keyed
/// mesh as retained until clearMeshes(), and the draw lists as replaced wholesale.
///
/// NOT in this interface: shadow casting. The depth-only pre-pass takes its draws as
/// a render-time argument and exists only on the OpenGL backend, so it stays behind
/// Scene3DLayer::renderShadowCasters() rather than being forced into a seam whose
/// QRhi side has no counterpart.
class IMeshSink {
 public:
  virtual ~IMeshSink() = default;

  IMeshSink(const IMeshSink&) = delete;
  IMeshSink& operator=(const IMeshSink&) = delete;

  /// Store or replace a keyed mesh's CPU data. GPU upload is deferred to the next
  /// frame, so this is safe to call off the render thread (an import completing on
  /// the thread pool drains straight into here).
  virtual void setMeshData(const std::string& key, MeshData data) = 0;

  /// Forget every keyed mesh. Must be safe with no render context current: the
  /// layer calls it from detach() and on a model swap, on the GUI thread.
  virtual void clearMeshes() = 0;

  /// Replace the visual draw list.
  virtual void setVisualDraws(std::vector<MeshDrawCall> draws) = 0;
  /// Replace the collision-hull overlay draw list.
  virtual void setCollisionDraws(std::vector<MeshDrawCall> draws) = 0;

 protected:
  IMeshSink() = default;
};

}  // namespace pj::scene3d
