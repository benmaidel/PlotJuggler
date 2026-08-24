#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between a SceneEntities LAYER (which decodes marker
// batches) and a render pass (which uploads and draws).

#include <glm/glm.hpp>
#include <memory>

#include "pj_scene3d_core/scene_entities_render.h"  // DecodedSceneEntities

namespace pj::scene3d {

/// Viewer-side display overrides. A per-topic PREFERENCE, not marker data: the
/// marker protocol has no notion of opacity, recolouring or wireframe.
struct MarkerDisplayOverrides {
  /// Multiplies every primitive's alpha.
  float opacity = 1.0F;
  bool color_override = false;
  glm::vec4 override_color{1.0F};
  /// Draw solids and triangle batches as edges rather than filled faces.
  bool wireframe = false;
};

/// Where a SceneEntitiesLayer sends its decoded batch and display state.
///
/// Decoding a SceneEntities message into the render structs is already
/// backend-agnostic (`scene_entities_decode`), and the batch's primitives are
/// FRAME-LOCAL with an interned frame table. Resolving those frames is not part of
/// this interface because the backends differ: the OpenGL pass resolves them itself
/// from its FrameContext, while the QRhi pass needs the transforms pushed.
class IMarkerSink {
 public:
  virtual ~IMarkerSink() = default;

  IMarkerSink(const IMarkerSink&) = delete;
  IMarkerSink& operator=(const IMarkerSink&) = delete;

  virtual void setActive(std::shared_ptr<const DecodedSceneEntities> markers) = 0;
  virtual void setOverrides(const MarkerDisplayOverrides& overrides) = 0;
  virtual void setVisible(bool visible) = 0;

 protected:
  IMarkerSink() = default;
};

}  // namespace pj::scene3d
