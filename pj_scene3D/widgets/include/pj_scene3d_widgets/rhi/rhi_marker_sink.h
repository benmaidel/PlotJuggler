#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <string>
#include <vector>

#include "pj_scene3d_widgets/marker_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"

namespace pj::scene3d::rhi {

/// Adapts a SceneEntitiesLayer's decoded batch onto RhiMarkerPass.
///
/// Frame placement differs from the other adapters, and this is the interesting
/// part. A marker batch is not placed by ONE transform: its primitives are
/// frame-local against an interned frame TABLE, so each entry needs resolving
/// separately and a batch may legitimately be only partly resolvable. The adapter
/// therefore exposes the frame names it is waiting on, and takes a vector of
/// optionals back — nullopt meaning "this frame did not resolve", which the pass
/// already treats as "skip those primitives" rather than drawing them at the origin.
class RhiMarkerSink final : public IMarkerSink {
 public:
  explicit RhiMarkerSink(RhiMarkerPass& pass) : pass_(&pass) {}

  void setActive(std::shared_ptr<const DecodedSceneEntities> markers) override;
  void setOverrides(const MarkerDisplayOverrides& overrides) override;
  void setVisible(bool visible) override;

  /// The interned frame names of the active batch, in `frame_index` order. Empty
  /// when no batch is set. The caller resolves these and hands the results to
  /// setFrameTransforms() in the SAME order.
  [[nodiscard]] std::vector<std::string> frameNames() const;

  /// Resolved fixed_frame <- frame transforms, indexed by `frame_index`. Must be
  /// re-pushed as TF moves.
  void setFrameTransforms(std::vector<std::optional<glm::mat4>> transforms);

 private:
  RhiMarkerPass* pass_ = nullptr;
  std::shared_ptr<const DecodedSceneEntities> batch_;
};

}  // namespace pj::scene3d::rhi
