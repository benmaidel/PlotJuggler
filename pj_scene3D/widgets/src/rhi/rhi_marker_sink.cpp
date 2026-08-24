// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_marker_sink.h"

#include <utility>

namespace pj::scene3d::rhi {

void RhiMarkerSink::setActive(std::shared_ptr<const DecodedSceneEntities> markers) {
  batch_ = std::move(markers);
  if (pass_ != nullptr) {
    pass_->setActive(batch_);
  }
}

void RhiMarkerSink::setOverrides(const MarkerDisplayOverrides& overrides) {
  if (pass_ == nullptr) {
    return;
  }
  // The pass's overrides struct is layout-identical by construction — both name the
  // same viewer preferences — but they are distinct types, so copy field by field
  // rather than casting.
  RhiMarkerPass::DisplayOverrides out;
  out.opacity = overrides.opacity;
  out.color_override = overrides.color_override;
  out.override_color = overrides.override_color;
  out.wireframe = overrides.wireframe;
  pass_->setOverrides(out);
}

void RhiMarkerSink::setVisible(bool visible) {
  if (pass_ != nullptr) {
    pass_->setVisible(visible);
  }
}

std::vector<std::string> RhiMarkerSink::frameNames() const {
  if (batch_ == nullptr) {
    return {};
  }
  return batch_->frames;
}

void RhiMarkerSink::setFrameTransforms(std::vector<std::optional<glm::mat4>> transforms) {
  if (pass_ != nullptr) {
    pass_->setFrameTransforms(std::move(transforms));
  }
}

}  // namespace pj::scene3d::rhi
