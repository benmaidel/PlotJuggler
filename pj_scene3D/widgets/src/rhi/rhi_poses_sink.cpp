// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_poses_sink.h"

#include <utility>

namespace pj::scene3d::rhi {

void RhiPosesSink::setInstances(std::vector<PoseTriadInstance> instances) {
  if (pass_ != nullptr) {
    pass_->setInstances(std::move(instances));
  }
}

void RhiPosesSink::setFrameTransform(const glm::mat4& fixed_from_source) {
  if (pass_ != nullptr) {
    pass_->setFrameWorld(fixed_from_source);
  }
}

}  // namespace pj::scene3d::rhi
