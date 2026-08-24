#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_widgets/poses_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"

namespace pj::scene3d::rhi {

/// Adapts a PosesInFrameLayer's expanded arm instances onto RhiPosesPass.
///
/// Full parity, and the cleanest of the adapters: RhiPosesPass was built around the
/// same frame-local instance convention the layer already produces, so the instances
/// pass straight through with no repacking. Styling, colour override and the
/// triad-vs-single-arm choice all happened in core before this.
class RhiPosesSink final : public IPosesSink {
 public:
  explicit RhiPosesSink(RhiPosesPass& pass) : pass_(&pass) {}

  void setInstances(std::vector<PoseTriadInstance> instances) override;

  /// Places the poses' SOURCE frame into the fixed frame. Outside IPosesSink for the
  /// usual reason: the OpenGL pass takes it as a render-time argument, the QRhi pass
  /// as a uniform, so whoever resolves TF pushes it — and keeps pushing it, or poses
  /// in a moving frame freeze at their first pose.
  void setFrameTransform(const glm::mat4& fixed_from_source);

 private:
  RhiPosesPass* pass_ = nullptr;
};

}  // namespace pj::scene3d::rhi
