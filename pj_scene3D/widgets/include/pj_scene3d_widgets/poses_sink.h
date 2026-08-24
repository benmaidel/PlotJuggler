#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between a pose-array LAYER (which decodes and expands
// poses into arm instances) and a render pass (which uploads and draws).

#include <vector>

#include "pj_scene3d_core/poses_in_frame_render.h"  // PoseTriadInstance

namespace pj::scene3d {

/// Where a PosesInFrameLayer sends its expanded arm instances.
///
/// The narrowest of the layer sinks, because almost everything a pose array needs is
/// already backend-agnostic: decoding, the triad-vs-single-arm expansion, styling and
/// colour override all happen in core (`buildPoseTriadInstances`), leaving only the
/// upload behind a backend.
///
/// Instances are FRAME-LOCAL by contract. Their placement is a separate concern the
/// two backends handle differently — the OpenGL pass takes it as a render-time
/// argument, the QRhi pass as a uniform — so it is not part of this interface, and an
/// adapter has to be given it by whoever resolves TF.
class IPosesSink {
 public:
  virtual ~IPosesSink() = default;

  IPosesSink(const IPosesSink&) = delete;
  IPosesSink& operator=(const IPosesSink&) = delete;

  virtual void setInstances(std::vector<PoseTriadInstance> instances) = 0;

 protected:
  IPosesSink() = default;
};

}  // namespace pj::scene3d
