#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <rhi/qrhi.h>

namespace pj::scene3d::rhi {

/// The directional-light shadow map's TARGET: a square depth texture plus the
/// depth-only render target that writes it. The QRhi counterpart of ShadowMapPass.
///
/// Ownership is split exactly as on OpenGL, and for the same reason: this class owns
/// only the target, because the target is one-per-VIEW, while the caster geometry
/// lives in whichever RhiMeshPass owns those meshes. So the view calls begin(), asks
/// each layer to draw its casters through RhiMeshPass::drawDepthOnly(), then end().
///
/// Depth is a TEXTURE rather than a renderbuffer because the receiver samples it —
/// the same constraint that forced the HDR chain's depth to be a texture, and worth
/// remembering: QRhi accepts a renderbuffer here and then silently has nothing to
/// bind.
///
/// Allocation is LAZY (ensure() is only called once a frame actually has mesh
/// casters), so a point-cloud-only scene never pays for a 16 MB depth map.
class RhiShadowMapPass {
 public:
  RhiShadowMapPass() = default;
  ~RhiShadowMapPass();

  RhiShadowMapPass(const RhiShadowMapPass&) = delete;
  RhiShadowMapPass& operator=(const RhiShadowMapPass&) = delete;

  /// Create the depth texture and its render target if absent. Idempotent; false
  /// leaves the pass not ready() and the caller must render unshadowed.
  [[nodiscard]] bool ensure(QRhi& rhi);

  void release();

  [[nodiscard]] bool ready() const {
    return render_target_ != nullptr;
  }

  /// The map to bind on the receiver. Null until ensure() succeeds.
  [[nodiscard]] QRhiTexture* depthTexture() const {
    return depth_;
  }

  /// Needed by RhiMeshPass to build its depth-only pipeline, which must be compiled
  /// against THIS target's render-pass descriptor.
  [[nodiscard]] QRhiRenderPassDescriptor* renderPassDescriptor() const {
    return rpd_;
  }
  [[nodiscard]] QRhiTextureRenderTarget* renderTarget() const {
    return render_target_;
  }

  /// Begin the depth-only pass: clears depth to 1.0 and sets the full-map viewport.
  /// Caster draws are issued between this and end() by the mesh passes.
  void begin(QRhiCommandBuffer& cb, QRhiResourceUpdateBatch* updates = nullptr);
  void end(QRhiCommandBuffer& cb);

 private:
  QRhi* rhi_ = nullptr;
  QRhiTexture* depth_ = nullptr;
  QRhiTextureRenderTarget* render_target_ = nullptr;
  QRhiRenderPassDescriptor* rpd_ = nullptr;
};

}  // namespace pj::scene3d::rhi
