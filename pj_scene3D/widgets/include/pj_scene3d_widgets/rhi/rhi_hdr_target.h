#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <QSize>
#include <rhi/qrhi.h>

namespace pj::scene3d::rhi {

/// Off-screen multisample HDR render target the scene draws into, before a
/// fullscreen present resolves it onto the widget's own target.
///
/// This mirrors the OpenGL renderer's SceneHdrFbo and exists for the same two
/// reasons. First, HDR: an RGBA16F colour buffer lets the scene render in linear
/// light so exposure/tonemapping can be applied at composite time instead of
/// clipping at 1.0 during the geometry passes. Second, anti-aliasing that
/// survives docking: a QRhiWidget composited inside the app negotiates
/// sampleCount 1 for its own target, so MSAA must live on a chain this class owns
/// rather than being requested from the widget.
///
/// The resolved depth texture is the piece SSAO and EDL will need — they sample
/// single-sample depth — and it is why the design is viable at all: QRhi gained
/// depth resolve (QRhi::ResolveDepthStencil) which older versions lacked. It is
/// requested only when the backend reports that feature; everything else still
/// works without it.
class RhiHdrTarget {
 public:
  RhiHdrTarget() = default;
  ~RhiHdrTarget();

  RhiHdrTarget(const RhiHdrTarget&) = delete;
  RhiHdrTarget& operator=(const RhiHdrTarget&) = delete;

  /// (Re)build the chain for `pixel_size`. Cheap no-op when nothing changed, so it
  /// is safe to call every frame. Returns false when the chain is unavailable, in
  /// which case the caller must fall back to drawing straight into the widget.
  [[nodiscard]] bool ensure(QRhi& rhi, const QSize& pixel_size, int desired_samples);

  void release();

  [[nodiscard]] bool ready() const { return render_target_ != nullptr; }
  [[nodiscard]] QRhiTextureRenderTarget* renderTarget() const { return render_target_; }
  [[nodiscard]] QRhiRenderPassDescriptor* renderPassDescriptor() const { return rpd_; }

  /// Single-sample colour the present pass samples. Null until ensure() succeeds.
  [[nodiscard]] QRhiTexture* resolvedColor() const { return resolve_color_; }
  /// Single-sample depth for future screen-space passes; null when the backend
  /// cannot resolve depth.
  [[nodiscard]] QRhiTexture* resolvedDepth() const { return resolve_depth_; }

  /// Sample count actually in use — may be below the requested value, since QRhi
  /// only guarantees what QRhi::supportedSampleCounts() reports.
  [[nodiscard]] int sampleCount() const { return samples_; }

 private:
  QRhi* rhi_ = nullptr;
  QSize size_;
  int samples_ = 1;

  QRhiTexture* msaa_color_ = nullptr;
  QRhiRenderBuffer* msaa_depth_ = nullptr;
  QRhiTexture* resolve_color_ = nullptr;
  QRhiTexture* resolve_depth_ = nullptr;
  QRhiTextureRenderTarget* render_target_ = nullptr;
  QRhiRenderPassDescriptor* rpd_ = nullptr;
};

}  // namespace pj::scene3d::rhi
