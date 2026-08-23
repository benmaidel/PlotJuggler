#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Composites the resolved HDR scene colour onto the widget's own target with a
/// fullscreen triangle.
///
/// Implements the same IRhiRenderPass contract as the geometry passes, but is
/// initialized against a DIFFERENT render pass descriptor: the geometry passes are
/// built for the off-screen HDR target, this one for the widget's target. Keeping
/// the interface shared is what lets the widget drive both with one loop.
///
/// Today it applies exposure and copies. The GL renderer's composite additionally
/// does tonemap (ACES/AgX/Neutral), saturation, a manual sRGB encode, a
/// background bypass for far-plane pixels, and multiplies in SSAO/EDL; porting
/// those belongs with the passes that produce their inputs.
class RhiPresentPass final : public IRhiRenderPass {
 public:
  RhiPresentPass() = default;
  ~RhiPresentPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// The resolved single-sample colour to composite. Must be set before draw();
  /// the binding is rebuilt whenever the texture identity changes, which happens
  /// on every resize because the HDR chain re-creates its textures.
  void setSourceTexture(QRhiTexture* texture);
  void setExposure(float exposure) { exposure_ = exposure; }

 private:
  /// Mirrors the PresentUbo block in shaders/present.frag. std140 rounds the block
  /// up to 16 bytes, hence the explicit padding.
  struct alignas(16) PresentUbo {
    float exposure;
    float flip_v;
    float pad0;
    float pad1;
  };
  static_assert(sizeof(PresentUbo) == 16, "PresentUbo must match the std140 block layout");

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* ubo_ = nullptr;
  QRhiSampler* sampler_ = nullptr;
  /// 1x1 stand-in bound at pipeline-creation time so the SRB layout is final.
  QRhiTexture* placeholder_tex_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;
  QRhiTexture* source_ = nullptr;
  bool bindings_dirty_ = true;
  float exposure_ = 1.0F;
  /// Cached from QRhi::isYUpInFramebuffer() at initialize(); see present.frag.
  bool flip_v_ = false;
};

}  // namespace pj::scene3d::rhi
