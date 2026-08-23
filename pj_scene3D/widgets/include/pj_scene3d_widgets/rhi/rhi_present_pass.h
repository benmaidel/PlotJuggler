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
/// Applies the GL renderer's composite operator chain: exposure -> tonemap
/// (None/ACES/AgX/Khronos PBR Neutral) -> saturation -> the single manual sRGB
/// encode, with the far-plane background bypass and the per-pixel annotation
/// bypass. The SSAO/EDL multiplies are the one part still missing, because the
/// passes producing those inputs are not ported.
///
/// This pass is what makes the whole renderer linear-light: every geometry pass
/// writes LINEAR colour into the HDR target precisely because the encode happens
/// here, exactly once. Changing that convention means changing every pass.
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

  /// The resolved single-sample depth, used only for the far-plane background
  /// bypass. Pass nullptr when the HDR chain could not produce one (no MSAA, or
  /// QRhi::ResolveDepthStencil unsupported); the bypass then relies on the host
  /// clearing the target's alpha to 0 instead — see backgroundNeedsAlphaClear().
  void setDepthTexture(QRhiTexture* texture);

  /// Composite operator settings, mirroring SceneViewWidget::CompositeParams.
  struct CompositeParams {
    /// 0 None, 1 ACES, 2 AgX, 3 Khronos PBR Neutral.
    int tonemap_mode = 1;
    float exposure = 1.1F;
    /// Applied AFTER the tonemap: ACES desaturates, so this restores the punch
    /// rather than pre-boosting colour into the tonemap's shoulder. Reordering
    /// these two changes the look.
    float saturation = 1.2F;
  };
  void setCompositeParams(const CompositeParams& params);
  [[nodiscard]] const CompositeParams& compositeParams() const {
    return params_;
  }

  /// True when no resolved depth is bound, so the host must clear the HDR target's
  /// alpha to 0 for the background to escape grading. With depth available the
  /// clear alpha should be 1, which is what keeps translucent data drawn over the
  /// background fully graded.
  [[nodiscard]] bool backgroundNeedsAlphaClear() const {
    return depth_ == nullptr;
  }

 private:
  /// Mirrors the PresentUbo block in shaders/present.frag. std140 rounds the block
  /// up to a multiple of 16 bytes, hence the explicit padding.
  struct alignas(16) PresentUbo {
    float exposure;
    float flip_v;
    int tonemap_mode;
    float saturation;
    float has_depth;
    float pad0;
    float pad1;
    float pad2;
  };
  static_assert(sizeof(PresentUbo) == 32, "PresentUbo must match the std140 block layout");

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* ubo_ = nullptr;
  QRhiSampler* sampler_ = nullptr;
  /// Depth needs NEAREST filtering of its own; see the comment at its creation.
  QRhiSampler* depth_sampler_ = nullptr;
  /// 1x1 stand-ins bound at pipeline-creation time so the SRB layout is final.
  QRhiTexture* placeholder_tex_ = nullptr;
  QRhiTexture* depth_placeholder_tex_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;
  QRhiTexture* source_ = nullptr;
  QRhiTexture* depth_ = nullptr;
  bool bindings_dirty_ = true;
  CompositeParams params_;
  /// Cached from QRhi::isYUpInFramebuffer() at initialize(); see present.frag.
  bool flip_v_ = false;
};

}  // namespace pj::scene3d::rhi
