#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <rhi/qrhi.h>

#include <QSize>
#include <array>
#include <glm/glm.hpp>

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"
#include "pj_scene3d_widgets/scene_look_defaults.h"

namespace pj::scene3d::rhi {

/// Screen-space ambient occlusion over the resolved single-sample depth: the QRhi
/// counterpart of SsaoPass. Two R16F targets — raw hemisphere-kernel AO, then a 4x4
/// box blur — and the composite multiplies the blurred result into the HDR colour
/// before tonemapping.
///
/// Deliberately NOT an IRhiRenderPass. That interface records draws into a render
/// pass the widget has already begun, whereas this owns two render passes of its
/// own and must run BETWEEN the scene pass and the composite. Forcing it into the
/// same shape would mean pretending an off-screen chain is a geometry draw.
///
/// It needs no knowledge of NDC conventions: view/screen reconstruction goes
/// through the matrices in RhiFrameContext, which the host builds once.
class RhiSsaoPass {
 public:
  RhiSsaoPass();
  ~RhiSsaoPass();

  RhiSsaoPass(const RhiSsaoPass&) = delete;
  RhiSsaoPass& operator=(const RhiSsaoPass&) = delete;

  /// (Re)build targets and pipelines for `pixel_size`. Cheap no-op when nothing
  /// changed, so it is safe to call every frame. False means the pass is
  /// unavailable and the composite must skip the AO multiply.
  [[nodiscard]] bool ensure(QRhi& rhi, const QSize& pixel_size);

  /// The resolved single-sample depth to sample. Null disables the pass.
  void setDepthTexture(QRhiTexture* depth);

  /// Sampling radius in metres and the occlusion contrast exponent.
  void setRadius(float radius_m);
  void setPower(float ao_power);

  /// Record both passes. Must be called OUTSIDE any render pass — it opens its own.
  void render(QRhiCommandBuffer& cb, const RhiFrameContext& ctx);

  /// Blurred AO (R16F, 1 = unoccluded), or null when the pass is not ready.
  [[nodiscard]] QRhiTexture* output() const;
  [[nodiscard]] bool ready() const;

  void release();

 private:
  /// Kernel size is baked into ssao.frag's loop, so the two must agree.
  static constexpr int kKernelSize = 32;

  /// std140 layout of ssao.frag's SsaoUbo. The kernel is vec4-per-sample because
  /// std140 pads an array element to 16 bytes regardless.
  struct SsaoUbo {
    float screen_from_view[16];
    float view_from_screen[16];
    float kernel[kKernelSize][4];
    float texel[2];
    float radius;
    float bias;
    float ao_power;
    float pad0;
    float pad1;
    float pad2;
  };
  struct BlurUbo {
    float texel[2];
    float pad0;
    float pad1;
  };
  static_assert(sizeof(BlurUbo) == 16);

  /// One off-screen R16F stage: texture, its render target, and the pipeline that
  /// fills it.
  struct Stage {
    QRhiTexture* texture = nullptr;
    QRhiTextureRenderTarget* rt = nullptr;
    QRhiRenderPassDescriptor* rpd = nullptr;
    QRhiGraphicsPipeline* pipeline = nullptr;
    QRhiShaderResourceBindings* srb = nullptr;
    QRhiBuffer* ubo = nullptr;
  };

  bool buildStage(
      QRhi& rhi, Stage& stage, const QSize& size, const char* frag_path, quint32 ubo_size, QRhiTexture* input,
      QRhiSampler* sampler);
  void releaseStage(Stage& stage);
  void rebuildInputBindings();

  QRhi* rhi_ = nullptr;
  QSize size_;
  /// Nearest sampling throughout: both stages read a value-per-texel field where
  /// interpolation would smear depth across silhouettes.
  QRhiSampler* sampler_ = nullptr;
  Stage ao_;
  Stage blur_;
  QRhiTexture* depth_ = nullptr;
  /// Hemisphere kernel, generated once — the distribution is what makes the
  /// occlusion estimate unbiased, so it must not be regenerated per frame.
  std::array<glm::vec3, kKernelSize> kernel_{};
  float radius_m_ = look::kSsaoRadiusM;
  float ao_power_ = look::kSsaoPower;
  bool input_bindings_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
