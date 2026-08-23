#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstddef>
#include <cstdint>
#include <vector>

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"
#include "pj_widgets/Colormap.h"

namespace pj::scene3d::rhi {

/// Point clouds, drawn as instanced camera-facing billboards.
///
/// This is the one pass that is a REDESIGN rather than a translation. The GL
/// renderer draws GL_POINTS and sets gl_PointSize in the vertex shader, reading
/// gl_PointCoord in the fragment shader to make round sphere imposters. QRhi
/// exposes no programmable point size (and Metal has none), so a "point" becomes
/// real geometry: one unit quad instanced per point, expanded in world space
/// around the point's centre and clipped to a disc in the fragment shader.
/// Expanding in world space (rather than screen space) preserves the perspective
/// size falloff gl_PointSize gave for free.
///
/// Colour comes from sampling pj_widgets' colormap LUT texture, not from
/// concatenating colormapGlsl() into the shader source as the GL renderer does —
/// that trick is incompatible with ahead-of-time shader baking, and the LUT is
/// what pj_scene2D's depth path already uses, so 2D and 3D agree by construction.
class RhiPointcloudPass final : public IRhiRenderPass {
 public:
  RhiPointcloudPass() = default;
  ~RhiPointcloudPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Describes where xyz and the colouring scalar sit inside one point record.
  /// Defaults match the contiguous float32 layout that lets a cloud's WIRE buffer
  /// be uploaded verbatim (xyz at 0, a float scalar at 12, 16-byte stride) — the
  /// same fast path the GL renderer recognises.
  struct Layout {
    int stride_bytes = 16;
    int xyz_offset = 0;
    int scalar_offset = 12;
  };

  /// Copy `count` point records from `data`. The bytes are taken as-is, so a
  /// matching wire buffer needs no conversion. Passing count 0 clears the cloud.
  void setPoints(const void* data, int count, const Layout& layout);

  /// Scalar range mapped across the colormap. An empty range renders flat.
  void setScalarRange(float min_value, float max_value);
  void setColormap(PJ::Colormap colormap) {
    colormap_ = colormap;
  }
  /// World-space radius of each point.
  void setPointRadius(float metres);
  /// Places the cloud's source frame into the fixed frame. Identity by default,
  /// which is only correct for data already expressed in the fixed frame.
  void setModelMatrix(const glm::mat4& model) {
    model_ = model;
  }

  [[nodiscard]] int pointCount() const {
    return point_count_;
  }

 private:
  /// Mirrors the PointcloudUbo block in the shaders. std140: mat4 at 0, the two
  /// vec4s at 64 and 80, then four floats at 96..111 — already a multiple of 16.
  struct alignas(16) PointcloudUbo {
    float view_proj[16];
    float model[16];
    float cam_right[4];
    float cam_up[4];
    float point_radius;
    float scalar_min;
    float scalar_max;
    float colormap_row;
  };
  static_assert(sizeof(PointcloudUbo) == 176, "PointcloudUbo must match the std140 block layout");

  bool ensureColormapTexture(QRhi& rhi, QRhiResourceUpdateBatch& updates);

  glm::mat4 model_{1.0F};
  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* instance_buf_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiTexture* colormap_tex_ = nullptr;
  QRhiSampler* colormap_sampler_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  /// Raw point records, kept so the GPU buffer can be rebuilt after a device loss
  /// without the caller having to re-supply the cloud.
  std::vector<std::uint8_t> points_;
  Layout layout_;
  int point_count_ = 0;
  int instance_capacity_ = 0;

  PJ::Colormap colormap_ = PJ::Colormap::kTurbo;
  float scalar_min_ = 0.0F;
  float scalar_max_ = 1.0F;
  float point_radius_ = 0.02F;

  bool colormap_uploaded_ = false;
  bool points_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
