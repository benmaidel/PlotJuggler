#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Ground reference grid on the z=0 plane, drawn as lines.
///
/// The first pass ported to QRhi, and therefore the reference implementation for
/// the rest: baked shader pack, one std140 uniform block, an Immutable vertex
/// buffer uploaded once, and no global GPU state (QRhi has none to leak).
///
/// The line geometry itself comes from the existing Qt/GL-free
/// passes/grid_geometry.h, so nothing about the grid's shape is duplicated here.
class RhiGridPass final : public IRhiRenderPass {
 public:
  RhiGridPass() = default;
  ~RhiGridPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Grid extent in metres (full side length, centred on the origin) and the
  /// number of cells per axis. Changing either re-tessellates on the next
  /// prepare(); it does not touch the pipeline.
  void setGeometry(float extent_m, int divisions);
  void setLineColor(const glm::vec4& rgba) { line_color_ = rgba; }

 private:
  /// Mirrors the LinesUbo block in shaders/lines.{vert,frag} (shared with the TF
  /// connections pass). std140 puts the mat4 at offset 0 and the vec4 at 64; the
  /// total is already a multiple of 16, so no tail padding is needed. Keep this in
  /// lockstep with the shader.
  struct alignas(16) LinesUbo {
    float view_proj[16];
    float line_color[4];
  };
  static_assert(sizeof(LinesUbo) == 80, "LinesUbo must match the std140 block layout");

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* vbo_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  float extent_m_ = 20.0F;
  int divisions_ = 20;
  glm::vec4 line_color_{0.62F, 0.62F, 0.64F, 1.0F};

  /// Vertex count currently resident in vbo_; 0 until the first upload.
  int vertex_count_ = 0;
  /// Set when the geometry must be re-tessellated and re-uploaded.
  bool geometry_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
