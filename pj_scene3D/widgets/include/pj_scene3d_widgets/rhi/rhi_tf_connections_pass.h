#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Line segments connecting each TF frame to its parent, so the frame tree's
/// structure is visible rather than having to be inferred from floating triads.
///
/// Shares shaders/lines.{vert,frag} with the grid: both are world-space coloured
/// line segments through view_proj, and the shader consumes only the position, so
/// the two can use different vertex strides against one pipeline layout.
class RhiTfConnectionsPass final : public IRhiRenderPass {
 public:
  RhiTfConnectionsPass() = default;
  ~RhiTfConnectionsPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Endpoints as consecutive pairs: [a0, b0, a1, b1, ...]. An odd trailing
  /// element is ignored. Empty clears the pass.
  void setSegments(std::vector<glm::vec3> endpoints);
  void setColor(const glm::vec4& rgba) { color_ = rgba; }

  [[nodiscard]] int segmentCount() const { return vertex_count_ / 2; }

 private:
  /// Mirrors the LinesUbo block in shaders/lines.{vert,frag}.
  struct alignas(16) LinesUbo {
    float view_proj[16];
    float color[4];
  };
  static_assert(sizeof(LinesUbo) == 80, "LinesUbo must match the std140 block layout");

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* vbo_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  std::vector<glm::vec3> endpoints_;
  /// Magenta by default, matching the OpenGL renderer's connection lines.
  glm::vec4 color_{0.85F, 0.30F, 0.85F, 1.0F};

  int vertex_count_ = 0;
  int vertex_capacity_ = 0;
  bool geometry_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
