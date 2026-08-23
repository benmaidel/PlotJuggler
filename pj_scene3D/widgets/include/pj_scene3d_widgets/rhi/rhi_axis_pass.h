#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_widgets/rhi/rhi_arrow_shading.h"
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Coordinate-frame triads: one solid arrow per axis (X red, Y green, Z blue) for
/// every frame handed to setFrames().
///
/// Second pass ported to QRhi, chosen because it exercises INSTANCING with
/// per-instance transforms — the shape most of the remaining passes need
/// (poses, markers, voxel grids all draw one mesh many times). One indexed arrow
/// mesh is uploaded once and drawn `3 * frames` times from a per-instance buffer.
///
/// Shares shaders/arrow.{vert,frag} with RhiPosesPass so TF gizmos and PoseArray
/// gizmos cannot drift apart visually. The frames handed to setFrames() are
/// already world-space, so this pass writes identity into the shader's
/// `frame_world`.
class RhiAxisPass final : public IRhiRenderPass {
 public:
  RhiAxisPass() = default;
  ~RhiAxisPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// World transforms of the frames to draw. Each yields three arrows; the
  /// instance buffer is rebuilt on the next prepare().
  void setFrames(std::vector<glm::mat4> frame_transforms);

  /// Arrow length in metres (the triad's visual size).
  void setAxisLength(float metres);

 private:
  using Instance = arrow::Instance;

  void rebuildInstances();

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* vbo_ = nullptr;
  QRhiBuffer* ibo_ = nullptr;
  QRhiBuffer* instance_buf_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  std::vector<glm::mat4> frames_;
  std::vector<Instance> instances_;
  float axis_length_ = 0.5F;

  int index_count_ = 0;
  /// Instance capacity currently allocated, in instances. The buffer grows
  /// geometrically so a streaming TF tree does not re-create it every frame.
  int instance_capacity_ = 0;
  bool mesh_uploaded_ = false;
  bool instances_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
