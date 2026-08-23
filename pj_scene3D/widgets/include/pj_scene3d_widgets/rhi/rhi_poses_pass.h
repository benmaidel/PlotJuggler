#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_core/poses_in_frame_render.h"  // PoseTriadInstance
#include "pj_scene3d_widgets/rhi/rhi_arrow_shading.h"
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Pose-array gizmos: the arms produced by buildPoseTriadInstances(), drawn as one
/// instanced arrow batch.
///
/// The QRhi counterpart of PosesRenderPass, and it shares shaders/arrow.{vert,frag}
/// with RhiAxisPass so pose gizmos and TF "Frames" gizmos read identically — the
/// GL renderer states that as an intentional property, and one shared shader pair
/// enforces it rather than trusting two copies to stay in step.
///
/// What makes this pass distinct from RhiAxisPass is the *frame-local* instance
/// convention: setInstances() takes arms expressed in their source frame, and the
/// resolved fixed_frame<-source_frame transform goes in separately via
/// setFrameWorld(). TF motion, playback scrubbing and camera motion therefore
/// change one uniform, never the instance buffer — which is what keeps a
/// thousands-of-poses particle cloud cheap to animate.
class RhiPosesPass final : public IRhiRenderPass {
 public:
  RhiPosesPass() = default;
  ~RhiPosesPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Stage the arms to draw, in their source frame's coordinates. CPU-only, so it
  /// is safe to call from a decode thread; the GPU buffer is refreshed on the next
  /// prepare().
  void setInstances(std::vector<PoseTriadInstance> instances);

  /// The fixed_frame <- source_frame transform the caller resolved from TF. Cheap:
  /// it only rewrites the uniform block, leaving the instance buffer untouched.
  void setFrameWorld(const glm::mat4& frame_world);

  /// Number of staged arms (poses * arms-per-pose), for tests and diagnostics.
  [[nodiscard]] std::size_t instanceCount() const {
    return instances_.size();
  }

 private:
  /// The staged arms are handed straight to the GPU with no repacking, so their
  /// CPU layout must already be the per-instance vertex layout.
  static_assert(
      sizeof(PoseTriadInstance) == sizeof(arrow::Instance),
      "PoseTriadInstance must match the arrow per-instance stride to upload verbatim");
  static_assert(
      offsetof(PoseTriadInstance, model) == offsetof(arrow::Instance, model),
      "PoseTriadInstance::model must sit where arrow.vert expects the model columns");
  static_assert(
      offsetof(PoseTriadInstance, color) == offsetof(arrow::Instance, color),
      "PoseTriadInstance::color must sit where arrow.vert expects the colour");

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* vbo_ = nullptr;
  QRhiBuffer* ibo_ = nullptr;
  QRhiBuffer* instance_buf_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  std::vector<PoseTriadInstance> instances_;
  glm::mat4 frame_world_{1.0F};

  int index_count_ = 0;
  /// Instance capacity currently allocated, in instances. Grown geometrically so a
  /// streaming pose array does not re-create the buffer every sample.
  int instance_capacity_ = 0;
  bool mesh_uploaded_ = false;
  bool instances_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
