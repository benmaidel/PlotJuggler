#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <QSize>
#include <glm/glm.hpp>
#include <rhi/qrhi.h>

namespace pj::scene3d::rhi {

/// Per-frame state shared by every pass. Deliberately plain data: the passes stay
/// unaware of the widget, the camera model and the datastore, which is what lets
/// them be unit-tested against an offscreen QRhi.
struct RhiFrameContext {
  /// Combined projection * view, already corrected for the backend's clip space
  /// (see QRhi::clipSpaceCorrMatrix) — passes must NOT re-apply that correction.
  glm::mat4 view_proj{1.0F};
  /// Render target size in device pixels; needed by passes that size things in
  /// screen space (point sprites, screen-space line widths, post effects).
  QSize pixel_size;
};

/// One drawable stage of the QRhi scene renderer.
///
/// Split into prepare/draw rather than a single render() because QRhi wants all
/// buffer and texture uploads batched into the QRhiResourceUpdateBatch handed to
/// beginPass(); recording an upload once the pass has begun is not allowed. So
/// the widget collects every pass's uploads first, opens the render pass with
/// them, then asks each pass to record its draws.
class IRhiRenderPass {
 public:
  virtual ~IRhiRenderPass() = default;

  IRhiRenderPass(const IRhiRenderPass&) = delete;
  IRhiRenderPass& operator=(const IRhiRenderPass&) = delete;

  /// Create pipelines/buffers against `rhi`, compatible with `rpd` and with
  /// `sample_count` (the MSAA level of the target this pass draws into).
  ///
  /// `sample_count` is NOT optional: QRhi requires a graphics pipeline's sample
  /// count to match its render target's, and a mismatch is not reported as an
  /// error — it silently writes a fraction of the samples, which reads as a
  /// uniformly washed-out, semi-transparent draw.
  ///
  /// Must be idempotent and safe to call again after a QRhi swap (widget
  /// reparent, screen change): the previous device's objects are already gone via
  /// release(), so an implementation re-creates from scratch. Returns false if the
  /// pass is unusable (e.g. its shader pack has no variant for this backend), in
  /// which case the widget skips it instead of failing the whole frame.
  [[nodiscard]] virtual bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) = 0;

  /// Queue this frame's uploads (uniform blocks, vertex data). Called before
  /// beginPass, never inside it.
  virtual void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) = 0;

  /// Record draw calls into an already-begun render pass.
  virtual void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) = 0;

  /// Destroy every GPU object. Called on QRhi loss and from the widget's
  /// destructor; must tolerate being called twice and before initialize().
  virtual void release() = 0;

 protected:
  IRhiRenderPass() = default;
};

}  // namespace pj::scene3d::rhi
