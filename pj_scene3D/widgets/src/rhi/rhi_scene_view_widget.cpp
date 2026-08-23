// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"

#include <QLoggingCategory>
#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>
#include <utility>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiView, "pj.scene3d.rhi.view")

// Background of the empty scene. Matches the GL renderer's light theme default so
// a screenshot can be compared against the existing reference render.
constexpr float kClearR = 0.96F;
constexpr float kClearG = 0.96F;
constexpr float kClearB = 0.96F;

/// Convert a QMatrix4x4 (Qt, column-major storage, row-major constructor) into a
/// glm::mat4 so the correction can be composed with glm's camera matrices.
glm::mat4 toGlm(const QMatrix4x4& m) {
  glm::mat4 out{1.0F};
  const float* src = m.constData();  // column-major, 16 floats
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[col][row] = src[(col * 4) + row];
    }
  }
  return out;
}

}  // namespace

RhiSceneViewWidget::RhiSceneViewWidget(QWidget* parent) : QRhiWidget(parent) {
  // Metal on macOS: Apple's OpenGL is frozen at 4.1 and deprecated, and is the
  // reason this renderer exists. Elsewhere OpenGL remains the tested path.
#if defined(Q_OS_MACOS)
  setApi(Api::Metal);
#else
  setApi(Api::OpenGL);
#endif
  setFocusPolicy(Qt::StrongFocus);
  setMouseTracking(true);
  camera_ = std::make_unique<OrbitCamera>();
}

RhiSceneViewWidget::~RhiSceneViewWidget() {
  // Qt does not call releaseResources() on destruction — only on a QRhi change —
  // so without this every GPU object leaks when a dock destroys the view.
  releaseResources();
}

void RhiSceneViewWidget::setCamera(std::unique_ptr<ICamera> camera) {
  if (camera == nullptr) {
    return;
  }
  if (camera_ != nullptr) {
    camera->adoptState(camera_->state());
  }
  camera_ = std::move(camera);
  update();
}

void RhiSceneViewWidget::setSceneSamples(int samples) {
  if (samples == desired_samples_) {
    return;
  }
  desired_samples_ = samples;
  // The chain is re-sized lazily in render(); dropping it now would need the QRhi
  // to still be current, which it may not be from an arbitrary caller.
  update();
}

std::vector<IRhiRenderPass*> RhiSceneViewWidget::passes() {
  // Grid first: it does not write depth, so drawing it before the opaque triads
  // lets them occlude it correctly rather than the reverse.
  // Grid and connection lines first (neither writes depth), then the opaque
  // cloud and triads so they occlude the annotations correctly.
  // Order matters. The occupancy map is a translucent ground overlay, so it is
  // blended over the reference grid before any opaque geometry; the cloud and
  // triads then depth-write over both.
  return {&grid_pass_, &occupancy_pass_, &tf_connections_pass_, &pointcloud_pass_, &axis_pass_};
}

glm::mat4 RhiSceneViewWidget::buildViewProj(const QSize& pixel_size) const {
  if (camera_ == nullptr || pixel_size.height() <= 0) {
    return glm::mat4{1.0F};
  }
  const float aspect = static_cast<float>(pixel_size.width()) / static_cast<float>(pixel_size.height());
  const glm::mat4 proj = camera_->projMatrix(aspect);
  const glm::mat4 view = camera_->viewMatrix();
  const QRhi* r = rhi();
  const glm::mat4 correction = r != nullptr ? toGlm(r->clipSpaceCorrMatrix()) : glm::mat4{1.0F};
  return correction * proj * view;
}

void RhiSceneViewWidget::initialize(QRhiCommandBuffer* /*cb*/) {
  QRhi* r = rhi();
  if (r == nullptr) {
    return;
  }
  // A swapped QRhi invalidated every GPU object we own.
  if (rhi_cached_ != r) {
    for (IRhiRenderPass* pass : passes()) {
      pass->release();
    }
    present_pass_.release();
    hdr_target_.release();
    scene_rpd_ = nullptr;
    rhi_cached_ = r;
  }

  QRhiRenderTarget* rt = renderTarget();
  if (rt == nullptr || rt->renderPassDescriptor() == nullptr) {
    return;
  }
  // Only the present pass can be built here: it targets the WIDGET's render pass.
  // The geometry passes target the off-screen HDR chain, whose descriptor does not
  // exist until render() has sized it, so they are initialized lazily there.
  // The present pass draws into the WIDGET target, whose sample count is the
  // widget's own (1 unless setSampleCount was called) — not the HDR chain's.
  if (!present_pass_.initialize(*r, *rt->renderPassDescriptor(), std::max(1, sampleCount()))) {
    qCWarning(lcRhiView) << "present pass unavailable; falling back to direct-to-widget rendering";
  }
}

void RhiSceneViewWidget::render(QRhiCommandBuffer* cb) {
  QRhi* r = rhi();
  QRhiRenderTarget* widget_rt = renderTarget();
  if (r == nullptr || cb == nullptr || widget_rt == nullptr) {
    return;
  }

  RhiFrameContext ctx;
  ctx.pixel_size = widget_rt->pixelSize();
  ctx.view_proj = buildViewProj(ctx.pixel_size);

  const bool hdr_ready = hdr_target_.ensure(*r, ctx.pixel_size, desired_samples_);
  // The geometry pipelines are only valid for the descriptor they were built
  // against, and the HDR chain makes a fresh one whenever it is re-created.
  if (hdr_ready && hdr_target_.renderPassDescriptor() != scene_rpd_) {
    for (IRhiRenderPass* pass : passes()) {
      pass->release();
      if (!pass->initialize(*r, *hdr_target_.renderPassDescriptor(), hdr_target_.sampleCount())) {
        qCWarning(lcRhiView) << "a render pass failed to initialize and will be skipped";
      }
    }
    scene_rpd_ = hdr_target_.renderPassDescriptor();
  }
  present_pass_.setSourceTexture(hdr_ready ? hdr_target_.resolvedColor() : nullptr);

  // Every upload must be batched BEFORE a pass opens; QRhi forbids recording them
  // inside one. Both the scene passes and the present pass contribute here.
  QRhiResourceUpdateBatch* updates = r->nextResourceUpdateBatch();
  for (IRhiRenderPass* pass : passes()) {
    pass->prepare(*updates, ctx);
  }
  present_pass_.prepare(*updates, ctx);

  const QColor clear = QColor::fromRgbF(kClearR, kClearG, kClearB);
  used_hdr_chain_ = hdr_ready && hdr_target_.renderTarget() != nullptr;

  if (used_hdr_chain_) {
    // Scene into the off-screen multisample chain. Resolve to single-sample
    // happens implicitly at endPass, driven by the attachment's resolveTexture.
    cb->beginPass(hdr_target_.renderTarget(), clear, {1.0F, 0}, updates);
    cb->setViewport({0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()),
                     static_cast<float>(ctx.pixel_size.height())});
    for (IRhiRenderPass* pass : passes()) {
      pass->draw(*cb, ctx);
    }
    cb->endPass();

    // Composite onto the widget's own target. The clear colour is irrelevant here
    // because the fullscreen triangle covers every pixel.
    cb->beginPass(widget_rt, clear, {1.0F, 0});
    cb->setViewport({0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()),
                     static_cast<float>(ctx.pixel_size.height())});
    present_pass_.draw(*cb, ctx);
    cb->endPass();
  } else {
    // Fallback: straight into the widget target. Loses MSAA and the HDR buffer but
    // still shows the scene, which beats a blank dock.
    cb->beginPass(widget_rt, clear, {1.0F, 0}, updates);
    cb->setViewport({0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()),
                     static_cast<float>(ctx.pixel_size.height())});
    for (IRhiRenderPass* pass : passes()) {
      pass->draw(*cb, ctx);
    }
    cb->endPass();
  }
  has_rendered_ = true;
}

void RhiSceneViewWidget::releaseResources() {
  for (IRhiRenderPass* pass : passes()) {
    pass->release();
  }
  present_pass_.release();
  hdr_target_.release();
  scene_rpd_ = nullptr;
  rhi_cached_ = nullptr;
}

void RhiSceneViewWidget::mousePressEvent(QMouseEvent* event) {
  last_cursor_ = event->pos();
  QRhiWidget::mousePressEvent(event);
}

void RhiSceneViewWidget::mouseMoveEvent(QMouseEvent* event) {
  if (camera_ != nullptr && (event->buttons() & Qt::LeftButton) != 0) {
    const QPoint delta = event->pos() - last_cursor_;
    camera_->rotate(static_cast<float>(delta.x()), static_cast<float>(delta.y()));
    update();
  }
  last_cursor_ = event->pos();
  QRhiWidget::mouseMoveEvent(event);
}

void RhiSceneViewWidget::wheelEvent(QWheelEvent* event) {
  if (camera_ != nullptr) {
    camera_->zoom(static_cast<float>(event->angleDelta().y()) / 120.0F);
    update();
  }
  event->accept();
}

}  // namespace pj::scene3d::rhi
