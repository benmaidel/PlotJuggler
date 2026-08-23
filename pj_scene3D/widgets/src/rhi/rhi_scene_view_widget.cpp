// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"

#include <QLoggingCategory>
#include <QMouseEvent>
#include <QWheelEvent>
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

std::vector<IRhiRenderPass*> RhiSceneViewWidget::passes() {
  return {&grid_pass_};
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
  // A swapped QRhi invalidated every pass's objects; rebuild against the new one.
  if (rhi_cached_ != r) {
    for (IRhiRenderPass* pass : passes()) {
      pass->release();
    }
    rhi_cached_ = r;
  }

  QRhiRenderTarget* rt = renderTarget();
  if (rt == nullptr || rt->renderPassDescriptor() == nullptr) {
    return;
  }
  for (IRhiRenderPass* pass : passes()) {
    if (!pass->initialize(*r, *rt->renderPassDescriptor())) {
      qCWarning(lcRhiView) << "a render pass failed to initialize and will be skipped this session";
    }
  }
}

void RhiSceneViewWidget::render(QRhiCommandBuffer* cb) {
  QRhi* r = rhi();
  QRhiRenderTarget* rt = renderTarget();
  if (r == nullptr || cb == nullptr || rt == nullptr) {
    return;
  }

  RhiFrameContext ctx;
  ctx.pixel_size = rt->pixelSize();
  ctx.view_proj = buildViewProj(ctx.pixel_size);

  // All uploads must be batched BEFORE the pass opens; QRhi forbids recording
  // them once beginPass has run.
  QRhiResourceUpdateBatch* updates = r->nextResourceUpdateBatch();
  for (IRhiRenderPass* pass : passes()) {
    pass->prepare(*updates, ctx);
  }

  cb->beginPass(rt, QColor::fromRgbF(kClearR, kClearG, kClearB), {1.0F, 0}, updates);
  cb->setViewport({0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()),
                   static_cast<float>(ctx.pixel_size.height())});
  for (IRhiRenderPass* pass : passes()) {
    pass->draw(*cb, ctx);
  }
  cb->endPass();
  has_rendered_ = true;
}

void RhiSceneViewWidget::releaseResources() {
  for (IRhiRenderPass* pass : passes()) {
    pass->release();
  }
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
