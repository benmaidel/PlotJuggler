// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"

#include <QLoggingCategory>
#include <QMouseEvent>
#include <QWheelEvent>
#include <algorithm>
#include <cmath>
#include <utility>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiView, "pj.scene3d.rhi.view")

// Background of the empty scene, as a DISPLAY-referred sRGB colour matching the GL
// renderer's light theme default.
constexpr float kClearR = 0.96F;
constexpr float kClearG = 0.96F;
constexpr float kClearB = 0.96F;

/// Approximate sRGB -> linear for the clear colour. The HDR target holds linear
/// light, so the background has to be linearized on the way IN and then bypass the
/// composite's grade on the way out; the two cancel and the theme colour survives
/// exactly. Clearing with the display value instead would brighten the background
/// by the encode.
float linearizeChannel(float c) {
  return std::pow(c, 2.2F);
}

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

void RhiSceneViewWidget::setSsaoEnabled(bool enabled) {
  if (ssao_enabled_ == enabled) {
    return;
  }
  ssao_enabled_ = enabled;
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
  // Order matters, and it is layered by depth behaviour. The reference grid and
  // the TF connection lines depth-test but do not depth-write, and the occupancy
  // map is a translucent ground overlay, so all three go down first. The meshes,
  // voxel cubes and point cloud then depth-write over them (the mesh pass runs its
  // own opaque-then-translucent split internally). The gizmos come last: pose
  // triads blend (they honour an opacity knob), so they must see the final depth
  // buffer to be occluded correctly.
  return {&grid_pass_,       &occupancy_pass_, &tf_connections_pass_, &mesh_pass_, &voxel_pass_,
          &pointcloud_pass_, &marker_pass_,    &axis_pass_,           &poses_pass_};
}

glm::mat4 RhiSceneViewWidget::buildScreenFromView(const QSize& pixel_size) const {
  if (camera_ == nullptr || pixel_size.height() <= 0) {
    return glm::mat4{1.0F};
  }
  const QRhi* r = rhi();
  if (r == nullptr) {
    return glm::mat4{1.0F};
  }
  const float aspect = static_cast<float>(pixel_size.width()) / static_cast<float>(pixel_size.height());
  const glm::mat4 correction = toGlm(r->clipSpaceCorrMatrix());
  const glm::mat4 clip_from_view = correction * camera_->projMatrix(aspect);

  // NDC -> screen. The two axes that vary by backend:
  //
  //  - z: the correction matrix is precisely what remaps OpenGL's [-1,1] NDC depth
  //    to the [0,1] the other backends use, so an IDENTITY correction means the GL
  //    convention is still in force and z needs the extra *0.5+0.5 here.
  //  - v: OpenGL's framebuffer row 0 is the bottom, everyone else's is the top,
  //    which is the same fact the present pass consumes as isYUpInFramebuffer().
  const bool ndc_z_is_minus_one_to_one = correction[2][2] == 1.0F && correction[3][2] == 0.0F;
  const float sz = ndc_z_is_minus_one_to_one ? 0.5F : 1.0F;
  const float bz = ndc_z_is_minus_one_to_one ? 0.5F : 0.0F;
  const float sy = r->isYUpInFramebuffer() ? 0.5F : -0.5F;

  glm::mat4 ndc_to_screen(1.0F);
  ndc_to_screen[0] = glm::vec4(0.5F, 0.0F, 0.0F, 0.0F);
  ndc_to_screen[1] = glm::vec4(0.0F, sy, 0.0F, 0.0F);
  ndc_to_screen[2] = glm::vec4(0.0F, 0.0F, sz, 0.0F);
  ndc_to_screen[3] = glm::vec4(0.5F, 0.5F, bz, 1.0F);
  return ndc_to_screen * clip_from_view;
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
    ssao_pass_.release();
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
  if (camera_ != nullptr) {
    ctx.camera_pos_world = camera_->position();
  }
  ctx.screen_from_view = buildScreenFromView(ctx.pixel_size);
  ctx.view_from_screen = glm::inverse(ctx.screen_from_view);

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
  // The composite's far-plane background bypass needs the resolved depth. It only
  // exists with MSAA plus QRhi::ResolveDepthStencil, so this may legitimately be
  // null; the pass then reports that the clear alpha must stand in for it.
  present_pass_.setDepthTexture(hdr_ready ? hdr_target_.resolvedDepth() : nullptr);

  // Every upload must be batched BEFORE a pass opens; QRhi forbids recording them
  // inside one. Both the scene passes and the present pass contribute here.
  QRhiResourceUpdateBatch* updates = r->nextResourceUpdateBatch();
  for (IRhiRenderPass* pass : passes()) {
    pass->prepare(*updates, ctx);
  }
  present_pass_.prepare(*updates, ctx);

  used_hdr_chain_ = hdr_ready && hdr_target_.renderTarget() != nullptr;

  // Two clears, because the two targets mean different things. The off-screen HDR
  // target is linear light, and its ALPHA is the composite's grade marker: 1 says
  // "grade this", which is right for the background only because the depth-based
  // bypass rescues it. With no resolved depth there is no such rescue, so the clear
  // drops to alpha 0 and the marker path bypasses the background instead. The
  // widget target is only ever fully covered by the fullscreen composite triangle,
  // so its clear colour is immaterial.
  const float clear_alpha = present_pass_.backgroundNeedsAlphaClear() ? 0.0F : 1.0F;
  const QColor scene_clear = used_hdr_chain_ ? QColor::fromRgbF(
                                                   linearizeChannel(kClearR), linearizeChannel(kClearG),
                                                   linearizeChannel(kClearB), clear_alpha)
                                             : QColor::fromRgbF(kClearR, kClearG, kClearB);
  const QColor clear = QColor::fromRgbF(kClearR, kClearG, kClearB);

  if (used_hdr_chain_) {
    // Scene into the off-screen multisample chain. Resolve to single-sample
    // happens implicitly at endPass, driven by the attachment's resolveTexture.
    cb->beginPass(hdr_target_.renderTarget(), scene_clear, {1.0F, 0}, updates);
    cb->setViewport(
        {0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()), static_cast<float>(ctx.pixel_size.height())});
    for (IRhiRenderPass* pass : passes()) {
      pass->draw(*cb, ctx);
    }
    cb->endPass();

    // Screen-space occlusion, between the scene and the composite because it reads
    // the RESOLVED depth (which only exists once the scene pass has ended) and
    // writes its own off-screen targets. It needs that resolved depth, so it stays
    // off when the chain could not produce one.
    QRhiTexture* ao = nullptr;
    if (ssao_enabled_ && hdr_target_.resolvedDepth() != nullptr) {
      ssao_pass_.setDepthTexture(hdr_target_.resolvedDepth());
      if (ssao_pass_.ensure(*r, ctx.pixel_size)) {
        ssao_pass_.render(*cb, ctx);
        ao = ssao_pass_.output();
      }
    }
    present_pass_.setAoTexture(ao);

    // Composite onto the widget's own target. The clear colour is irrelevant here
    // because the fullscreen triangle covers every pixel.
    cb->beginPass(widget_rt, clear, {1.0F, 0});
    cb->setViewport(
        {0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()), static_cast<float>(ctx.pixel_size.height())});
    present_pass_.draw(*cb, ctx);
    cb->endPass();
  } else {
    // Fallback: straight into the widget target. Loses MSAA and the HDR buffer but
    // still shows the scene, which beats a blank dock.
    cb->beginPass(widget_rt, scene_clear, {1.0F, 0}, updates);
    cb->setViewport(
        {0.0F, 0.0F, static_cast<float>(ctx.pixel_size.width()), static_cast<float>(ctx.pixel_size.height())});
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
  ssao_pass_.release();
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
