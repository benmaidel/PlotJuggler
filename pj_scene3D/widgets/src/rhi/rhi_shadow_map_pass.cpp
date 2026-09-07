// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_shadow_map_pass.h"

#include <QLoggingCategory>

#include "pj_scene3d_core/shadow_camera.h"  // kShadowMapSize

namespace pj::scene3d::rhi {

namespace {
Q_LOGGING_CATEGORY(lcRhiShadow, "pj.scene3d.rhi.shadow")
}  // namespace

RhiShadowMapPass::~RhiShadowMapPass() {
  release();
}

bool RhiShadowMapPass::ensure(QRhi& rhi) {
  if (render_target_ != nullptr && rhi_ == &rhi) {
    return true;
  }
  if (rhi_ != &rhi) {
    release();
  }
  rhi_ = &rhi;

  // RenderTarget so it can be written, and sampled (the default) so the receiver can
  // read it. D32F matches the OpenGL map's GL_DEPTH_COMPONENT32F.
  depth_ = rhi.newTexture(QRhiTexture::D32F, QSize(kShadowMapSize, kShadowMapSize), 1, QRhiTexture::RenderTarget);
  if (depth_ == nullptr || !depth_->create()) {
    qCWarning(lcRhiShadow) << "shadow depth texture" << kShadowMapSize << "could not be created";
    release();
    return false;
  }

  // Depth-ONLY: no colour attachment. The description's colour list stays empty and
  // the depth texture goes in depthTexture(), which is what makes this cheap.
  QRhiTextureRenderTargetDescription desc;
  desc.setDepthTexture(depth_);
  render_target_ = rhi.newTextureRenderTarget(desc);
  if (render_target_ == nullptr) {
    release();
    return false;
  }
  rpd_ = render_target_->newCompatibleRenderPassDescriptor();
  render_target_->setRenderPassDescriptor(rpd_);
  if (!render_target_->create()) {
    qCWarning(lcRhiShadow) << "shadow render target could not be created";
    release();
    return false;
  }
  qCDebug(lcRhiShadow) << "shadow map" << kShadowMapSize << "x" << kShadowMapSize << "D32F";
  return true;
}

void RhiShadowMapPass::release() {
  delete render_target_;
  render_target_ = nullptr;
  delete rpd_;
  rpd_ = nullptr;
  delete depth_;
  depth_ = nullptr;
  rhi_ = nullptr;
}

void RhiShadowMapPass::begin(QRhiCommandBuffer& cb, QRhiResourceUpdateBatch* updates) {
  // Clear to the far plane so an un-drawn texel never shadows: the receiver's
  // comparison is `light_depth <= sampled`, which is true everywhere at 1.0.
  cb.beginPass(render_target_, {}, {1.0F, 0}, updates);
  cb.setViewport({0.0F, 0.0F, static_cast<float>(kShadowMapSize), static_cast<float>(kShadowMapSize)});
}

void RhiShadowMapPass::end(QRhiCommandBuffer& cb) {
  cb.endPass();
}

}  // namespace pj::scene3d::rhi
