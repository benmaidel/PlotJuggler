// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_hdr_target.h"

#include <QLoggingCategory>
#include <algorithm>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiHdr, "pj.scene3d.rhi.hdr")

/// Highest supported sample count at or below `desired`. QRhi only promises the
/// counts it enumerates, and asking for an unsupported one fails target creation.
int clampSamples(const QRhi& rhi, int desired) {
  int best = 1;
  for (const int candidate : rhi.supportedSampleCounts()) {
    if (candidate <= desired && candidate > best) {
      best = candidate;
    }
  }
  return best;
}

}  // namespace

RhiHdrTarget::~RhiHdrTarget() {
  release();
}

bool RhiHdrTarget::ensure(QRhi& rhi, const QSize& pixel_size, int desired_samples) {
  if (pixel_size.isEmpty()) {
    return false;
  }
  const int samples = clampSamples(rhi, std::max(1, desired_samples));
  if (rhi_ == &rhi && render_target_ != nullptr && size_ == pixel_size && samples_ == samples) {
    return true;
  }

  release();
  rhi_ = &rhi;
  size_ = pixel_size;
  samples_ = samples;

  // Linear-light HDR colour. RGBA16F is what lets exposure/tonemapping happen at
  // composite time rather than clipping inside each geometry pass.
  msaa_color_ = rhi.newTexture(QRhiTexture::RGBA16F, pixel_size, samples_, QRhiTexture::RenderTarget);
  if (msaa_color_ == nullptr || !msaa_color_->create()) {
    qCWarning(lcRhiHdr) << "MSAA colour texture creation failed at" << pixel_size << "samples" << samples_;
    release();
    return false;
  }

  msaa_depth_ = rhi.newRenderBuffer(QRhiRenderBuffer::DepthStencil, pixel_size, samples_);
  if (msaa_depth_ == nullptr || !msaa_depth_->create()) {
    qCWarning(lcRhiHdr) << "MSAA depth buffer creation failed";
    release();
    return false;
  }

  // Resolve destination: single-sample, sampled by the present pass. Needs the
  // RenderTarget flag because a resolve writes into it.
  resolve_color_ = rhi.newTexture(QRhiTexture::RGBA16F, pixel_size, 1, QRhiTexture::RenderTarget);
  if (resolve_color_ == nullptr || !resolve_color_->create()) {
    qCWarning(lcRhiHdr) << "resolve colour texture creation failed";
    release();
    return false;
  }

  QRhiColorAttachment color(msaa_color_);
  color.setResolveTexture(resolve_color_);

  QRhiTextureRenderTargetDescription desc;
  desc.setColorAttachments({color});
  desc.setDepthStencilBuffer(msaa_depth_);

  // Depth resolve is what SSAO/EDL will sample. Only request it where the backend
  // supports it; without it the chain still renders, the screen-space passes just
  // have no depth source yet.
  if (samples_ > 1 && rhi.isFeatureSupported(QRhi::ResolveDepthStencil)) {
    resolve_depth_ = rhi.newTexture(QRhiTexture::D32F, pixel_size, 1, QRhiTexture::RenderTarget);
    if (resolve_depth_ != nullptr && resolve_depth_->create()) {
      desc.setDepthResolveTexture(resolve_depth_);
    } else {
      delete resolve_depth_;
      resolve_depth_ = nullptr;
      qCWarning(lcRhiHdr) << "depth resolve texture creation failed; screen-space passes will have no depth";
    }
  }

  render_target_ = rhi.newTextureRenderTarget(desc);
  if (render_target_ == nullptr) {
    release();
    return false;
  }
  rpd_ = render_target_->newCompatibleRenderPassDescriptor();
  if (rpd_ == nullptr) {
    release();
    return false;
  }
  render_target_->setRenderPassDescriptor(rpd_);
  if (!render_target_->create()) {
    qCWarning(lcRhiHdr) << "HDR render target creation failed";
    release();
    return false;
  }

  qCDebug(lcRhiHdr) << "HDR chain" << pixel_size << "samples" << samples_ << "depth-resolve"
                    << (resolve_depth_ != nullptr);
  return true;
}

void RhiHdrTarget::release() {
  delete render_target_;
  render_target_ = nullptr;
  delete rpd_;
  rpd_ = nullptr;
  delete resolve_depth_;
  resolve_depth_ = nullptr;
  delete resolve_color_;
  resolve_color_ = nullptr;
  delete msaa_depth_;
  msaa_depth_ = nullptr;
  delete msaa_color_;
  msaa_color_ = nullptr;
  size_ = QSize();
  samples_ = 1;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
