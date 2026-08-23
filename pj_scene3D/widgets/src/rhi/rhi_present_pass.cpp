// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_present_pass.h"

#include <QFile>
#include <QLoggingCategory>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiPresent, "pj.scene3d.rhi.present")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiPresent) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiPresentPass::~RhiPresentPass() {
  release();
}

void RhiPresentPass::setSourceTexture(QRhiTexture* texture) {
  if (source_ == texture) {
    return;
  }
  source_ = texture;
  bindings_dirty_ = true;
}

void RhiPresentPass::setDepthTexture(QRhiTexture* texture) {
  if (depth_ == texture) {
    return;
  }
  depth_ = texture;
  bindings_dirty_ = true;
}

void RhiPresentPass::setAoTexture(QRhiTexture* texture) {
  if (ao_ == texture) {
    return;
  }
  ao_ = texture;
  bindings_dirty_ = true;
}

void RhiPresentPass::setCompositeParams(const CompositeParams& params) {
  params_ = params;
}

bool RhiPresentPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/present.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/present.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(PresentUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }

  // Linear filtering so a supersampled chain (render scale > 1) box-averages on
  // the way down; ClampToEdge because the fullscreen triangle never samples
  // outside [0,1].
  sampler_ = rhi.newSampler(
      QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None, QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge);
  if (!sampler_->create()) {
    release();
    return false;
  }
  // Depth is sampled NEAREST: filtering it would interpolate across a silhouette
  // and smear the far-plane comparison into a halo around every object edge.
  depth_sampler_ = rhi.newSampler(
      QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge,
      QRhiSampler::ClampToEdge);
  if (depth_sampler_ == nullptr || !depth_sampler_->create()) {
    release();
    return false;
  }

  // A 1x1 placeholder so the SRB's LAYOUT is final before the pipeline is
  // created. This is not cosmetic: a pipeline is compiled against the resource
  // layout of the SRB it is created with, so an SRB that gains bindings later is
  // not layout-compatible and the draw reads garbage — which is exactly how the
  // point-cloud pass first failed. Swapping one texture for another at the same
  // binding later IS compatible, which is what setSourceTexture relies on.
  placeholder_tex_ = rhi.newTexture(QRhiTexture::RGBA16F, QSize(1, 1));
  if (placeholder_tex_ == nullptr || !placeholder_tex_->create()) {
    release();
    return false;
  }
  // A colour stand-in also serves the depth slot: the binding only has to be
  // layout-compatible, and when no real depth is bound the shader's has_depth flag
  // keeps it from ever being sampled.
  depth_placeholder_tex_ = rhi.newTexture(QRhiTexture::R16F, QSize(1, 1));
  if (depth_placeholder_tex_ == nullptr || !depth_placeholder_tex_->create()) {
    release();
    return false;
  }
  ao_placeholder_tex_ = rhi.newTexture(QRhiTexture::R16F, QSize(1, 1));
  if (ao_placeholder_tex_ == nullptr || !ao_placeholder_tex_->create()) {
    release();
    return false;
  }

  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      QRhiShaderResourceBinding::sampledTexture(
          1, QRhiShaderResourceBinding::FragmentStage, placeholder_tex_, sampler_),
      QRhiShaderResourceBinding::sampledTexture(
          2, QRhiShaderResourceBinding::FragmentStage, depth_placeholder_tex_, depth_sampler_),
      QRhiShaderResourceBinding::sampledTexture(
          3, QRhiShaderResourceBinding::FragmentStage, ao_placeholder_tex_, sampler_),
  });
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
  pipeline_->setVertexInputLayout({});  // attributeless fullscreen triangle
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  // A full-coverage copy: no depth interaction and no blending.
  pipeline_->setDepthTest(false);
  pipeline_->setDepthWrite(false);
  pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  if (!pipeline_->create()) {
    qCWarning(lcRhiPresent) << "present pipeline creation failed";
    release();
    return false;
  }
  // Sampling orientation is a property of the backend, fixed for this device.
  flip_v_ = !rhi.isYUpInFramebuffer();
  bindings_dirty_ = true;
  return true;
}

void RhiPresentPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr) {
    return;
  }
  if (bindings_dirty_ && source_ != nullptr) {
    // Rebuild rather than mutate: QRhi reads an SRB at submit time, so editing one
    // between draws would make every draw see the last binding.
    srb_->destroy();
    // Same bindings, different texture: layout-compatible, so the pipeline stays
    // valid.
    srb_->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(
            0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, source_, sampler_),
        QRhiShaderResourceBinding::sampledTexture(
            2, QRhiShaderResourceBinding::FragmentStage, depth_ != nullptr ? depth_ : depth_placeholder_tex_,
            depth_sampler_),
        QRhiShaderResourceBinding::sampledTexture(
            3, QRhiShaderResourceBinding::FragmentStage, ao_ != nullptr ? ao_ : ao_placeholder_tex_, sampler_),
    });
    if (!srb_->create()) {
      return;
    }
    bindings_dirty_ = false;
  }

  PresentUbo ubo{};
  ubo.exposure = params_.exposure;
  ubo.flip_v = flip_v_ ? 1.0F : 0.0F;
  ubo.tonemap_mode = params_.tonemap_mode;
  ubo.saturation = params_.saturation;
  ubo.has_depth = depth_ != nullptr ? 1.0F : 0.0F;
  ubo.has_ao = ao_ != nullptr ? 1.0F : 0.0F;
  ubo.ao_strength = params_.ao_strength;
  updates.updateDynamicBuffer(ubo_, 0, sizeof(PresentUbo), &ubo);
}

void RhiPresentPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || source_ == nullptr || bindings_dirty_) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  cb.draw(3);
}

void RhiPresentPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete sampler_;
  sampler_ = nullptr;
  delete placeholder_tex_;
  placeholder_tex_ = nullptr;
  delete depth_placeholder_tex_;
  depth_placeholder_tex_ = nullptr;
  delete ao_placeholder_tex_;
  ao_placeholder_tex_ = nullptr;
  delete depth_sampler_;
  depth_sampler_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  source_ = nullptr;
  bindings_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
