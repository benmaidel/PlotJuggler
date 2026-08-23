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

bool RhiPresentPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd) {
  if (pipeline_ != nullptr && rhi_ == &rhi) {
    return true;
  }
  release();
  rhi_ = &rhi;

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
  sampler_ = rhi.newSampler(QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::None, QRhiSampler::ClampToEdge,
                            QRhiSampler::ClampToEdge);
  if (!sampler_->create()) {
    release();
    return false;
  }

  srb_ = rhi.newShaderResourceBindings();
  // Created empty and populated in prepare(): the source texture does not exist
  // until the HDR chain has been sized, which happens after initialize().
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
  pipeline_->setVertexInputLayout({});  // attributeless fullscreen triangle
  pipeline_->setShaderResourceBindings(srb_);
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
  if (pipeline_ == nullptr || source_ == nullptr) {
    return;
  }
  if (bindings_dirty_) {
    // Rebuild rather than mutate: QRhi reads an SRB at submit time, so editing one
    // between draws would make every draw see the last binding.
    srb_->destroy();
    srb_->setBindings({
        QRhiShaderResourceBinding::uniformBuffer(
            0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
        QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, source_, sampler_),
    });
    if (!srb_->create()) {
      return;
    }
    bindings_dirty_ = false;
  }

  PresentUbo ubo{};
  ubo.exposure = exposure_;
  ubo.flip_v = flip_v_ ? 1.0F : 0.0F;
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
  delete ubo_;
  ubo_ = nullptr;
  source_ = nullptr;
  bindings_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
