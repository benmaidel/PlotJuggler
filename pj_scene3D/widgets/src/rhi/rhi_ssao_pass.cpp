// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_ssao_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <cmath>
#include <cstring>
#include <random>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiSsao, "pj.scene3d.rhi.ssao")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiSsao) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiSsaoPass::RhiSsaoPass() {
  // Cosine-ish hemisphere kernel around +Z, weighted toward the origin so nearby
  // geometry dominates the estimate. Fixed seed: a kernel that changed between runs
  // would make the AO field irreproducible and defeat screenshot comparison.
  std::mt19937 rng(1337U);
  std::uniform_real_distribution<float> unit(0.0F, 1.0F);
  std::uniform_real_distribution<float> signed_unit(-1.0F, 1.0F);
  for (int i = 0; i < kKernelSize; ++i) {
    glm::vec3 sample(signed_unit(rng), signed_unit(rng), unit(rng));
    sample = glm::normalize(sample) * unit(rng);
    // Accelerating scale: pull samples toward the centre of the hemisphere.
    const float t = static_cast<float>(i) / static_cast<float>(kKernelSize);
    kernel_[static_cast<std::size_t>(i)] = sample * (0.1F + (0.9F * t * t));
  }
}

RhiSsaoPass::~RhiSsaoPass() {
  release();
}

void RhiSsaoPass::setDepthTexture(QRhiTexture* depth) {
  if (depth_ == depth) {
    return;
  }
  depth_ = depth;
  input_bindings_dirty_ = true;
}

void RhiSsaoPass::setRadius(float radius_m) {
  radius_m_ = radius_m;
}

void RhiSsaoPass::setPower(float ao_power) {
  ao_power_ = ao_power;
}

QRhiTexture* RhiSsaoPass::output() const {
  return ready() ? blur_.texture : nullptr;
}

bool RhiSsaoPass::ready() const {
  return blur_.pipeline != nullptr && ao_.pipeline != nullptr && depth_ != nullptr && !input_bindings_dirty_;
}

bool RhiSsaoPass::buildStage(
    QRhi& rhi, Stage& stage, const QSize& size, const char* frag_path, quint32 ubo_size, QRhiTexture* input,
    QRhiSampler* sampler) {
  stage.texture = rhi.newTexture(QRhiTexture::R16F, size, 1, QRhiTexture::RenderTarget);
  if (stage.texture == nullptr || !stage.texture->create()) {
    qCWarning(lcRhiSsao) << "AO target creation failed";
    return false;
  }
  stage.rt = rhi.newTextureRenderTarget({{stage.texture}});
  if (stage.rt == nullptr) {
    return false;
  }
  stage.rpd = stage.rt->newCompatibleRenderPassDescriptor();
  if (stage.rpd == nullptr) {
    return false;
  }
  stage.rt->setRenderPassDescriptor(stage.rpd);
  if (!stage.rt->create()) {
    qCWarning(lcRhiSsao) << "AO render target creation failed";
    return false;
  }

  stage.ubo = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, ubo_size);
  if (!stage.ubo->create()) {
    return false;
  }
  stage.srb = rhi.newShaderResourceBindings();
  stage.srb->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, stage.ubo),
      QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, input, sampler),
  });
  if (!stage.srb->create()) {
    return false;
  }

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/present.vert.qsb"));
  const QShader frag = loadBakedShader(QString::fromLatin1(frag_path));
  if (!vert.isValid() || !frag.isValid()) {
    return false;
  }
  stage.pipeline = rhi.newGraphicsPipeline();
  stage.pipeline->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
  stage.pipeline->setVertexInputLayout({});  // attributeless fullscreen triangle
  stage.pipeline->setShaderResourceBindings(stage.srb);
  stage.pipeline->setRenderPassDescriptor(stage.rpd);
  stage.pipeline->setSampleCount(1);
  stage.pipeline->setDepthTest(false);
  stage.pipeline->setDepthWrite(false);
  stage.pipeline->setCullMode(QRhiGraphicsPipeline::None);
  if (!stage.pipeline->create()) {
    qCWarning(lcRhiSsao) << "AO pipeline creation failed";
    return false;
  }
  return true;
}

bool RhiSsaoPass::ensure(QRhi& rhi, const QSize& pixel_size) {
  if (pixel_size.isEmpty()) {
    return false;
  }
  if (rhi_ == &rhi && size_ == pixel_size && ao_.pipeline != nullptr && blur_.pipeline != nullptr) {
    return true;
  }
  release();
  rhi_ = &rhi;
  size_ = pixel_size;

  // Nearest for both stages. The AO stage reconstructs positions from depth, where
  // a filtered fetch would interpolate across a silhouette and invent geometry; the
  // blur stage does its own explicit 4x4 average and wants exact texels.
  sampler_ = rhi.newSampler(
      QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge,
      QRhiSampler::ClampToEdge);
  if (sampler_ == nullptr || !sampler_->create()) {
    release();
    return false;
  }

  // The AO stage's input is the scene depth, which may not be bound yet; bind its
  // own output as a stand-in purely to fix the SRB LAYOUT, then rebind for real.
  if (!buildStage(
          rhi, ao_, pixel_size, ":/scene3d_shaders/ssao.frag.qsb", sizeof(SsaoUbo),
          depth_ != nullptr ? depth_ : ao_.texture, sampler_)) {
    release();
    return false;
  }
  if (!buildStage(
          rhi, blur_, pixel_size, ":/scene3d_shaders/ssao_blur.frag.qsb", sizeof(BlurUbo), ao_.texture, sampler_)) {
    release();
    return false;
  }
  input_bindings_dirty_ = true;
  return true;
}

void RhiSsaoPass::rebuildInputBindings() {
  if (rhi_ == nullptr || ao_.srb == nullptr || depth_ == nullptr) {
    return;
  }
  // Same binding slots, different texture: layout-compatible, so the pipeline built
  // against the placeholder stays valid.
  ao_.srb->destroy();
  ao_.srb->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ao_.ubo),
      QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, depth_, sampler_),
  });
  if (!ao_.srb->create()) {
    qCWarning(lcRhiSsao) << "AO input binding rebuild failed";
    return;
  }
  input_bindings_dirty_ = false;
}

void RhiSsaoPass::render(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) {
  if (rhi_ == nullptr || ao_.pipeline == nullptr || blur_.pipeline == nullptr || depth_ == nullptr) {
    return;
  }
  if (input_bindings_dirty_) {
    rebuildInputBindings();
    if (input_bindings_dirty_) {
      return;
    }
  }

  const float texel_x = 1.0F / static_cast<float>(size_.width());
  const float texel_y = 1.0F / static_cast<float>(size_.height());

  SsaoUbo ao_ubo{};
  std::memcpy(ao_ubo.screen_from_view, &ctx.screen_from_view[0][0], sizeof(ao_ubo.screen_from_view));
  std::memcpy(ao_ubo.view_from_screen, &ctx.view_from_screen[0][0], sizeof(ao_ubo.view_from_screen));
  for (int i = 0; i < kKernelSize; ++i) {
    const glm::vec3& k = kernel_[static_cast<std::size_t>(i)];
    ao_ubo.kernel[i][0] = k.x;
    ao_ubo.kernel[i][1] = k.y;
    ao_ubo.kernel[i][2] = k.z;
  }
  ao_ubo.texel[0] = texel_x;
  ao_ubo.texel[1] = texel_y;
  ao_ubo.radius = radius_m_;
  ao_ubo.bias = 0.025F;
  ao_ubo.ao_power = ao_power_;

  BlurUbo blur_ubo{};
  blur_ubo.texel[0] = texel_x;
  blur_ubo.texel[1] = texel_y;

  // Uploads must be recorded before a pass opens, so they ride the first beginPass.
  QRhiResourceUpdateBatch* updates = rhi_->nextResourceUpdateBatch();
  updates->updateDynamicBuffer(ao_.ubo, 0, sizeof(SsaoUbo), &ao_ubo);
  updates->updateDynamicBuffer(blur_.ubo, 0, sizeof(BlurUbo), &blur_ubo);

  const QRhiViewport viewport(0.0F, 0.0F, static_cast<float>(size_.width()), static_cast<float>(size_.height()));

  cb.beginPass(ao_.rt, Qt::transparent, {1.0F, 0}, updates);
  cb.setGraphicsPipeline(ao_.pipeline);
  cb.setViewport(viewport);
  cb.setShaderResources(ao_.srb);
  cb.draw(3);
  cb.endPass();

  cb.beginPass(blur_.rt, Qt::transparent, {1.0F, 0});
  cb.setGraphicsPipeline(blur_.pipeline);
  cb.setViewport(viewport);
  cb.setShaderResources(blur_.srb);
  cb.draw(3);
  cb.endPass();
}

void RhiSsaoPass::releaseStage(Stage& stage) {
  delete stage.pipeline;
  stage.pipeline = nullptr;
  delete stage.srb;
  stage.srb = nullptr;
  delete stage.ubo;
  stage.ubo = nullptr;
  delete stage.rt;
  stage.rt = nullptr;
  delete stage.rpd;
  stage.rpd = nullptr;
  delete stage.texture;
  stage.texture = nullptr;
}

void RhiSsaoPass::release() {
  releaseStage(blur_);
  releaseStage(ao_);
  delete sampler_;
  sampler_ = nullptr;
  rhi_ = nullptr;
  size_ = {};
  input_bindings_dirty_ = true;
}

}  // namespace pj::scene3d::rhi
