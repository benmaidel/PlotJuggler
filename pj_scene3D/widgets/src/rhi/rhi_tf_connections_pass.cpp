// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_tf_connections_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <algorithm>
#include <cstring>
#include <utility>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiTf, "pj.scene3d.rhi.tf_connections")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiTf) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiTfConnectionsPass::~RhiTfConnectionsPass() {
  release();
}

void RhiTfConnectionsPass::setSegments(std::vector<glm::vec3> endpoints) {
  // Drop an unpaired tail rather than draw a segment to an undefined point.
  if ((endpoints.size() % 2U) != 0U) {
    endpoints.pop_back();
  }
  endpoints_ = std::move(endpoints);
  geometry_dirty_ = true;
}

bool RhiTfConnectionsPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/lines.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/lines.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(LinesUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }
  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({QRhiShaderResourceBinding::uniformBuffer(
      0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_)});
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setTopology(QRhiGraphicsPipeline::Lines);
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});

  // Tight vec3 endpoints: the shared shader reads only the position, so this pass
  // needs no padding to match the grid's wider vertex.
  QRhiVertexInputLayout layout;
  layout.setBindings({QRhiVertexInputBinding(sizeof(glm::vec3))});
  layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0)});
  pipeline_->setVertexInputLayout(layout);

  // Annotation, not geometry: depth-tested so it is occluded by solid objects, but
  // not depth-writing, so a thin line never rejects fragments of the frames it
  // connects.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(false);
  pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiTf) << "TF connections pipeline creation failed";
    release();
    return false;
  }
  geometry_dirty_ = true;
  return true;
}

void RhiTfConnectionsPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr) {
    return;
  }

  if (geometry_dirty_) {
    const int needed = static_cast<int>(endpoints_.size());
    if (needed > vertex_capacity_) {
      // Grow geometrically: a live TF tree gains frames over time, and Dynamic
      // because the endpoints move every time the transforms update.
      delete vbo_;
      vertex_capacity_ = std::max(needed * 2, 64);
      vbo_ = rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer,
                             static_cast<quint32>(vertex_capacity_) * static_cast<quint32>(sizeof(glm::vec3)));
      if (vbo_ == nullptr || !vbo_->create()) {
        vertex_capacity_ = 0;
        vertex_count_ = 0;
        return;
      }
    }
    if (needed > 0 && vbo_ != nullptr) {
      updates.updateDynamicBuffer(vbo_, 0, static_cast<quint32>(needed * static_cast<int>(sizeof(glm::vec3))),
                                  endpoints_.data());
    }
    vertex_count_ = needed;
    geometry_dirty_ = false;
  }

  LinesUbo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  ubo.color[0] = color_.r;
  ubo.color[1] = color_.g;
  ubo.color[2] = color_.b;
  ubo.color[3] = color_.a;
  updates.updateDynamicBuffer(ubo_, 0, sizeof(LinesUbo), &ubo);
}

void RhiTfConnectionsPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || vbo_ == nullptr || vertex_count_ < 2) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput input(vbo_, 0);
  cb.setVertexInput(0, 1, &input);
  cb.draw(static_cast<quint32>(vertex_count_));
}

void RhiTfConnectionsPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  delete vbo_;
  vbo_ = nullptr;
  vertex_count_ = 0;
  vertex_capacity_ = 0;
  geometry_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
