// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"

#include <QFile>
#include <QLatin1StringView>
#include <QLoggingCategory>
#include <algorithm>
#include <cstring>
#include <utility>

#include "pj_scene3d_widgets/gizmos/arrow_mesh.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiPoses, "pj.scene3d.rhi.poses")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiPoses) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiPosesPass::~RhiPosesPass() {
  release();
}

void RhiPosesPass::setInstances(std::vector<PoseTriadInstance> instances) {
  instances_ = std::move(instances);
  instances_dirty_ = true;
}

void RhiPosesPass::setFrameWorld(const glm::mat4& frame_world) {
  frame_world_ = frame_world;
}

bool RhiPosesPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QLatin1StringView(arrow::kVertShader));
  const QShader frag = loadBakedShader(QLatin1StringView(arrow::kFragShader));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(arrow::Ubo));
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
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});

  // Binding 0: the shared arrow mesh, per vertex (pos + normal, 24 bytes).
  // Binding 1: the staged PoseTriadInstance array, per instance (80 bytes).
  QRhiVertexInputLayout layout;
  layout.setBindings({
      QRhiVertexInputBinding(6 * sizeof(float)),
      QRhiVertexInputBinding(sizeof(PoseTriadInstance), QRhiVertexInputBinding::PerInstance),
  });
  layout.setAttributes({
      QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
      QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)),
      QRhiVertexInputAttribute(1, 2, QRhiVertexInputAttribute::Float4, 0),
      QRhiVertexInputAttribute(1, 3, QRhiVertexInputAttribute::Float4, arrow::kModelColumnStride),
      QRhiVertexInputAttribute(1, 4, QRhiVertexInputAttribute::Float4, 2 * arrow::kModelColumnStride),
      QRhiVertexInputAttribute(1, 5, QRhiVertexInputAttribute::Float4, 3 * arrow::kModelColumnStride),
      QRhiVertexInputAttribute(1, 6, QRhiVertexInputAttribute::Float4, arrow::kColorOffset),
  });
  pipeline_->setVertexInputLayout(layout);

  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(true);
  pipeline_->setCullMode(QRhiGraphicsPipeline::Back);

  // Unlike the TF triads these honour a user opacity knob (PoseTriadStyle::opacity
  // reaches the shader as the instance colour's alpha), so the pipeline blends.
  // Depth write stays on, matching the GL pass: translucent arms therefore occlude
  // each other in draw order rather than being sorted, which is acceptable for
  // gizmos and avoids a per-frame sort over a particle cloud.
  QRhiGraphicsPipeline::TargetBlend blend;
  blend.enable = true;
  blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
  blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  blend.srcAlpha = QRhiGraphicsPipeline::One;
  blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  pipeline_->setTargetBlends({blend});

  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiPoses) << "poses pipeline creation failed";
    release();
    return false;
  }

  mesh_uploaded_ = false;
  instances_dirty_ = true;
  return true;
}

void RhiPosesPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr) {
    return;
  }

  if (!mesh_uploaded_) {
    // The same unit arrow the TF triads use, so the two gizmo families share
    // proportions as well as shading; each arm's own model matrix scales it.
    ArrowMeshParams params;
    params.length = 1.0F;
    const ArrowMeshData mesh = buildArrowMesh(params);
    const auto vbytes = static_cast<quint32>(mesh.vertices.size() * sizeof(float));
    const auto ibytes = static_cast<quint32>(mesh.indices.size() * sizeof(std::uint32_t));

    delete vbo_;
    delete ibo_;
    vbo_ = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, vbytes);
    ibo_ = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, ibytes);
    if (vbo_ == nullptr || ibo_ == nullptr || !vbo_->create() || !ibo_->create()) {
      index_count_ = 0;
      return;
    }
    updates.uploadStaticBuffer(vbo_, mesh.vertices.data());
    updates.uploadStaticBuffer(ibo_, mesh.indices.data());
    index_count_ = static_cast<int>(mesh.indices.size());
    mesh_uploaded_ = true;
  }

  if (instances_dirty_) {
    const int needed = static_cast<int>(instances_.size());
    if (needed > instance_capacity_) {
      delete instance_buf_;
      instance_capacity_ = std::max(needed * 2, 64);
      instance_buf_ = rhi_->newBuffer(
          QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer,
          static_cast<quint32>(instance_capacity_) * static_cast<quint32>(sizeof(PoseTriadInstance)));
      if (instance_buf_ == nullptr || !instance_buf_->create()) {
        instance_capacity_ = 0;
        return;
      }
    }
    if (!instances_.empty()) {
      updates.updateDynamicBuffer(
          instance_buf_, 0, static_cast<quint32>(needed * static_cast<int>(sizeof(PoseTriadInstance))),
          instances_.data());
    }
    instances_dirty_ = false;
  }

  arrow::Ubo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  std::memcpy(ubo.frame_world, &frame_world_[0][0], sizeof(ubo.frame_world));
  updates.updateDynamicBuffer(ubo_, 0, sizeof(arrow::Ubo), &ubo);
}

void RhiPosesPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || index_count_ == 0 || instance_buf_ == nullptr || instances_.empty()) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput inputs[] = {{vbo_, 0}, {instance_buf_, 0}};
  cb.setVertexInput(0, 2, inputs, ibo_, 0, QRhiCommandBuffer::IndexUInt32);
  cb.drawIndexed(static_cast<quint32>(index_count_), static_cast<quint32>(instances_.size()));
}

void RhiPosesPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  delete instance_buf_;
  instance_buf_ = nullptr;
  delete ibo_;
  ibo_ = nullptr;
  delete vbo_;
  vbo_ = nullptr;
  index_count_ = 0;
  instance_capacity_ = 0;
  mesh_uploaded_ = false;
  instances_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
