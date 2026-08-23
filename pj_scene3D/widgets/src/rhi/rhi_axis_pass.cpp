// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_axis_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <utility>

#include "pj_scene3d_widgets/gizmos/arrow_mesh.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiAxis, "pj.scene3d.rhi.axis")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiAxis) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

/// Rotation taking the mesh's +X axis onto each world axis, so one arrow mesh
/// serves all three. Index 0 = X (identity), 1 = Y, 2 = Z.
glm::mat4 axisRotation(int axis) {
  switch (axis) {
    case 1:
      return glm::rotate(glm::mat4(1.0F), glm::radians(90.0F), glm::vec3(0.0F, 0.0F, 1.0F));
    case 2:
      return glm::rotate(glm::mat4(1.0F), glm::radians(-90.0F), glm::vec3(0.0F, 1.0F, 0.0F));
    default:
      return glm::mat4(1.0F);
  }
}

constexpr float kAxisColors[3][4] = {
    {0.90F, 0.15F, 0.15F, 1.0F},  // X
    {0.15F, 0.75F, 0.20F, 1.0F},  // Y
    {0.20F, 0.35F, 0.95F, 1.0F},  // Z
};

}  // namespace

RhiAxisPass::~RhiAxisPass() {
  release();
}

void RhiAxisPass::setFrames(std::vector<glm::mat4> frame_transforms) {
  frames_ = std::move(frame_transforms);
  instances_dirty_ = true;
}

void RhiAxisPass::setAxisLength(float metres) {
  if (metres == axis_length_) {
    return;
  }
  axis_length_ = metres;
  instances_dirty_ = true;
}

void RhiAxisPass::rebuildInstances() {
  instances_.clear();
  instances_.reserve(frames_.size() * 3);
  // The mesh is a unit-length arrow along +X, so the triad's size is a uniform
  // scale — which keeps mat3(model) a valid normal matrix (see axis.vert).
  const glm::mat4 scale = glm::scale(glm::mat4(1.0F), glm::vec3(axis_length_));
  for (const glm::mat4& frame : frames_) {
    for (int axis = 0; axis < 3; ++axis) {
      Instance inst{};
      const glm::mat4 model = frame * axisRotation(axis) * scale;
      std::memcpy(inst.model, &model[0][0], sizeof(inst.model));
      std::memcpy(inst.color, kAxisColors[axis], sizeof(inst.color));
      instances_.push_back(inst);
    }
  }
}

bool RhiAxisPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/axis.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/axis.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(AxisUbo));
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
  // Binding 1: per-instance model matrix columns + colour (80 bytes). The
  // PerInstance classification is what turns one mesh into 3*frames draws.
  QRhiVertexInputLayout layout;
  layout.setBindings({
      QRhiVertexInputBinding(6 * sizeof(float)),
      QRhiVertexInputBinding(sizeof(Instance), QRhiVertexInputBinding::PerInstance),
  });
  layout.setAttributes({
      QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
      QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)),
      QRhiVertexInputAttribute(1, 2, QRhiVertexInputAttribute::Float4, 0),
      QRhiVertexInputAttribute(1, 3, QRhiVertexInputAttribute::Float4, 4 * sizeof(float)),
      QRhiVertexInputAttribute(1, 4, QRhiVertexInputAttribute::Float4, 8 * sizeof(float)),
      QRhiVertexInputAttribute(1, 5, QRhiVertexInputAttribute::Float4, 12 * sizeof(float)),
      QRhiVertexInputAttribute(1, 6, QRhiVertexInputAttribute::Float4, 16 * sizeof(float)),
  });
  pipeline_->setVertexInputLayout(layout);

  // Solid opaque geometry: depth-tested AND depth-writing, unlike the grid, so
  // triads occlude each other correctly.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(true);
  pipeline_->setCullMode(QRhiGraphicsPipeline::Back);
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiAxis) << "axis pipeline creation failed";
    release();
    return false;
  }

  mesh_uploaded_ = false;
  instances_dirty_ = true;
  return true;
}

void RhiAxisPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr) {
    return;
  }

  if (!mesh_uploaded_) {
    ArrowMeshParams params;
    params.length = 1.0F;  // unit arrow; the instance transform scales it
    const ArrowMeshData mesh = buildArrowMesh(params);
    const int vbytes = static_cast<int>(mesh.vertices.size() * sizeof(float));
    const int ibytes = static_cast<int>(mesh.indices.size() * sizeof(std::uint32_t));

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
    rebuildInstances();
    const int needed = static_cast<int>(instances_.size());
    if (needed > instance_capacity_) {
      // Grow geometrically: a live TF tree gains frames over time and should not
      // re-create the buffer on every addition.
      delete instance_buf_;
      instance_capacity_ = std::max(needed * 2, 16);
      instance_buf_ =
          rhi_->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::VertexBuffer,
                          static_cast<quint32>(instance_capacity_) * static_cast<quint32>(sizeof(Instance)));
      if (instance_buf_ == nullptr || !instance_buf_->create()) {
        instance_capacity_ = 0;
        return;
      }
    }
    if (!instances_.empty()) {
      updates.updateDynamicBuffer(instance_buf_, 0, static_cast<quint32>(needed * static_cast<int>(sizeof(Instance))),
                                  instances_.data());
    }
    instances_dirty_ = false;
  }

  AxisUbo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  updates.updateDynamicBuffer(ubo_, 0, sizeof(AxisUbo), &ubo);
}

void RhiAxisPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || index_count_ == 0 || instance_buf_ == nullptr || instances_.empty()) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput inputs[] = {{vbo_, 0}, {instance_buf_, 0}};
  cb.setVertexInput(0, 2, inputs, ibo_, 0, QRhiCommandBuffer::IndexUInt32);
  cb.drawIndexed(static_cast<quint32>(index_count_), static_cast<quint32>(instances_.size()));
}

void RhiAxisPass::release() {
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
