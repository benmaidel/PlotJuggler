// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_grid_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <cstring>
#include <vector>

#include "pj_scene3d_widgets/passes/grid_geometry.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiGrid, "pj.scene3d.rhi.grid")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiGrid) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiGridPass::~RhiGridPass() {
  release();
}

void RhiGridPass::setGeometry(float extent_m, int divisions) {
  if (extent_m == extent_m_ && divisions == divisions_) {
    return;
  }
  extent_m_ = extent_m;
  divisions_ = divisions;
  geometry_dirty_ = true;
}

bool RhiGridPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd) {
  if (pipeline_ != nullptr && rhi_ == &rhi) {
    return true;
  }
  // A different QRhi means the old device's objects are already invalid.
  release();
  rhi_ = &rhi;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/grid.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/grid.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    qCWarning(lcRhiGrid) << "grid shader pack unusable for this backend; pass disabled";
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(GridUbo));
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

  // GridVertex is {glm::vec3 pos; float parity;} == 16 bytes. Only the position
  // is declared: the lines pass ignores parity, and an undeclared trailing
  // attribute costs nothing because the stride already accounts for it.
  QRhiVertexInputLayout layout;
  layout.setBindings({QRhiVertexInputBinding(sizeof(GridVertex))});
  layout.setAttributes({QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0)});
  pipeline_->setVertexInputLayout(layout);

  // The grid is a depth-tested opaque draw so geometry can occlude it, but it
  // must not write depth: it is a reference overlay on the ground plane, and
  // letting it own depth would make thin lines reject fragments of objects
  // resting exactly on z=0.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(false);
  pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  pipeline_->setShaderResourceBindings(srb_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiGrid) << "grid pipeline creation failed";
    release();
    return false;
  }

  // Force a re-upload: the vertex buffer belongs to the previous device.
  geometry_dirty_ = true;
  return true;
}

void RhiGridPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr) {
    return;
  }

  if (geometry_dirty_) {
    const std::vector<GridVertex> vertices = buildGridLines(extent_m_, divisions_);
    const int bytes = static_cast<int>(vertices.size() * sizeof(GridVertex));
    // Immutable: the tessellation only changes when the user edits extent or
    // divisions, which re-creates the buffer rather than streaming it per frame.
    delete vbo_;
    vbo_ = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, bytes);
    if (vbo_ == nullptr || !vbo_->create()) {
      vertex_count_ = 0;
      return;
    }
    updates.uploadStaticBuffer(vbo_, vertices.data());
    vertex_count_ = static_cast<int>(vertices.size());
    geometry_dirty_ = false;
  }

  GridUbo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  ubo.line_color[0] = line_color_.r;
  ubo.line_color[1] = line_color_.g;
  ubo.line_color[2] = line_color_.b;
  ubo.line_color[3] = line_color_.a;
  updates.updateDynamicBuffer(ubo_, 0, sizeof(GridUbo), &ubo);
}

void RhiGridPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || vbo_ == nullptr || vertex_count_ == 0) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput input(vbo_, 0);
  cb.setVertexInput(0, 1, &input);
  cb.draw(static_cast<quint32>(vertex_count_));
}

void RhiGridPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  delete vbo_;
  vbo_ = nullptr;
  vertex_count_ = 0;
  geometry_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
