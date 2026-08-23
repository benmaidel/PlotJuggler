// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <algorithm>
#include <cstring>
#include <glm/glm.hpp>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiCloud, "pj.scene3d.rhi.pointcloud")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiCloud) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiPointcloudPass::~RhiPointcloudPass() {
  release();
}

void RhiPointcloudPass::setPoints(const void* data, int count, const Layout& layout) {
  points_.clear();
  point_count_ = 0;
  layout_ = layout;
  if (data != nullptr && count > 0 && layout.stride_bytes > 0) {
    const auto* bytes = static_cast<const std::uint8_t*>(data);
    const std::size_t total = static_cast<std::size_t>(count) * static_cast<std::size_t>(layout.stride_bytes);
    points_.assign(bytes, bytes + total);
    point_count_ = count;
  }
  points_dirty_ = true;
}

void RhiPointcloudPass::setScalarRange(float min_value, float max_value) {
  scalar_min_ = min_value;
  scalar_max_ = max_value;
}

void RhiPointcloudPass::setPointRadius(float metres) {
  point_radius_ = metres;
}

bool RhiPointcloudPass::ensureColormapTexture(QRhi& /*rhi*/, QRhiResourceUpdateBatch& updates) {
  // The texture itself is created in initialize(); this only fills it in.
  if (colormap_uploaded_ || colormap_tex_ == nullptr) {
    return colormap_tex_ != nullptr;
  }
  const std::vector<std::uint8_t> lut = PJ::buildColormapLut(PJ::kColormapLutWidth);
  QRhiTextureSubresourceUploadDescription sub(lut.data(), static_cast<quint32>(lut.size()));
  sub.setSourceSize(QSize(PJ::kColormapLutWidth, PJ::kColormapCount));
  updates.uploadTexture(colormap_tex_, QRhiTextureUploadDescription({0, 0, sub}));
  colormap_uploaded_ = true;
  return true;
}

bool RhiPointcloudPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/pointcloud.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/pointcloud.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(PointcloudUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }
  // Linear across t so the colormap ramp is smooth; nearest across rows so a row
  // never blends into its neighbouring colormap.
  colormap_sampler_ = rhi.newSampler(
      QRhiSampler::Linear, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge);
  if (!colormap_sampler_->create()) {
    release();
    return false;
  }
  // The colormap texture must exist NOW, not lazily in prepare(): a pipeline is
  // compiled against the resource LAYOUT of the SRB it is created with, so an SRB
  // that gains bindings later is not layout-compatible and the draw reads garbage.
  // (This is the same reason pj_scene2D binds 1x1 placeholder textures.)
  colormap_tex_ = rhi.newTexture(QRhiTexture::RGBA8, QSize(PJ::kColormapLutWidth, PJ::kColormapCount));
  if (colormap_tex_ == nullptr || !colormap_tex_->create()) {
    qCWarning(lcRhiCloud) << "colormap LUT texture creation failed";
    release();
    return false;
  }

  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      QRhiShaderResourceBinding::sampledTexture(
          1, QRhiShaderResourceBinding::FragmentStage, colormap_tex_, colormap_sampler_),
  });
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});

  // ONE binding: the per-instance point records. The quad corners come from
  // gl_VertexIndex in the shader (see pointcloud.vert) rather than a second,
  // per-vertex binding.
  QRhiVertexInputLayout layout;
  layout.setBindings({
      QRhiVertexInputBinding(static_cast<quint32>(layout_.stride_bytes), QRhiVertexInputBinding::PerInstance),
  });
  // A single Float4 covering {xyz, scalar}. See pointcloud.vert for why this is
  // not a Float3 plus a separate float at offset 12.
  layout.setAttributes({
      QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float4, static_cast<quint32>(layout_.xyz_offset)),
  });
  pipeline_->setVertexInputLayout(layout);

  // Opaque geometry: the fragment shader discards outside the disc, so depth
  // writes stay correct and points occlude one another properly. No culling —
  // the billboard's winding depends on the camera.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(true);
  pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiCloud) << "pointcloud pipeline creation failed";
    release();
    return false;
  }

  colormap_uploaded_ = false;
  points_dirty_ = true;
  return true;
}

void RhiPointcloudPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr) {
    return;
  }

  if (!ensureColormapTexture(*rhi_, updates)) {
    return;
  }

  if (points_dirty_) {
    if (point_count_ > instance_capacity_) {
      delete instance_buf_;
      instance_capacity_ = std::max(point_count_ * 2, 1024);
      // Static, not Dynamic: Dynamic is meant for small per-frame data (uniform
      // blocks) and some backends stage it through a size-limited scratch area,
      // which silently left most of a 2000-point cloud as zeroes. A cloud is
      // large and changes only when a new message arrives, which is exactly what
      // Static + uploadStaticBuffer is for.
      instance_buf_ = rhi_->newBuffer(
          QRhiBuffer::Static, QRhiBuffer::VertexBuffer,
          static_cast<quint32>(instance_capacity_) * static_cast<quint32>(layout_.stride_bytes));
      if (instance_buf_ == nullptr || !instance_buf_->create()) {
        instance_capacity_ = 0;
        return;
      }
    }
    if (!points_.empty() && instance_buf_ != nullptr) {
      updates.uploadStaticBuffer(instance_buf_, 0, static_cast<quint32>(points_.size()), points_.data());
    }
    points_dirty_ = false;
  }

  // Camera basis for the billboard, recovered from view_proj's rows. The upper 3x3
  // of a view matrix is orthonormal, so its ROWS are the camera's right/up/forward
  // in world space; view_proj scales them per axis but normalizing restores the
  // directions, which is all the expansion needs.
  const glm::mat4& vp = ctx.view_proj;
  const glm::vec3 right = glm::normalize(glm::vec3(vp[0][0], vp[1][0], vp[2][0]));
  const glm::vec3 up = glm::normalize(glm::vec3(vp[0][1], vp[1][1], vp[2][1]));

  PointcloudUbo ubo{};
  std::memcpy(ubo.view_proj, &vp[0][0], sizeof(ubo.view_proj));
  std::memcpy(ubo.model, &model_[0][0], sizeof(ubo.model));
  ubo.cam_right[0] = right.x;
  ubo.cam_right[1] = right.y;
  ubo.cam_right[2] = right.z;
  ubo.cam_up[0] = up.x;
  ubo.cam_up[1] = up.y;
  ubo.cam_up[2] = up.z;
  ubo.point_radius = point_radius_;
  ubo.scalar_min = scalar_min_;
  ubo.scalar_max = scalar_max_;
  // Sample the row's centre: rows are colormap ids, and hitting the centre avoids
  // bleeding into a neighbour even if the sampler is ever switched to linear.
  ubo.colormap_row = (static_cast<float>(colormap_) + 0.5F) / static_cast<float>(PJ::kColormapCount);
  updates.updateDynamicBuffer(ubo_, 0, sizeof(PointcloudUbo), &ubo);
}

void RhiPointcloudPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || point_count_ == 0 || instance_buf_ == nullptr) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput input(instance_buf_, 0);
  cb.setVertexInput(0, 1, &input);
  // Six vertices per instance form the quad; the shader derives each corner from
  // gl_VertexIndex, so there is no index buffer.
  cb.draw(6, static_cast<quint32>(point_count_));
}

void RhiPointcloudPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete colormap_sampler_;
  colormap_sampler_ = nullptr;
  delete colormap_tex_;
  colormap_tex_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  delete instance_buf_;
  instance_buf_ = nullptr;
  instance_capacity_ = 0;
  colormap_uploaded_ = false;
  points_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
