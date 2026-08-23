// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <array>
#include <cstring>

#include "pj_scene3d_widgets/cube_mesh.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiVoxel, "pj.scene3d.rhi.voxel")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiVoxel) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiVoxelGridPass::~RhiVoxelGridPass() {
  release();
}

void RhiVoxelGridPass::setField(const float* values, int columns, int rows, int slices) {
  if (values == nullptr || columns <= 0 || rows <= 0 || slices <= 0) {
    values_.clear();
    columns_ = 0;
    rows_ = 0;
    slices_ = 0;
    return;
  }
  const std::size_t count =
      static_cast<std::size_t>(columns) * static_cast<std::size_t>(rows) * static_cast<std::size_t>(slices);
  values_.assign(values, values + count);
  columns_ = columns;
  rows_ = rows;
  slices_ = slices;
  field_dirty_ = true;
}

void RhiVoxelGridPass::setColorRange(float lo, float hi) {
  color_lo_ = lo;
  color_hi_ = hi;
}

bool RhiVoxelGridPass::ensureVolumeTexture(QRhi& rhi) {
  if (voxelCount() == 0) {
    return false;
  }
  if (volume_tex_ != nullptr && tex_columns_ == columns_ && tex_rows_ == rows_ && tex_slices_ == slices_) {
    return true;
  }
  if (!rhi.isFeatureSupported(QRhi::ThreeDimensionalTextures)) {
    qCWarning(lcRhiVoxel) << "backend has no 3D textures; voxel pass disabled";
    return false;
  }

  delete volume_tex_;
  volume_tex_ = rhi.newTexture(QRhiTexture::R32F, columns_, rows_, slices_, 1, QRhiTexture::ThreeDimensional);
  if (volume_tex_ == nullptr || !volume_tex_->create()) {
    qCWarning(lcRhiVoxel) << "3D volume texture creation failed at" << columns_ << rows_ << slices_;
    delete volume_tex_;
    volume_tex_ = nullptr;
    tex_columns_ = 0;
    tex_rows_ = 0;
    tex_slices_ = 0;
    return false;
  }
  tex_columns_ = columns_;
  tex_rows_ = rows_;
  tex_slices_ = slices_;
  field_dirty_ = true;

  // Same binding types, so this stays layout-compatible with the pipeline.
  srb_->destroy();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      QRhiShaderResourceBinding::sampledTexture(
          1, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, volume_tex_,
          volume_sampler_),
      QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, colormap_tex_,
                                                colormap_sampler_),
  });
  return srb_->create();
}

bool RhiVoxelGridPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/voxel.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/voxel.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(VoxelUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }
  // Nearest for the volume: the shader texelFetches exact lattice cells, so any
  // filtering would only blur values across voxel boundaries.
  volume_sampler_ = rhi.newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None,
                                   QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge);
  colormap_sampler_ = rhi.newSampler(QRhiSampler::Linear, QRhiSampler::Nearest, QRhiSampler::None,
                                     QRhiSampler::ClampToEdge, QRhiSampler::ClampToEdge);
  if (!volume_sampler_->create() || !colormap_sampler_->create()) {
    release();
    return false;
  }

  colormap_tex_ = rhi.newTexture(QRhiTexture::RGBA8, QSize(PJ::kColormapLutWidth, PJ::kColormapCount));
  // 1x1x1 stand-in so the SRB layout is final before the pipeline is created.
  volume_tex_ = rhi.newTexture(QRhiTexture::R32F, 1, 1, 1, 1, QRhiTexture::ThreeDimensional);
  if (colormap_tex_ == nullptr || !colormap_tex_->create() || volume_tex_ == nullptr || !volume_tex_->create()) {
    release();
    return false;
  }
  tex_columns_ = 1;
  tex_rows_ = 1;
  tex_slices_ = 1;

  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      // Visible to the VERTEX stage as well: that is where the lattice value is
      // fetched to decide whether the cube is drawn at all.
      QRhiShaderResourceBinding::sampledTexture(
          1, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, volume_tex_,
          volume_sampler_),
      QRhiShaderResourceBinding::sampledTexture(2, QRhiShaderResourceBinding::FragmentStage, colormap_tex_,
                                                colormap_sampler_),
  });
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});

  QRhiVertexInputLayout layout;
  layout.setBindings({QRhiVertexInputBinding(sizeof(CubeVertex))});
  layout.setAttributes({
      QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, 0),
      QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, 3 * sizeof(float)),
  });
  pipeline_->setVertexInputLayout(layout);

  // Solid opaque geometry.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(true);
  pipeline_->setCullMode(QRhiGraphicsPipeline::Back);
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiVoxel) << "voxel pipeline creation failed";
    release();
    return false;
  }

  cube_uploaded_ = false;
  colormap_uploaded_ = false;
  field_dirty_ = true;
  return true;
}

void RhiVoxelGridPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr || voxelCount() == 0) {
    return;
  }

  if (!cube_uploaded_) {
    delete cube_vbo_;
    delete cube_ibo_;
    cube_vbo_ = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, sizeof(kCubeVertices));
    // kCubeIndices is uint8_t, which QRhi cannot index with (IndexUInt16 /
    // IndexUInt32 only), so widen it on the way to the GPU.
    std::array<std::uint16_t, kCubeIndices.size()> indices{};
    for (std::size_t i = 0; i < kCubeIndices.size(); ++i) {
      indices[i] = kCubeIndices[i];
    }
    cube_ibo_ = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer,
                                static_cast<quint32>(indices.size() * sizeof(std::uint16_t)));
    if (cube_vbo_ == nullptr || cube_ibo_ == nullptr || !cube_vbo_->create() || !cube_ibo_->create()) {
      return;
    }
    updates.uploadStaticBuffer(cube_vbo_, kCubeVertices.data());
    updates.uploadStaticBuffer(cube_ibo_, indices.data());
    cube_uploaded_ = true;
  }

  if (!colormap_uploaded_ && colormap_tex_ != nullptr) {
    const std::vector<std::uint8_t> lut = PJ::buildColormapLut(PJ::kColormapLutWidth);
    QRhiTextureSubresourceUploadDescription sub(lut.data(), static_cast<int>(lut.size()));
    sub.setSourceSize(QSize(PJ::kColormapLutWidth, PJ::kColormapCount));
    updates.uploadTexture(colormap_tex_, QRhiTextureUploadDescription({0, 0, sub}));
    colormap_uploaded_ = true;
  }

  if (!ensureVolumeTexture(*rhi_)) {
    return;
  }

  if (field_dirty_) {
    // A 3D texture uploads one DEPTH SLICE per entry, addressed through the
    // entry's layer index — there is no single-shot whole-volume upload.
    std::vector<QRhiTextureUploadEntry> entries;
    entries.reserve(static_cast<std::size_t>(slices_));
    const std::size_t slice_floats = static_cast<std::size_t>(columns_) * static_cast<std::size_t>(rows_);
    const int slice_bytes = static_cast<int>(slice_floats * sizeof(float));
    for (int z = 0; z < slices_; ++z) {
      QRhiTextureSubresourceUploadDescription sub(values_.data() + (static_cast<std::size_t>(z) * slice_floats),
                                                  slice_bytes);
      sub.setSourceSize(QSize(columns_, rows_));
      entries.emplace_back(z, 0, sub);
    }
    // QRhiTextureUploadDescription has no iterator-pair constructor, only
    // setEntries().
    QRhiTextureUploadDescription desc;
    desc.setEntries(entries.cbegin(), entries.cend());
    updates.uploadTexture(volume_tex_, desc);
    field_dirty_ = false;
  }

  VoxelUbo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  std::memcpy(ubo.model, &model_[0][0], sizeof(ubo.model));
  ubo.cell_size[0] = cell_size_.x;
  ubo.cell_size[1] = cell_size_.y;
  ubo.cell_size[2] = cell_size_.z;
  ubo.dims[0] = columns_;
  ubo.dims[1] = rows_;
  ubo.dims[2] = slices_;
  ubo.draw_mode = static_cast<std::int32_t>(draw_mode_);
  ubo.threshold = threshold_;
  ubo.range_hi = range_hi_;
  ubo.color_lo = color_lo_;
  ubo.color_hi = color_hi_;
  ubo.colormap_row = (static_cast<float>(colormap_) + 0.5F) / static_cast<float>(PJ::kColormapCount);
  updates.updateDynamicBuffer(ubo_, 0, sizeof(VoxelUbo), &ubo);
}

void RhiVoxelGridPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || voxelCount() == 0 || cube_vbo_ == nullptr || volume_tex_ == nullptr) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  const QRhiCommandBuffer::VertexInput input(cube_vbo_, 0);
  cb.setVertexInput(0, 1, &input, cube_ibo_, 0, QRhiCommandBuffer::IndexUInt16);
  // One draw for the whole lattice; the shader clips the voxels the predicate
  // rejects.
  cb.drawIndexed(static_cast<quint32>(kCubeIndices.size()), static_cast<quint32>(voxelCount()));
}

void RhiVoxelGridPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete colormap_sampler_;
  colormap_sampler_ = nullptr;
  delete volume_sampler_;
  volume_sampler_ = nullptr;
  delete colormap_tex_;
  colormap_tex_ = nullptr;
  delete volume_tex_;
  volume_tex_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  delete cube_ibo_;
  cube_ibo_ = nullptr;
  delete cube_vbo_;
  cube_vbo_ = nullptr;
  tex_columns_ = 0;
  tex_rows_ = 0;
  tex_slices_ = 0;
  cube_uploaded_ = false;
  colormap_uploaded_ = false;
  field_dirty_ = true;
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
