// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"

#include <QFile>
#include <QLoggingCategory>
#include <cstring>

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiOcc, "pj.scene3d.rhi.occupancy")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiOcc) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

}  // namespace

RhiOccupancyGridPass::~RhiOccupancyGridPass() {
  release();
}

void RhiOccupancyGridPass::setGrid(const std::uint8_t* cells, int width, int height) {
  if (cells == nullptr || width <= 0 || height <= 0) {
    cells_.clear();
    width_ = 0;
    height_ = 0;
    return;
  }
  const std::size_t count = static_cast<std::size_t>(width) * static_cast<std::size_t>(height);
  cells_.assign(cells, cells + count);
  width_ = width;
  height_ = height;
  // A resize re-creates the texture; a same-size replacement still needs the whole
  // surface re-uploaded, so both funnel through the full path.
  full_upload_pending_ = true;
  dirty_regions_.clear();
}

void RhiOccupancyGridPass::updateRegion(const QRect& region, const std::uint8_t* patch) {
  if (!hasGrid() || patch == nullptr || region.isEmpty()) {
    return;
  }
  // Refuse a patch that does not fit rather than clip it: the source rows are
  // strided by the patch's own width, so dropping columns would misalign every
  // subsequent row against the data the caller sent.
  if (!QRect(0, 0, width_, height_).contains(region)) {
    qCWarning(lcRhiOcc) << "patch" << region << "outside grid" << width_ << "x" << height_ << "- ignored";
    return;
  }

  // Fold the patch into the CPU copy so a later device loss can rebuild the whole
  // texture without the caller replaying every update.
  for (int row = 0; row < region.height(); ++row) {
    std::uint8_t* dst = cells_.data() + (static_cast<std::size_t>(region.y() + row) * width_) + region.x();
    const std::uint8_t* src = patch + (static_cast<std::size_t>(row) * region.width());
    std::memcpy(dst, src, static_cast<std::size_t>(region.width()));
  }

  // A pending full upload already covers this rectangle.
  if (!full_upload_pending_) {
    dirty_regions_.push_back(region);
  }
}

bool RhiOccupancyGridPass::ensureTexture(QRhi& rhi) {
  if (!hasGrid()) {
    return false;
  }
  if (grid_tex_ != nullptr && tex_width_ == width_ && tex_height_ == height_) {
    return true;
  }
  delete grid_tex_;
  grid_tex_ = rhi.newTexture(QRhiTexture::R8, QSize(width_, height_));
  if (grid_tex_ == nullptr || !grid_tex_->create()) {
    qCWarning(lcRhiOcc) << "cell texture creation failed at" << width_ << "x" << height_;
    delete grid_tex_;
    grid_tex_ = nullptr;
    tex_width_ = 0;
    tex_height_ = 0;
    return false;
  }
  tex_width_ = width_;
  tex_height_ = height_;
  full_upload_pending_ = true;
  dirty_regions_.clear();

  // Rebuild the bindings for the new texture. Same binding types, so this stays
  // layout-compatible with the pipeline (which is what matters — see
  // IRhiRenderPass::initialize).
  srb_->destroy();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, grid_tex_, sampler_),
  });
  return srb_->create();
}

bool RhiOccupancyGridPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/occupancy.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/occupancy.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(OccupancyUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }
  // Nearest: a costmap cell is a discrete value, and interpolating across the
  // unknown sentinel (255) would smear phantom occupancy along explored edges.
  sampler_ = rhi.newSampler(QRhiSampler::Nearest, QRhiSampler::Nearest, QRhiSampler::None, QRhiSampler::ClampToEdge,
                            QRhiSampler::ClampToEdge);
  if (!sampler_->create()) {
    release();
    return false;
  }
  // 1x1 stand-in so the SRB layout is final before the pipeline is created; the
  // real grid texture replaces it at the same binding later.
  grid_tex_ = rhi.newTexture(QRhiTexture::R8, QSize(1, 1));
  if (grid_tex_ == nullptr || !grid_tex_->create()) {
    release();
    return false;
  }
  tex_width_ = 1;
  tex_height_ = 1;

  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({
      QRhiShaderResourceBinding::uniformBuffer(
          0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_),
      QRhiShaderResourceBinding::sampledTexture(1, QRhiShaderResourceBinding::FragmentStage, grid_tex_, sampler_),
  });
  if (!srb_->create()) {
    release();
    return false;
  }

  pipeline_ = rhi.newGraphicsPipeline();
  pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
  pipeline_->setVertexInputLayout({});  // attributeless: corners from gl_VertexIndex

  // Translucent overlay, matching the GL renderer's blend: straight alpha for
  // colour, and (One, OneMinusSrcAlpha) for alpha so coverage accumulates instead
  // of the last draw replacing it.
  QRhiGraphicsPipeline::TargetBlend blend;
  blend.enable = true;
  blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
  blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  blend.srcAlpha = QRhiGraphicsPipeline::One;
  blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
  pipeline_->setTargetBlends({blend});

  // The grid lies on (or very near) the ground plane, so it z-fights with the
  // reference grid and any floor geometry. Depth bias is the QRhi equivalent of
  // the GL pass's glPolygonOffset. It does not write depth: it is a flat overlay,
  // and owning depth would make it reject objects standing on the map.
  pipeline_->setDepthTest(true);
  pipeline_->setDepthWrite(false);
  pipeline_->setDepthBias(-2);
  pipeline_->setSlopeScaledDepthBias(-1.0F);
  pipeline_->setCullMode(QRhiGraphicsPipeline::None);
  pipeline_->setShaderResourceBindings(srb_);
  // Must equal the render target's sample count (see IRhiRenderPass::initialize).
  pipeline_->setSampleCount(sample_count_);
  pipeline_->setRenderPassDescriptor(&rpd);
  if (!pipeline_->create()) {
    qCWarning(lcRhiOcc) << "occupancy pipeline creation failed";
    release();
    return false;
  }

  full_upload_pending_ = true;
  return true;
}

void RhiOccupancyGridPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  if (pipeline_ == nullptr || rhi_ == nullptr || !hasGrid()) {
    return;
  }
  if (!ensureTexture(*rhi_)) {
    return;
  }

  if (full_upload_pending_) {
    QRhiTextureSubresourceUploadDescription sub(cells_.data(), static_cast<int>(cells_.size()));
    sub.setSourceSize(QSize(width_, height_));
    // R8 rows are 1 byte per cell, so the data is already tightly packed; QRhi has
    // no glPixelStorei equivalent and expects exactly that.
    updates.uploadTexture(grid_tex_, QRhiTextureUploadDescription({0, 0, sub}));
    full_upload_pending_ = false;
    dirty_regions_.clear();
  } else if (!dirty_regions_.empty()) {
    for (const QRect& r : dirty_regions_) {
      // A partial upload needs its own tightly-packed staging copy: the source
      // rows for a sub-rectangle are strided by the full grid width, which QRhi's
      // upload description cannot express.
      std::vector<std::uint8_t> patch(static_cast<std::size_t>(r.width()) * static_cast<std::size_t>(r.height()));
      for (int row = 0; row < r.height(); ++row) {
        const std::uint8_t* src = cells_.data() + (static_cast<std::size_t>(r.y() + row) * width_) + r.x();
        std::memcpy(patch.data() + (static_cast<std::size_t>(row) * r.width()), src,
                    static_cast<std::size_t>(r.width()));
      }
      QRhiTextureSubresourceUploadDescription sub(patch.data(), static_cast<int>(patch.size()));
      sub.setSourceSize(r.size());
      sub.setDestinationTopLeft(r.topLeft());
      updates.uploadTexture(grid_tex_, QRhiTextureUploadDescription({0, 0, sub}));
    }
    dirty_regions_.clear();
  }

  const glm::mat4 mvp = ctx.view_proj * model_;
  OccupancyUbo ubo{};
  std::memcpy(ubo.mvp, &mvp[0][0], sizeof(ubo.mvp));
  ubo.opacity = opacity_;
  ubo.color_scheme = static_cast<std::int32_t>(color_scheme_);
  updates.updateDynamicBuffer(ubo_, 0, sizeof(OccupancyUbo), &ubo);
}

void RhiOccupancyGridPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_ == nullptr || !hasGrid() || grid_tex_ == nullptr) {
    return;
  }
  cb.setGraphicsPipeline(pipeline_);
  cb.setShaderResources(srb_);
  cb.draw(6);
}

void RhiOccupancyGridPass::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete sampler_;
  sampler_ = nullptr;
  delete grid_tex_;
  grid_tex_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  tex_width_ = 0;
  tex_height_ = 0;
  full_upload_pending_ = true;
  dirty_regions_.clear();
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
