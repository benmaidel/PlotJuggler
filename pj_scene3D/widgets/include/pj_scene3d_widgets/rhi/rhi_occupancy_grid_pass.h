#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <QRect>
#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Occupancy grid / costmap: one textured quad in the grid's own xy-plane.
///
/// The cell data lives in an R8 texture using the canonical OccupancyGrid byte
/// encoding (0..100 = occupancy percent, 255 = unknown), so a grid can be uploaded
/// without expanding it to RGBA on the CPU. Unknown cells are discarded in the
/// shader rather than drawn dark, which is what lets a partially-explored map show
/// the scene through its gaps.
///
/// Supports PARTIAL updates: incremental map patches re-upload only their own
/// rectangle instead of the whole grid, matching how OccupancyGridUpdate messages
/// arrive.
class RhiOccupancyGridPass final : public IRhiRenderPass {
 public:
  enum class ColorScheme { kMap = 0, kCostmap = 1 };

  RhiOccupancyGridPass() = default;
  ~RhiOccupancyGridPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Replace the whole grid. `cells` is row-major, `width * height` bytes.
  /// Re-creates the GPU texture when the dimensions change.
  void setGrid(const std::uint8_t* cells, int width, int height);

  /// Apply an incremental patch: `patch` holds `region.width() * region.height()`
  /// row-major cells that replace that rectangle, and only that rectangle is
  /// re-uploaded to the GPU. This is the shape OccupancyGridUpdate messages arrive
  /// in, and the reason the pass keeps its own CPU copy — the caller supplies only
  /// the delta, never the whole map again.
  ///
  /// Ignored when no grid is set, or when the region does not lie inside it: a
  /// partially-clipped patch would silently misalign the remaining rows against
  /// their source, which is worse than dropping it.
  void updateRegion(const QRect& region, const std::uint8_t* patch);

  /// World transform placing the unit quad: the grid's frame pose combined with
  /// its extent (resolution * cell counts). Cheap to set every frame.
  void setModelMatrix(const glm::mat4& model) { model_ = model; }

  void setColorScheme(ColorScheme scheme) { color_scheme_ = scheme; }
  void setOpacity(float opacity) { opacity_ = opacity; }

  [[nodiscard]] bool hasGrid() const { return width_ > 0 && height_ > 0; }

 private:
  /// Mirrors the OccupancyUbo block in shaders/occupancy.{vert,frag}. std140 puts
  /// the mat4 at 0 and the scalars at 64..79, so the block is 80 bytes.
  struct alignas(16) OccupancyUbo {
    float mvp[16];
    float opacity;
    std::int32_t color_scheme;
    float pad0;
    float pad1;
  };
  static_assert(sizeof(OccupancyUbo) == 80, "OccupancyUbo must match the std140 block layout");

  /// (Re)create the cell texture and rebuild the bindings for it.
  bool ensureTexture(QRhi& rhi);

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* ubo_ = nullptr;
  QRhiTexture* grid_tex_ = nullptr;
  QRhiSampler* sampler_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  /// CPU copy, kept so the texture can be rebuilt after a device loss without the
  /// caller re-supplying the grid.
  std::vector<std::uint8_t> cells_;
  int width_ = 0;
  int height_ = 0;
  /// Texture dimensions currently allocated; a mismatch forces a re-create.
  int tex_width_ = 0;
  int tex_height_ = 0;

  glm::mat4 model_{1.0F};
  ColorScheme color_scheme_ = ColorScheme::kMap;
  float opacity_ = 0.85F;

  bool full_upload_pending_ = true;
  /// Rectangles awaiting re-upload, coalesced only by insertion order.
  std::vector<QRect> dirty_regions_;
};

}  // namespace pj::scene3d::rhi
