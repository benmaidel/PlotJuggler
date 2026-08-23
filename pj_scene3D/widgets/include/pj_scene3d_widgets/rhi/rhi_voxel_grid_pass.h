#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

#include "pj_widgets/Colormap.h"
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// Dense voxel grid, drawn as GPU-instanced cubes.
///
/// The dense-to-cubes expansion happens entirely on the GPU: the selected field is
/// uploaded once as a 3D texture and ONE instanced draw covers
/// `columns * rows * slices` cubes. The vertex shader derives each voxel from
/// gl_InstanceIndex, texelFetches its value, applies the viewer-side draw
/// predicate, and degenerate-clips the cubes it rejects. So the CPU and draw-call
/// cost are independent of voxel count, and re-scrubbing to a cached grid
/// re-uploads nothing.
///
/// This pass carries the QRhi capabilities nothing before it needed: a 3D texture
/// (`QRhi::ThreeDimensionalTextures`) and texelFetch inside a VERTEX shader.
///
/// Scalar fields only for now (R32F). The GL renderer also accepts a direct RGBA
/// field; that is a second texture format and colour path, deliberately left for a
/// follow-up rather than bundled in here.
class RhiVoxelGridPass final : public IRhiRenderPass {
 public:
  /// Which voxels to draw. The schema does not encode this — it is a viewer-side
  /// decision, which is why the predicate lives in the shader.
  enum class DrawMode { kAll = 0, kNonZero = 1, kAtOrAbove = 2, kInRange = 3 };

  RhiVoxelGridPass() = default;
  ~RhiVoxelGridPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Replace the field. `values` is `columns * rows * slices` floats in x-fastest
  /// then y then z order (matching the lattice indexing the shader recovers).
  /// Re-creates the 3D texture when the dimensions change.
  void setField(const float* values, int columns, int rows, int slices);

  /// Metric size of one voxel, and the grid-local -> world transform.
  void setCellSize(const glm::vec3& metres) { cell_size_ = metres; }
  void setModelMatrix(const glm::mat4& model) { model_ = model; }

  void setDrawMode(DrawMode mode) { draw_mode_ = mode; }
  /// Lower bound for kAtOrAbove / kInRange, and the upper bound for kInRange.
  void setThreshold(float threshold) { threshold_ = threshold; }
  void setRangeHigh(float value) { range_hi_ = value; }
  /// Value range mapped across the colormap.
  void setColorRange(float lo, float hi);
  void setColormap(PJ::Colormap colormap) { colormap_ = colormap; }

  [[nodiscard]] int voxelCount() const { return columns_ * rows_ * slices_; }

 private:
  /// Mirrors the VoxelUbo block in shaders/voxel.{vert,frag}. std140: two mat4s
  /// (0, 64), vec4 at 128, ivec4 at 144, then the scalars from 160; padded to 192.
  struct alignas(16) VoxelUbo {
    float view_proj[16];
    float model[16];
    float cell_size[4];
    std::int32_t dims[4];
    std::int32_t draw_mode;
    float threshold;
    float range_hi;
    float color_lo;
    float color_hi;
    float colormap_row;
    float pad0;
    float pad1;
  };
  static_assert(sizeof(VoxelUbo) == 192, "VoxelUbo must match the std140 block layout");

  bool ensureVolumeTexture(QRhi& rhi);

  QRhi* rhi_ = nullptr;
  /// Sample count the current pipeline was built for; a change forces a rebuild.
  int sample_count_ = 1;
  QRhiBuffer* cube_vbo_ = nullptr;
  QRhiBuffer* cube_ibo_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  QRhiTexture* volume_tex_ = nullptr;
  QRhiTexture* colormap_tex_ = nullptr;
  QRhiSampler* volume_sampler_ = nullptr;
  QRhiSampler* colormap_sampler_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;

  /// CPU copy so the texture can be rebuilt after a device loss.
  std::vector<float> values_;
  int columns_ = 0;
  int rows_ = 0;
  int slices_ = 0;
  /// Texture dimensions currently allocated; a mismatch forces a re-create.
  int tex_columns_ = 0;
  int tex_rows_ = 0;
  int tex_slices_ = 0;

  glm::vec3 cell_size_{0.1F};
  glm::mat4 model_{1.0F};
  DrawMode draw_mode_ = DrawMode::kNonZero;
  float threshold_ = 0.5F;
  float range_hi_ = 1.0F;
  float color_lo_ = 0.0F;
  float color_hi_ = 1.0F;
  PJ::Colormap colormap_ = PJ::Colormap::kTurbo;

  bool cube_uploaded_ = false;
  bool colormap_uploaded_ = false;
  bool field_dirty_ = true;
};

}  // namespace pj::scene3d::rhi
