#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between a voxel-grid LAYER (which decodes and selects
// the display field) and a render pass (which uploads and draws). Free of both
// OpenGL and QRhi, so one layer can drive either renderer — the same split
// pointcloud_sink.h makes for clouds.

#include <cstdint>
#include <glm/glm.hpp>
#include <string>
#include <vector>

#include "pj_base/builtin/frame_transforms.hpp"  // Pose
#include "pj_scene3d_core/voxel_grid_value.h"    // VoxelDrawMode
#include "pj_scene3d_core/voxel_grid_view.h"     // VoxelValueKind
#include "pj_widgets/Colormap.h"

namespace pj::scene3d {

/// One dense lattice ready for upload: where it sits, how big its cells are, and
/// the already-selected display field. The layer has done the field selection and
/// value extraction, so a backend only has to place and colour it.
struct VoxelGridUpload {
  std::string frame_id;
  PJ::sdk::Pose origin;       ///< Lower-front-left corner pose, in `frame_id`.
  glm::vec3 cell_size{1.0f};  ///< Metric voxel size (need not be cubic).
  uint32_t column_count = 0;  ///< x (fastest)
  uint32_t row_count = 0;     ///< y
  uint32_t slice_count = 0;   ///< z
  VoxelValueKind kind = VoxelValueKind::kScalar;
  std::vector<float> scalar;  ///< column*row*slice floats, when kind == kScalar
  std::vector<uint8_t> rgba;  ///< column*row*slice*4 bytes, when kind == kRgba
};

/// Where a VoxelGridLayer sends its selected field and display state.
///
/// Everything above this line — decoding the grid, choosing which field drives the
/// display, extracting its values, the XML state and the config widget — is
/// backend-agnostic. Only the upload is not.
///
/// Render-context lifecycle is deliberately absent, as in IPointCloudSink: the two
/// backends drive frames differently, so it stays with whoever owns the pass.
class IVoxelGridSink {
 public:
  virtual ~IVoxelGridSink() = default;

  IVoxelGridSink(const IVoxelGridSink&) = delete;
  IVoxelGridSink& operator=(const IVoxelGridSink&) = delete;

  virtual void setGrid(VoxelGridUpload upload) = 0;
  virtual void clearGrid() = 0;

  virtual void setDrawMode(VoxelDrawMode mode) = 0;
  virtual void setThreshold(float threshold) = 0;
  virtual void setManualRange(float lo, float hi) = 0;
  virtual void setAutoRange(bool on) = 0;
  virtual void setColormap(PJ::Colormap colormap) = 0;
  virtual void setOpacity(float opacity) = 0;
  virtual void setVisible(bool visible) = 0;

 protected:
  IVoxelGridSink() = default;
};

}  // namespace pj::scene3d
