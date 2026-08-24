#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstdint>
#include <glm/glm.hpp>
#include <vector>

#include "pj_scene3d_widgets/occupancy_grid_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"

namespace pj::scene3d::rhi {

/// Adapts an OccupancyGridLayer's reconstructed map onto RhiOccupancyGridPass.
///
/// Near feature parity, unlike the point-cloud adapter: the QRhi pass has the colour
/// schemes, opacity and — importantly — partial `updateRegion` uploads, so a live map
/// publishing small patches does not re-upload the whole texture. The layer's dirty
/// rectangles are forwarded rather than discarded.
///
/// Visibility is emulated by clearing, since the pass has no visibility flag.
class RhiOccupancyGridSink final : public IOccupancyGridSink {
 public:
  explicit RhiOccupancyGridSink(RhiOccupancyGridPass& pass) : pass_(&pass) {}

  void setGrid(const ReconstructedGrid& grid, bool full_rebuild, const std::vector<CellRect>& dirty_rects) override;
  void clearGrid() override;
  void setColorScheme(OccupancyColorScheme scheme) override;
  void setOpacity(float opacity) override;
  void setVisible(bool visible) override;

  /// Places the map's SOURCE frame into the fixed frame. Outside IOccupancyGridSink
  /// for the same reason as the cloud's: the OpenGL pass resolves this itself per
  /// frame, the QRhi pass cannot, so whoever owns the TF buffer must push it — and
  /// keep pushing it, or a map in a moving frame freezes at its first pose.
  void setFrameTransform(const glm::mat4& fixed_from_source);

 private:
  /// Recompute the pass's model matrix from the retained grid metadata and the
  /// current frame transform. The pass maps the unit square onto the map, so the
  /// matrix is frame * origin-pose * scale(metric extent).
  void pushModelMatrix();
  /// Re-push the retained cells, or clear when hidden.
  void pushRetained();

  RhiOccupancyGridPass* pass_ = nullptr;
  /// Retained so a visibility toggle or a frame move needs no re-reconstruction.
  std::vector<std::uint8_t> cells_;
  int width_ = 0;
  int height_ = 0;
  double resolution_ = 0.0;
  glm::mat4 origin_{1.0F};
  glm::mat4 fixed_from_source_{1.0F};
  bool visible_ = true;
};

}  // namespace pj::scene3d::rhi
