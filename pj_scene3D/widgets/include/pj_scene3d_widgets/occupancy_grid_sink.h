#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between an occupancy-grid LAYER (which reconstructs the
// map from a base grid plus incremental patches) and a render pass (which uploads
// and draws).

#include <vector>

#include "pj_scene3d_core/occupancy_grid_reconstructor.h"  // ReconstructedGrid, CellRect

namespace pj::scene3d {

/// Colour scheme for occupancy values.
///   kMap     — the classic white/black/grey ROS map look.
///   kCostmap — the inflated-cost gradient.
enum class OccupancyColorScheme { kMap, kCostmap };

/// Where an OccupancyGridLayer sends its reconstructed map and display state.
///
/// The reconstruction — time-travel over a base grid plus incremental
/// OccupancyGridUpdate patches — is entirely backend-agnostic and stays in the layer
/// and core. Only the upload sits behind this.
///
/// `setGrid` deliberately carries the dirty rectangles alongside the full grid: a
/// live map publishes small patches, and a backend that can upload just those
/// regions should not have to diff the grid to discover them. A backend without
/// partial uploads may ignore them and re-upload wholesale.
class IOccupancyGridSink {
 public:
  virtual ~IOccupancyGridSink() = default;

  IOccupancyGridSink(const IOccupancyGridSink&) = delete;
  IOccupancyGridSink& operator=(const IOccupancyGridSink&) = delete;

  virtual void setGrid(const ReconstructedGrid& grid, bool full_rebuild, const std::vector<CellRect>& dirty_rects) = 0;
  virtual void clearGrid() = 0;

  virtual void setColorScheme(OccupancyColorScheme scheme) = 0;
  virtual void setOpacity(float opacity) = 0;
  virtual void setVisible(bool visible) = 0;

 protected:
  IOccupancyGridSink() = default;
};

}  // namespace pj::scene3d
