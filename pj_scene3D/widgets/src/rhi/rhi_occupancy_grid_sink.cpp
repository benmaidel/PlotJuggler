// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_sink.h"

#include <QRect>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>

namespace pj::scene3d::rhi {
namespace {

glm::mat4 poseMatrix(const PJ::sdk::Pose& pose) {
  const glm::quat rotation(
      static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  glm::mat4 out = glm::translate(
      glm::mat4(1.0F), glm::vec3(
                           static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
                           static_cast<float>(pose.position.z)));
  return out * glm::mat4_cast(rotation);
}

}  // namespace

void RhiOccupancyGridSink::setGrid(
    const ReconstructedGrid& grid, bool full_rebuild, const std::vector<CellRect>& dirty_rects) {
  if (pass_ == nullptr) {
    return;
  }
  if (grid.empty() || grid.width == 0 || grid.height == 0) {
    clearGrid();
    return;
  }

  const int width = static_cast<int>(grid.width);
  const int height = static_cast<int>(grid.height);
  const bool geometry_changed = width != width_ || height != height_;

  // ROS cells are int8: 0..100 occupancy and -1 for unknown. The pass wants uint8
  // with 255 for unknown, and -1 reinterpreted as uint8 IS 255, so the byte pattern
  // already matches and no per-cell mapping is needed.
  const auto* bytes = reinterpret_cast<const std::uint8_t*>(grid.cells.data());

  width_ = width;
  height_ = height;
  resolution_ = grid.resolution;
  origin_ = poseMatrix(grid.origin);
  cells_.assign(bytes, bytes + grid.cells.size());
  pushModelMatrix();

  if (!visible_) {
    return;  // pushRetained's clear already stands
  }
  // Forward the incremental path when the layer says so: a live map publishes small
  // patches, and re-uploading the whole texture for each would defeat the point of
  // the reconstructor tracking dirty rectangles at all.
  if (full_rebuild || geometry_changed || dirty_rects.empty()) {
    pass_->setGrid(cells_.data(), width_, height_);
    return;
  }
  std::vector<std::uint8_t> patch;
  for (const CellRect& rect : dirty_rects) {
    if (rect.width == 0 || rect.height == 0) {
      continue;
    }
    // The pass takes a tightly-packed patch, so the sub-rectangle has to be copied
    // out row by row rather than handed over as a strided view.
    patch.resize(static_cast<std::size_t>(rect.width) * rect.height);
    for (std::uint32_t row = 0; row < rect.height; ++row) {
      const std::size_t src = ((static_cast<std::size_t>(rect.y) + row) * static_cast<std::size_t>(width_)) + rect.x;
      std::memcpy(patch.data() + (static_cast<std::size_t>(row) * rect.width), cells_.data() + src, rect.width);
    }
    pass_->updateRegion(
        QRect(
            static_cast<int>(rect.x), static_cast<int>(rect.y), static_cast<int>(rect.width),
            static_cast<int>(rect.height)),
        patch.data());
  }
}

void RhiOccupancyGridSink::pushModelMatrix() {
  if (pass_ == nullptr) {
    return;
  }
  // The pass maps the UNIT square onto the map, so the metric extent is a scale.
  const auto extent_x = static_cast<float>(resolution_ * width_);
  const auto extent_y = static_cast<float>(resolution_ * height_);
  const glm::mat4 scale = glm::scale(glm::mat4(1.0F), glm::vec3(extent_x, extent_y, 1.0F));
  pass_->setModelMatrix(fixed_from_source_ * origin_ * scale);
}

void RhiOccupancyGridSink::pushRetained() {
  if (pass_ == nullptr) {
    return;
  }
  if (!visible_ || cells_.empty() || width_ == 0 || height_ == 0) {
    pass_->setGrid(nullptr, 0, 0);
    return;
  }
  pass_->setGrid(cells_.data(), width_, height_);
}

void RhiOccupancyGridSink::clearGrid() {
  cells_.clear();
  width_ = 0;
  height_ = 0;
  if (pass_ != nullptr) {
    pass_->setGrid(nullptr, 0, 0);
  }
}

void RhiOccupancyGridSink::setColorScheme(OccupancyColorScheme scheme) {
  if (pass_ != nullptr) {
    pass_->setColorScheme(
        scheme == OccupancyColorScheme::kCostmap ? RhiOccupancyGridPass::ColorScheme::kCostmap
                                                 : RhiOccupancyGridPass::ColorScheme::kMap);
  }
}

void RhiOccupancyGridSink::setOpacity(float opacity) {
  if (pass_ != nullptr) {
    pass_->setOpacity(opacity);
  }
}

void RhiOccupancyGridSink::setVisible(bool visible) {
  if (visible_ == visible) {
    return;
  }
  visible_ = visible;
  pushRetained();
}

void RhiOccupancyGridSink::setFrameTransform(const glm::mat4& fixed_from_source) {
  fixed_from_source_ = fixed_from_source;
  pushModelMatrix();
}

}  // namespace pj::scene3d::rhi
