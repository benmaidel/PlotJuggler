// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_sink.h"

#include <QLoggingCategory>
#include <algorithm>
#include <glm/gtc/matrix_transform.hpp>
#include <glm/gtc/quaternion.hpp>
#include <utility>

namespace pj::scene3d::rhi {

namespace {

Q_LOGGING_CATEGORY(lcRhiVoxelSink, "pj.scene3d.rhi.voxel_sink")

glm::mat4 poseMatrix(const PJ::sdk::Pose& pose) {
  const glm::quat rotation(
      static_cast<float>(pose.orientation.w), static_cast<float>(pose.orientation.x),
      static_cast<float>(pose.orientation.y), static_cast<float>(pose.orientation.z));
  const glm::mat4 out = glm::translate(
      glm::mat4(1.0F), glm::vec3(
                           static_cast<float>(pose.position.x), static_cast<float>(pose.position.y),
                           static_cast<float>(pose.position.z)));
  return out * glm::mat4_cast(rotation);
}

}  // namespace

std::pair<float, float> voxelPredicateBounds(VoxelDrawMode mode, float threshold, float manual_lo, float manual_hi) {
  // Only kRange consults a band, and the OpenGL shader takes that band from the
  // MANUAL range — so the QRhi pass's threshold slot must carry manual_lo here, not
  // the threshold knob. In every other mode range_hi is unread and threshold means
  // what it says.
  if (mode == VoxelDrawMode::kRange) {
    return {manual_lo, manual_hi};
  }
  return {threshold, manual_hi};
}

void RhiVoxelGridSink::setGrid(VoxelGridUpload upload) {
  columns_ = static_cast<int>(upload.column_count);
  rows_ = static_cast<int>(upload.row_count);
  slices_ = static_cast<int>(upload.slice_count);
  cell_size_ = upload.cell_size;
  origin_ = poseMatrix(upload.origin);
  unsupported_kind_ = upload.kind != VoxelValueKind::kScalar;

  if (unsupported_kind_) {
    // Refused rather than approximated: see the class comment. Warn once per upload
    // so a grid that keeps republishing does not flood the log at frame rate — the
    // layer only calls setGrid on an actual new grid.
    qCWarning(lcRhiVoxelSink) << "VoxelGrid carries an RGBA field; the QRhi voxel pass draws scalars only.";
    scalar_.clear();
    has_grid_ = false;
    pushRetained();
    return;
  }

  scalar_ = std::move(upload.scalar);
  has_grid_ = !scalar_.empty() && columns_ > 0 && rows_ > 0 && slices_ > 0;

  // Matches VoxelGridRenderPass::uploadPending exactly, including the empty-field
  // fallback: an all-equal field would otherwise map to a degenerate colour range.
  const auto [lo, hi] = std::minmax_element(scalar_.begin(), scalar_.end());
  auto_lo_ = scalar_.empty() ? 0.0F : *lo;
  auto_hi_ = scalar_.empty() ? 1.0F : *hi;

  pushModelMatrix();
  pushRetained();
  pushRanges();
}

void RhiVoxelGridSink::clearGrid() {
  scalar_.clear();
  columns_ = 0;
  rows_ = 0;
  slices_ = 0;
  has_grid_ = false;
  unsupported_kind_ = false;
  pass_->setField(nullptr, 0, 0, 0);
}

void RhiVoxelGridSink::setDrawMode(VoxelDrawMode mode) {
  draw_mode_ = mode;
  pass_->setDrawMode(static_cast<RhiVoxelGridPass::DrawMode>(mode));
  pushRanges();
}

void RhiVoxelGridSink::setThreshold(float threshold) {
  threshold_ = threshold;
  pushRanges();
}

void RhiVoxelGridSink::setManualRange(float lo, float hi) {
  manual_lo_ = lo;
  manual_hi_ = hi;
  pushRanges();
}

void RhiVoxelGridSink::setAutoRange(bool on) {
  auto_range_ = on;
  pushRanges();
}

void RhiVoxelGridSink::setColormap(PJ::Colormap colormap) {
  pass_->setColormap(colormap);
}

void RhiVoxelGridSink::setOpacity(float /*opacity*/) {
  // No opacity uniform on the QRhi pass; see the class comment. Deliberately silent
  // rather than warning: the layer pushes this on construction and on every slider
  // tick, so a warning here would be noise, not news.
}

void RhiVoxelGridSink::setVisible(bool visible) {
  if (visible_ == visible) {
    return;
  }
  visible_ = visible;
  pushRetained();
}

void RhiVoxelGridSink::setFrameTransform(const glm::mat4& fixed_from_source) {
  fixed_from_source_ = fixed_from_source;
  pushModelMatrix();
}

void RhiVoxelGridSink::pushModelMatrix() {
  pass_->setModelMatrix(fixed_from_source_ * origin_);
  pass_->setCellSize(cell_size_);
}

void RhiVoxelGridSink::pushRetained() {
  if (!has_grid_ || !visible_) {
    pass_->setField(nullptr, 0, 0, 0);
    return;
  }
  pass_->setField(scalar_.data(), columns_, rows_, slices_);
}

void RhiVoxelGridSink::pushRanges() {
  const auto [predicate_lo, predicate_hi] = voxelPredicateBounds(draw_mode_, threshold_, manual_lo_, manual_hi_);
  pass_->setThreshold(predicate_lo);
  pass_->setRangeHigh(predicate_hi);
  // The colour range follows auto/manual independently of the predicate, matching
  // VoxelGridRenderPass::render's color_lo/color_hi.
  if (auto_range_) {
    pass_->setColorRange(auto_lo_, auto_hi_);
  } else {
    pass_->setColorRange(manual_lo_, manual_hi_);
  }
}

}  // namespace pj::scene3d::rhi
