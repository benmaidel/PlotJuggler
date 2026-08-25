// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_sink.h"

#include <gtest/gtest.h>

#include "pj_scene3d_core/voxel_grid_value.h"

namespace {

using pj::scene3d::VoxelDrawMode;
using pj::scene3d::rhi::voxelPredicateBounds;

// The two backends encode the kRange band differently and nothing at the type level
// stops the wrong translation: the OpenGL shader bands on (u_range_lo, u_range_hi),
// which the pass feeds from the MANUAL range, while the QRhi shader bands on
// (threshold, range_hi) — reusing its threshold slot as the lower edge. Mapping
// threshold->threshold would still draw a plausible subset of voxels, so only an
// explicit assertion catches it.
TEST(RhiVoxelGridSinkTest, RangeModeBandsOnTheManualRangeNotTheThreshold) {
  const auto [lo, hi] = voxelPredicateBounds(
      VoxelDrawMode::kRange, /*threshold=*/0.9F, /*manual_lo=*/0.2F,
      /*manual_hi=*/0.6F);
  EXPECT_FLOAT_EQ(lo, 0.2F) << "kRange took its lower edge from the threshold knob, not the manual range";
  EXPECT_FLOAT_EQ(hi, 0.6F);
}

// In every other mode the threshold knob is the threshold, and range_hi is unread by
// the shader — so it must not be allowed to shadow the threshold.
TEST(RhiVoxelGridSinkTest, ThresholdModeUsesTheThresholdKnob) {
  const auto [lo, hi] = voxelPredicateBounds(
      VoxelDrawMode::kThreshold, /*threshold=*/0.9F, /*manual_lo=*/0.2F,
      /*manual_hi=*/0.6F);
  EXPECT_FLOAT_EQ(lo, 0.9F);
  EXPECT_FLOAT_EQ(hi, 0.6F) << "range_hi should pass through untouched (the shader ignores it here)";
}

TEST(RhiVoxelGridSinkTest, AllAndNonZeroModesPassTheThresholdThrough) {
  for (const VoxelDrawMode mode : {VoxelDrawMode::kAll, VoxelDrawMode::kNonZero}) {
    const auto [lo, hi] = voxelPredicateBounds(mode, /*threshold=*/0.4F, /*manual_lo=*/0.1F, /*manual_hi=*/0.8F);
    EXPECT_FLOAT_EQ(lo, 0.4F);
    EXPECT_FLOAT_EQ(hi, 0.8F);
  }
}

// The draw-mode enums are separate types in separate headers, cast across with a
// static_cast in the adapter. They agree today; if either is ever reordered the cast
// silently starts selecting the wrong predicate.
TEST(RhiVoxelGridSinkTest, DrawModeEnumeratorsAgreeAcrossBackends) {
  using RhiMode = pj::scene3d::rhi::RhiVoxelGridPass::DrawMode;
  EXPECT_EQ(static_cast<int>(VoxelDrawMode::kAll), static_cast<int>(RhiMode::kAll));
  EXPECT_EQ(static_cast<int>(VoxelDrawMode::kNonZero), static_cast<int>(RhiMode::kNonZero));
  EXPECT_EQ(static_cast<int>(VoxelDrawMode::kThreshold), static_cast<int>(RhiMode::kAtOrAbove));
  EXPECT_EQ(static_cast<int>(VoxelDrawMode::kRange), static_cast<int>(RhiMode::kInRange));
}

}  // namespace
