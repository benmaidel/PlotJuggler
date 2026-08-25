#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <glm/glm.hpp>
#include <utility>
#include <vector>

#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"
#include "pj_scene3d_widgets/voxel_grid_sink.h"

namespace pj::scene3d::rhi {

/// The predicate bounds to hand RhiVoxelGridPass, given the layer's knobs.
///
/// This exists as a free function because the two backends ENCODE the draw predicate
/// differently, and getting the translation wrong is invisible: the OpenGL shader
/// takes the kRange band from (u_range_lo, u_range_hi) — which the pass feeds from the
/// MANUAL range — while the QRhi shader takes it from (threshold, range_hi), reusing
/// its threshold slot as the band's lower edge. Mapping threshold->threshold in kRange
/// mode would therefore silently band on the wrong value, and still render a plausible
/// subset of voxels. Pure and separately tested for exactly that reason.
///
/// Returns (threshold, range_hi) in the QRhi pass's terms.
[[nodiscard]] std::pair<float, float> voxelPredicateBounds(
    VoxelDrawMode mode, float threshold, float manual_lo, float manual_hi);

/// Adapts a VoxelGridLayer's packed grid onto RhiVoxelGridPass.
///
/// The narrowest-parity adapter of the five, with two gaps worth knowing because the
/// layer's UI offers both knobs regardless:
///
/// - **Opacity is ignored.** The QRhi pass draws voxels opaque; it has no opacity
///   uniform. The OpenGL pass does.
/// - **kRgba grids are refused, not approximated.** RhiVoxelGridPass::setField takes
///   floats only. Collapsing RGBA to a scalar would render something plausible and
///   wrong, so the adapter clears instead and the view stays honestly empty.
///
/// Unlike every other layer, this one cannot be checked in the running app at all:
/// no ROS message decodes to a VoxelGrid, so the fixture cannot carry one. Its
/// placement is therefore pinned by an offscreen render assertion instead — see
/// VoxelGridHonoursItsModelMatrix in rhi_passes_test.
class RhiVoxelGridSink final : public IVoxelGridSink {
 public:
  explicit RhiVoxelGridSink(RhiVoxelGridPass& pass) : pass_(&pass) {}

  void setGrid(VoxelGridUpload upload) override;
  void clearGrid() override;

  void setDrawMode(VoxelDrawMode mode) override;
  void setThreshold(float threshold) override;
  void setManualRange(float lo, float hi) override;
  void setAutoRange(bool on) override;
  void setColormap(PJ::Colormap colormap) override;
  void setOpacity(float opacity) override;
  void setVisible(bool visible) override;

  /// Places the grid's SOURCE frame into the fixed frame. Outside IVoxelGridSink for
  /// the same reason as the cloud's and the map's: the OpenGL pass resolves this per
  /// frame from the FrameContext, the QRhi pass cannot, so whoever owns the TF buffer
  /// must push it — and keep pushing it, or a grid in a moving frame freezes at its
  /// first pose.
  void setFrameTransform(const glm::mat4& fixed_from_source);

 private:
  /// model = frame * grid-origin pose. Cell size stays a separate uniform (the
  /// shader multiplies voxel indices by it), so it is NOT baked in here.
  void pushModelMatrix();
  /// Re-push the retained field, or clear when hidden / unsupported.
  void pushRetained();
  /// Push the predicate + colour range, which both depend on retained knobs and (for
  /// an auto colour range) on the field itself.
  void pushRanges();

  RhiVoxelGridPass* pass_ = nullptr;

  /// Retained so a visibility toggle, a knob change or a frame move needs no
  /// re-decode from the layer.
  std::vector<float> scalar_;
  int columns_ = 0;
  int rows_ = 0;
  int slices_ = 0;
  bool has_grid_ = false;
  /// True when the last setGrid carried an RGBA field, which this pass cannot draw.
  bool unsupported_kind_ = false;

  glm::vec3 cell_size_{1.0F};
  glm::mat4 origin_{1.0F};
  glm::mat4 fixed_from_source_{1.0F};

  /// Min/max over scalar_, recomputed on upload — the auto colour range, matching
  /// what the OpenGL pass derives at the same moment.
  float auto_lo_ = 0.0F;
  float auto_hi_ = 1.0F;

  VoxelDrawMode draw_mode_ = VoxelDrawMode::kNonZero;
  float threshold_ = 0.0F;
  float manual_lo_ = 0.0F;
  float manual_hi_ = 1.0F;
  bool auto_range_ = true;
  bool visible_ = true;
};

}  // namespace pj::scene3d::rhi
