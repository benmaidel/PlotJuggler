#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstdint>
#include <memory>
#include <optional>
#include <vector>

#include "pj_scene3d_widgets/pointcloud_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"

namespace pj::scene3d::rhi {

/// Adapts a PointCloudLayer's decoded output onto RhiPointcloudPass.
///
/// This is what the IPointCloudSink seam buys: the layer's decode machinery — async
/// Draco/Cloudini decode, sample-identity caching, latest-wins coalescing, colour
/// field discovery, auto-range — is reused verbatim against the QRhi renderer with
/// no duplication.
///
/// **The QRhi pass is not yet at feature parity with the OpenGL one**, and this
/// adapter is where that shows. Supported: geometry (both the interleaved and the
/// verbatim-wire paths), the colormap, its range, point radius, and visibility.
/// Silently ignored, because RhiPointcloudPass has no equivalent: per-point RGB and
/// solid colour (`setColorType`), point shape, pixel sizing, LUT inversion, and the
/// spatial-axis auto-range. A cloud whose colour mode is RGB therefore renders
/// through the colormap instead — acceptable while this drives a developer preview,
/// and the first thing to fix when the QRhi pass is brought up to parity.
class RhiPointCloudSink final : public IPointCloudSink {
 public:
  explicit RhiPointCloudSink(RhiPointcloudPass& pass) : pass_(&pass) {}

  void setActiveCloud(std::shared_ptr<const DecodedPointCloud> cloud) override;
  void setActiveFastCloud(FastCloudData cloud) override;

  void setVisible(bool visible) override;
  void setColormap(PJ::Colormap cm) override;
  void setColormapRange(float min_value, float max_value) override;
  void setSizeMeters(float meters) override;

  // Accepted and ignored — see the class doc for why each has no counterpart.
  void setShape(PointcloudShape /*shape*/) override {}
  void setSizePixels(float /*pixels*/) override {}
  void setColorType(PointcloudColorType /*type*/) override {}
  void setSolidColor(glm::vec3 /*rgb*/) override {}
  void setInvertLut(bool /*invert*/) override {}
  void setScalarAxis(int /*axis*/) override {}
  void setSpatialAutoBounds(std::optional<AABB> /*source_bounds*/) override {}

  /// No GPU reduction here, so both queries stay false forever. That is the
  /// documented contract for a backend without it: the layer keeps its CPU bounds
  /// scan rather than waiting for a result that will never arrive.
  void setGpuAabbEnabled(bool /*enabled*/) override {}
  void setBoundsCallback(std::function<void(std::optional<AABB>)> /*callback*/) override {}
  [[nodiscard]] bool gpuAabbAvailable() const override {
    return false;
  }
  [[nodiscard]] bool gpuAabbProbed() const override {
    return false;
  }

 private:
  /// Re-push the retained geometry, or clear it when hidden. RhiPointcloudPass has
  /// no visibility flag, so hiding is expressed as "zero points".
  void pushRetained();

  RhiPointcloudPass* pass_ = nullptr;
  /// The last geometry pushed, retained so a visibility toggle can restore it
  /// without the layer re-decoding.
  std::vector<std::byte> records_;
  int point_count_ = 0;
  RhiPointcloudPass::Layout layout_;
  bool visible_ = true;
};

}  // namespace pj::scene3d::rhi
