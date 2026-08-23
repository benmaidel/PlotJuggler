#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// The backend-agnostic seam between a point-cloud LAYER (which decodes) and a
// point-cloud RENDER PASS (which uploads and draws). Deliberately free of both
// OpenGL and QRhi so a layer can drive either renderer.

#include <cstddef>
#include <functional>
#include <glm/glm.hpp>
#include <memory>
#include <optional>

#include "pj_base/builtin/point_cloud.hpp"
#include "pj_scene3d_core/camera/camera.h"       // AABB
#include "pj_scene3d_core/pointcloud_convert.h"  // AttribLayout
#include "pj_widgets/Colormap.h"

namespace pj::scene3d {

struct DecodedPointCloud;  // defined in pj_scene3d_core/pointcloud.h

/// Shape mode for each point.
///   kSphere — world-radius sphere imposter; foreshortens with depth under a
///             perspective camera, fixed on-screen size under an orthographic one.
///   kPoint  — flat 1-pixel-fixed sprite (no perspective).
///   kCube   — instanced 3D cube, fixed-frame-axis-aligned.
enum class PointcloudShape { kSphere, kPoint, kCube };

/// Colour sourcing mode.
///   kField — per-point scalar -> colormap -> fragment colour.
///   kSolid — single uniform colour, scalar ignored.
///   kRgb   — per-point packed RGBA used directly (no colormap). Requires the
///            active cloud to carry `DecodedPointCloud::rgba`; falls back to white.
enum class PointcloudColorType { kField, kSolid, kRgb };

/// Fast-path retained state: the verbatim wire cloud (its anchor keeps the bytes
/// alive across render-context recreation) plus the precomputed bind layout. `wire`
/// is held BY VALUE — copying the BufferAnchor — because reducing it to a Span
/// would leave recreation re-uploading a dangling view.
struct FastCloudData {
  PJ::sdk::PointCloud wire;    // frame_id / data / anchor all live here
  std::size_t point_count{0};  // == wire.width * wire.height (size_t: no uint32 overflow)
  AttribLayout layout;
};

/// Where a PointCloudLayer sends its decoded output and display state.
///
/// This exists because layer decode logic — async Draco/Cloudini decode, sample
/// identity caching, latest-wins coalescing, colour-field discovery, auto-range —
/// is entirely backend-agnostic, while only the final upload is not. Splitting them
/// at this line is what lets one layer feed either the OpenGL or the QRhi renderer
/// instead of the decode work being duplicated per backend.
///
/// Render-context lifecycle is deliberately NOT part of this contract: the two
/// backends drive frames differently (OpenGL calls the layer per paint; QRhi splits
/// prepare/draw and is driven by the view), so it stays with whoever owns the pass.
class IPointCloudSink {
 public:
  virtual ~IPointCloudSink() = default;

  IPointCloudSink(const IPointCloudSink&) = delete;
  IPointCloudSink& operator=(const IPointCloudSink&) = delete;

  /// The converted cloud. Shared, because a colour-field change re-pushes the same
  /// decoded cloud rather than re-decoding it.
  virtual void setActiveCloud(std::shared_ptr<const DecodedPointCloud> cloud) = 0;
  /// The zero-copy path: wire bytes uploaded verbatim under `layout`.
  virtual void setActiveFastCloud(FastCloudData cloud) = 0;

  virtual void setVisible(bool visible) = 0;
  virtual void setShape(PointcloudShape shape) = 0;
  virtual void setSizeMeters(float meters) = 0;
  virtual void setSizePixels(float pixels) = 0;
  virtual void setColorType(PointcloudColorType type) = 0;
  virtual void setSolidColor(glm::vec3 rgb) = 0;
  virtual void setColormap(PJ::Colormap cm) = 0;
  virtual void setInvertLut(bool invert) = 0;
  virtual void setColormapRange(float min_value, float max_value) = 0;
  /// -1 for a non-spatial field, else 0/1/2 for x/y/z.
  virtual void setScalarAxis(int axis) = 0;
  /// Source-frame bounds driving the auto colormap range when setScalarAxis picked a
  /// spatial axis; nullopt falls back to the explicit setColormapRange values.
  virtual void setSpatialAutoBounds(std::optional<AABB> source_bounds) = 0;

  /// Optional GPU bounds reduction. A backend without it reports `false` from both
  /// queries forever, which keeps the layer's CPU bounds scan running — so this is
  /// a capability, not a requirement.
  virtual void setGpuAabbEnabled(bool enabled) = 0;
  virtual void setBoundsCallback(std::function<void(std::optional<AABB>)> callback) = 0;
  [[nodiscard]] virtual bool gpuAabbAvailable() const = 0;
  [[nodiscard]] virtual bool gpuAabbProbed() const = 0;

 protected:
  IPointCloudSink() = default;
};

}  // namespace pj::scene3d
