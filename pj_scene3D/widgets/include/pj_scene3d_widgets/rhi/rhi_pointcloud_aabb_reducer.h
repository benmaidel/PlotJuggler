#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <rhi/qrhi.h>

#include <optional>

#include "pj_scene3d_core/camera/camera.h"  // AABB

namespace pj::scene3d::rhi {

/// Async GPU reduction of a point cloud's geometric AABB: the QRhi counterpart of
/// PointcloudAabbReducer. Keeps the per-sample bounds scan off the GUI thread, which
/// is the whole point — a large cloud's CPU scan is the cost this removes.
///
/// The contract is deliberately the same as the OpenGL one, because PointCloudLayer
/// depends on it: the CPU seeds the first sample's bounds and keeps scanning until
/// available() confirms the compute path actually works, and probed() distinguishes
/// "not tried yet" from "tried, unsupported". A backend with no compute (a GL 4.0/4.1
/// context, notably) leaves available() false forever and the layer simply never
/// stops scanning.
///
/// Results arrive a FRAME LATER, not within the dispatching frame: the readback is
/// queued behind the compute pass and completes when the frame does. poll() returns
/// nullopt until then.
class RhiPointcloudAabbReducer {
 public:
  RhiPointcloudAabbReducer() = default;
  ~RhiPointcloudAabbReducer();

  RhiPointcloudAabbReducer(const RhiPointcloudAabbReducer&) = delete;
  RhiPointcloudAabbReducer& operator=(const RhiPointcloudAabbReducer&) = delete;

  /// Build the pipeline and buffers. Returns false — and leaves available() false —
  /// when the backend reports no compute support, which is not an error.
  [[nodiscard]] bool ensure(QRhi& rhi);
  void release();

  /// True once the compute pipeline exists, so the caller may drop its CPU scan.
  [[nodiscard]] bool available() const {
    return pipeline_ != nullptr;
  }
  /// True once ensure() has been attempted, making available() authoritative.
  [[nodiscard]] bool probed() const {
    return probed_;
  }

  /// Reduce `point_count` records from `source`, which MUST have been created with
  /// QRhiBuffer::StorageBuffer usage. Runs its own compute pass on `cb` and queues
  /// the readback behind it, so the caller only supplies the command buffer. A
  /// dispatch while one is already outstanding is dropped rather than queued: the
  /// newest cloud will be reduced next frame anyway, and overlapping readbacks into
  /// one result buffer would race.
  void dispatch(
      QRhi& rhi, QRhiCommandBuffer& cb, QRhiBuffer* source, int point_count, int stride_bytes, int xyz_offset_bytes);

  /// The completed reduction, or nullopt while none is ready. An AABB with valid
  /// false means the cloud had no finite point — the same signal the OpenGL reducer
  /// gives, and distinct from "not ready".
  [[nodiscard]] std::optional<AABB> poll();

 private:
  /// 6 ordered-key extents + 1 finite counter, as std430 uints.
  static constexpr int kResultWords = 7;

  QRhi* rhi_ = nullptr;
  QRhiComputePipeline* pipeline_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiBuffer* result_ = nullptr;
  QRhiBuffer* params_ = nullptr;
  /// Rebuilt whenever the source buffer changes identity, because the binding names
  /// it directly — the same rule the mesh pass's material bindings follow.
  QRhiBuffer* bound_source_ = nullptr;
  QRhiReadbackResult readback_;
  bool pending_ = false;
  bool ready_ = false;
  bool probed_ = false;
};

}  // namespace pj::scene3d::rhi
