// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_pointcloud_sink.h"

#include <cstring>

#include "pj_scene3d_core/pointcloud.h"

namespace pj::scene3d::rhi {
namespace {

/// One interleaved record the QRhi pass reads by default: xyz then the scalar.
struct InterleavedPoint {
  float x;
  float y;
  float z;
  float scalar;
};
static_assert(sizeof(InterleavedPoint) == 16, "must match RhiPointcloudPass::Layout's default stride");

}  // namespace

void RhiPointCloudSink::setActiveCloud(std::shared_ptr<const DecodedPointCloud> cloud) {
  records_.clear();
  point_count_ = 0;
  layout_ = RhiPointcloudPass::Layout{};

  if (cloud != nullptr && !cloud->positions.empty()) {
    // DecodedPointCloud keeps positions and scalars in separate vectors, so this
    // path has to interleave. The result deliberately matches the default layout,
    // which is also the wire layout the fast path uploads verbatim.
    const std::size_t count = cloud->positions.size();
    records_.resize(count * sizeof(InterleavedPoint));
    auto* out = reinterpret_cast<InterleavedPoint*>(records_.data());
    const bool has_scalar = cloud->scalar.size() == count;
    for (std::size_t i = 0; i < count; ++i) {
      const glm::vec3& p = cloud->positions[i];
      out[i] = InterleavedPoint{p.x, p.y, p.z, has_scalar ? cloud->scalar[i] : 0.0F};
    }
    point_count_ = static_cast<int>(count);
  }
  pushRetained();
}

void RhiPointCloudSink::setActiveFastCloud(FastCloudData cloud) {
  records_.clear();
  point_count_ = 0;
  layout_ = RhiPointcloudPass::Layout{};

  const std::size_t bytes = cloud.wire.data.size();
  if (cloud.point_count > 0 && bytes > 0) {
    // The wire bytes are already interleaved; copy them through unchanged and just
    // describe where the fields sit. A cloud with no scalar field points the scalar
    // at the x offset — it is then a constant per point rather than garbage, which
    // reads as a flat colour instead of noise.
    records_.resize(bytes);
    std::memcpy(records_.data(), cloud.wire.data.data(), bytes);
    point_count_ = static_cast<int>(cloud.point_count);
    layout_.stride_bytes = static_cast<int>(cloud.layout.stride);
    layout_.xyz_offset = static_cast<int>(cloud.layout.xyz_offset);
    layout_.scalar_offset =
        static_cast<int>(cloud.layout.has_scalar ? cloud.layout.scalar_offset : cloud.layout.xyz_offset);
  }
  pushRetained();
}

void RhiPointCloudSink::pushRetained() {
  if (pass_ == nullptr) {
    return;
  }
  if (!visible_ || point_count_ == 0 || records_.empty()) {
    pass_->setPoints(nullptr, 0, layout_);
    return;
  }
  pass_->setPoints(records_.data(), point_count_, layout_);
}

void RhiPointCloudSink::setVisible(bool visible) {
  if (visible_ == visible) {
    return;
  }
  visible_ = visible;
  pushRetained();
}

void RhiPointCloudSink::setFrameTransform(const glm::mat4& fixed_from_source) {
  if (pass_ != nullptr) {
    pass_->setModelMatrix(fixed_from_source);
  }
}

void RhiPointCloudSink::setColormap(PJ::Colormap cm) {
  if (pass_ != nullptr) {
    pass_->setColormap(cm);
  }
}

void RhiPointCloudSink::setColormapRange(float min_value, float max_value) {
  if (pass_ != nullptr) {
    pass_->setScalarRange(min_value, max_value);
  }
}

void RhiPointCloudSink::setSizeMeters(float meters) {
  if (pass_ != nullptr) {
    pass_->setPointRadius(meters);
  }
}

}  // namespace pj::scene3d::rhi
