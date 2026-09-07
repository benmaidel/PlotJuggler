// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_pointcloud_aabb_reducer.h"

#include <QFile>
#include <QLoggingCategory>
#include <algorithm>
#include <array>
#include <cstring>

namespace pj::scene3d::rhi {

namespace {

Q_LOGGING_CATEGORY(lcRhiAabb, "pj.scene3d.rhi.aabb")

constexpr int kLocalSizeX = 256;
/// Enough groups to saturate a desktop GPU while keeping the atomic traffic bounded
/// (seven atomics per group). Matches the OpenGL reducer's dispatch width.
constexpr int kMaxWorkgroups = 64;

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiAabb) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

/// Inverse of the shader's floatToOrderedKey.
float orderedKeyToFloat(std::uint32_t key) {
  const std::uint32_t mask = (key & 0x80000000u) != 0u ? 0x80000000u : 0xFFFFFFFFu;
  const std::uint32_t bits = key ^ mask;
  float out = 0.0F;
  std::memcpy(&out, &bits, sizeof(out));
  return out;
}

}  // namespace

RhiPointcloudAabbReducer::~RhiPointcloudAabbReducer() {
  release();
}

bool RhiPointcloudAabbReducer::ensure(QRhi& rhi) {
  if (pipeline_ != nullptr && rhi_ == &rhi) {
    return true;
  }
  if (rhi_ != &rhi) {
    release();
  }
  probed_ = true;
  rhi_ = &rhi;

  // Not an error: a GL 4.0/4.1 context has no compute at all, and the layer's CPU
  // scan is the documented answer.
  if (!rhi.isFeatureSupported(QRhi::Compute)) {
    qCDebug(lcRhiAabb) << "no compute support; the CPU bounds scan stays";
    return false;
  }

  const QShader comp = loadBakedShader(QStringLiteral(":/scene3d_shaders/pointcloud_aabb.comp.qsb"));
  if (!comp.isValid()) {
    return false;
  }

  result_ = rhi.newBuffer(QRhiBuffer::Static, QRhiBuffer::StorageBuffer, sizeof(std::uint32_t) * kResultWords);
  params_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(std::uint32_t) * 4);
  if (result_ == nullptr || params_ == nullptr || !result_->create() || !params_->create()) {
    release();
    return false;
  }
  // The pipeline is built here, but the bindings naming the SOURCE buffer cannot be —
  // that buffer belongs to the cloud and changes identity. dispatch() builds them.
  return true;
}

void RhiPointcloudAabbReducer::release() {
  delete pipeline_;
  pipeline_ = nullptr;
  delete srb_;
  srb_ = nullptr;
  delete result_;
  result_ = nullptr;
  delete params_;
  params_ = nullptr;
  bound_source_ = nullptr;
  rhi_ = nullptr;
  pending_ = false;
  ready_ = false;
}

void RhiPointcloudAabbReducer::dispatch(
    QRhi& rhi, QRhiCommandBuffer& cb, QRhiBuffer* source, int point_count, int stride_bytes, int xyz_offset_bytes) {
  if (!ensure(rhi) && pipeline_ == nullptr && !rhi.isFeatureSupported(QRhi::Compute)) {
    return;
  }
  if (result_ == nullptr || source == nullptr || point_count <= 0) {
    return;
  }
  // One outstanding reduction at a time: a second would write the same result buffer
  // while the first is still being read back.
  if (pending_) {
    return;
  }

  using SRB = QRhiShaderResourceBinding;
  if (srb_ == nullptr || bound_source_ != source) {
    delete srb_;
    srb_ = rhi.newShaderResourceBindings();
    srb_->setBindings({
        SRB::bufferLoad(0, SRB::ComputeStage, source),
        SRB::bufferLoadStore(1, SRB::ComputeStage, result_),
        SRB::uniformBuffer(2, SRB::ComputeStage, params_),
    });
    if (!srb_->create()) {
      qCWarning(lcRhiAabb) << "compute bindings creation failed";
      delete srb_;
      srb_ = nullptr;
      return;
    }
    bound_source_ = source;
    // The pipeline is compiled against the binding LAYOUT, so it is built only once
    // the first real set exists; layout is identical for every later source.
    if (pipeline_ == nullptr) {
      const QShader comp = loadBakedShader(QStringLiteral(":/scene3d_shaders/pointcloud_aabb.comp.qsb"));
      if (!comp.isValid()) {
        return;
      }
      pipeline_ = rhi.newComputePipeline();
      pipeline_->setShaderStage({QRhiShaderStage::Compute, comp});
      pipeline_->setShaderResourceBindings(srb_);
      if (!pipeline_->create()) {
        qCWarning(lcRhiAabb) << "compute pipeline creation failed";
        delete pipeline_;
        pipeline_ = nullptr;
        return;
      }
      qCDebug(lcRhiAabb) << "GPU AABB reduction available";
    } else {
      pipeline_->setShaderResourceBindings(srb_);
    }
  }

  // Sentinels: extents start at the ordered-key identities so the shader's atomicMin
  // / atomicMax always lose to a real point, and the finite counter starts at 0.
  const std::array<std::uint32_t, kResultWords> sentinel{0xFFFFFFFFu, 0xFFFFFFFFu, 0xFFFFFFFFu, 0u, 0u, 0u, 0u};
  const int groups = std::min(kMaxWorkgroups, ((point_count + kLocalSizeX) - 1) / kLocalSizeX);
  const std::array<std::uint32_t, 4> params{
      static_cast<std::uint32_t>(point_count), static_cast<std::uint32_t>(stride_bytes) / 4u,
      static_cast<std::uint32_t>(xyz_offset_bytes) / 4u, static_cast<std::uint32_t>(groups * kLocalSizeX)};

  QRhiResourceUpdateBatch* pre = rhi.nextResourceUpdateBatch();
  pre->uploadStaticBuffer(result_, sentinel.data());
  pre->updateDynamicBuffer(params_, 0, sizeof(params), params.data());

  cb.beginComputePass(pre);
  cb.setComputePipeline(pipeline_);
  cb.setShaderResources(srb_);
  cb.dispatch(groups, 1, 1);

  // Queued behind the dispatch and applied at endComputePass, so it observes the
  // reduction's result rather than the sentinels.
  readback_ = {};
  ready_ = false;
  readback_.completed = [this]() { ready_ = true; };
  QRhiResourceUpdateBatch* post = rhi.nextResourceUpdateBatch();
  post->readBackBuffer(result_, 0, sizeof(std::uint32_t) * kResultWords, &readback_);
  cb.endComputePass(post);
  pending_ = true;
}

std::optional<AABB> RhiPointcloudAabbReducer::poll() {
  if (!pending_ || !ready_) {
    return std::nullopt;
  }
  pending_ = false;
  ready_ = false;
  if (readback_.data.size() < static_cast<int>(sizeof(std::uint32_t) * kResultWords)) {
    return std::nullopt;
  }
  std::array<std::uint32_t, kResultWords> words{};
  std::memcpy(words.data(), readback_.data.constData(), sizeof(words));

  AABB out{};
  if (words[6] == 0u) {
    return out;  // no finite point; valid stays false
  }
  out.min = glm::vec3(orderedKeyToFloat(words[0]), orderedKeyToFloat(words[1]), orderedKeyToFloat(words[2]));
  out.max = glm::vec3(orderedKeyToFloat(words[3]), orderedKeyToFloat(words[4]), orderedKeyToFloat(words[5]));
  out.valid = true;
  return out;
}

}  // namespace pj::scene3d::rhi
