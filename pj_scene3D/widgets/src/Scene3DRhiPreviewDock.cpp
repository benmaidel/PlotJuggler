// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/Scene3DRhiPreviewDock.h"

#include <QLabel>
#include <QLoggingCategory>
#include <QVBoxLayout>
#include <algorithm>
#include <cmath>
#include <limits>
#include <optional>
#include <tuple>
#include <vector>

#include "pj_datastore/object_store.hpp"
#include "pj_runtime/CatalogModel.h"
#include "pj_runtime/SessionManager.h"
#include "pj_scene3d_core/camera/camera.h"
#include "pj_scene3d_core/tf/tf_buffer.h"
#include "pj_scene3d_widgets/layers/occupancy_grid_layer.h"
#include "pj_scene3d_widgets/layers/pointcloud_layer.h"
#include "pj_scene3d_widgets/render_pass.h"  // FrameContext
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_sink.h"
#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"
#include "pj_scene3d_widgets/transform_service.h"

namespace pj::scene3d {
namespace {

Q_LOGGING_CATEGORY(lcRhiPreview, "pj.scene3d.rhi.preview")

/// Tracker seconds -> absolute nanoseconds. The host hands us display-axis seconds;
/// no production code sets a nonzero DisplayOffset today, so this is the same
/// conversion Scene3DDockWidget performs (see IDataWidget::onTrackerTime).
std::int64_t toNanoseconds(double seconds) {
  return static_cast<std::int64_t>(std::llround(seconds * 1.0e9));
}

}  // namespace

bool Scene3DRhiPreviewDock::enabled() {
  return !qEnvironmentVariableIsEmpty("PJ_SCENE3D_RHI");
}

Scene3DRhiPreviewDock::Scene3DRhiPreviewDock(QWidget* parent) : QWidget(parent) {
  auto* layout = new QVBoxLayout(this);
  layout->setContentsMargins(0, 0, 0, 0);
  layout->setSpacing(0);

  // Labelled so nobody mistakes an intentionally near-empty view for a broken 3D
  // dock. This dock draws TF and the grid only.
  auto* banner = new QLabel(tr("QRhi preview — TF + grid only (PJ_SCENE3D_RHI)"), this);
  banner->setAlignment(Qt::AlignCenter);
  banner->setContentsMargins(4, 2, 4, 2);
  layout->addWidget(banner);

  view_ = new rhi::RhiSceneViewWidget(this);
  view_->setSizePolicy(QSizePolicy::Expanding, QSizePolicy::Expanding);
  layout->addWidget(view_, 1);
}

Scene3DRhiPreviewDock::~Scene3DRhiPreviewDock() {
  // Order matters: the layer holds a raw pointer to the sink.
  if (cloud_layer_ != nullptr) {
    cloud_layer_->setSink(nullptr);
    cloud_layer_->detach();
  }
  cloud_layer_.reset();
  cloud_sink_.reset();
  if (map_layer_ != nullptr) {
    map_layer_->setSink(nullptr);
    map_layer_->detach();
  }
  map_layer_.reset();
  map_sink_.reset();
}

void Scene3DRhiPreviewDock::setTransformService(TransformService* service) {
  transform_service_ = service;
  if (transform_service_ == nullptr) {
    return;
  }
  // Covers a dataset whose transforms land after this dock exists.
  //
  // The connection handle is kept and dropped explicitly rather than relying on
  // Qt::UniqueConnection: that flag only works with a pointer-to-member slot, and
  // Qt silently ignores it (with a runtime warning) for a lambda — so a second
  // setTransformService call would double-connect and bind twice per signal.
  if (tf_ready_conn_) {
    QObject::disconnect(tf_ready_conn_);
  }
  tf_ready_conn_ = connect(
      transform_service_, &TransformService::datasetTransformsReady, this,
      [this](PJ::DatasetId dataset_id) { bindDataset(dataset_id); });
  tryAdoptExistingDataset();
}

void Scene3DRhiPreviewDock::setSessionManager(PJ::SessionManager* session) {
  session_ = session;
  tryAdoptExistingDataset();
}

void Scene3DRhiPreviewDock::tryAdoptExistingDataset() {
  if (tf_buffer_ != nullptr || session_ == nullptr || transform_service_ == nullptr) {
    return;
  }
  // On layout restore the file is loaded — and its /tf ingested — before this dock
  // is built, so the datasetTransformsReady signal has already come and gone. Any
  // existing object topic identifies a dataset to bind, and a preview that renders
  // TF only has no reason to prefer one over another.
  const std::vector<PJ::ObjectTopicId> topics = session_->objectStore().listTopics();
  if (topics.empty()) {
    return;
  }
  bindDataset(session_->objectStore().descriptor(topics.front()).dataset_id);

  // Auto-adopt the first cloud and the first map. A drop is the normal route, but
  // layout restore never drops anything, so without this the preview could only show
  // them interactively and the headless harness could not verify them at all.
  for (const PJ::ObjectTopicId& topic : topics) {
    const PJ::ObjectTopicDescriptor descriptor = session_->objectStore().descriptor(topic);
    const PJ::sdk::BuiltinObjectType type = PJ::objectTypeFromMetadata(descriptor.metadata_json);
    const bool is_cloud =
        type == PJ::sdk::BuiltinObjectType::kPointCloud || type == PJ::sdk::BuiltinObjectType::kCompressedPointCloud;
    if (cloud_layer_ == nullptr && is_cloud) {
      adoptPointCloudTopic(topic, type, QString::fromStdString(descriptor.topic_name));
    } else if (map_layer_ == nullptr && type == PJ::sdk::BuiltinObjectType::kOccupancyGrid) {
      adoptOccupancyTopic(topic, QString::fromStdString(descriptor.topic_name));
    }
  }
}

void Scene3DRhiPreviewDock::bindDataset(PJ::DatasetId dataset_id) {
  if (transform_service_ == nullptr) {
    return;
  }
  // Dedupe only the buffer RE-READ, never the refresh. The service re-emits
  // datasetTransformsReady on every ingest flush, and the flush that actually fills
  // an initially-empty buffer is usually a LATER one for the same dataset — so
  // returning early here would discard exactly the notification that has something
  // new to show, leaving the view permanently empty unless the tracker happened to
  // tick afterwards.
  const bool already_bound = bound_dataset_.has_value() && *bound_dataset_ == dataset_id && tf_buffer_ != nullptr;
  if (!already_bound) {
    qCDebug(lcRhiPreview) << "binding TF from dataset" << dataset_id;
    bound_dataset_ = dataset_id;
    tf_buffer_ = transform_service_->transformBuffer(dataset_id);
  }
  chooseFixedFrame();
  refreshTf();
}

bool Scene3DRhiPreviewDock::tryAcceptObjectTopic(
    PJ::ObjectTopicId topic_id, PJ::sdk::BuiltinObjectType object_type, const QString& title) {
  // Bind TF first: the layer's attach() needs the dataset's transform buffer.
  if (session_ != nullptr) {
    bindDataset(session_->objectStore().descriptor(topic_id).dataset_id);
  }
  if (object_type == PJ::sdk::BuiltinObjectType::kPointCloud ||
      object_type == PJ::sdk::BuiltinObjectType::kCompressedPointCloud) {
    adoptPointCloudTopic(topic_id, object_type, title);
  } else if (object_type == PJ::sdk::BuiltinObjectType::kOccupancyGrid) {
    adoptOccupancyTopic(topic_id, title);
  }
  // Other types are accepted anyway, for the dataset id the drop revealed. Keeping
  // the dock is better than having the host replace a working preview because it
  // could not render one topic.
  return true;
}

void Scene3DRhiPreviewDock::adoptPointCloudTopic(
    PJ::ObjectTopicId topic_id, PJ::sdk::BuiltinObjectType object_type, const QString& title) {
  if (session_ == nullptr || tf_buffer_ == nullptr || view_ == nullptr) {
    return;
  }
  // Detach the previous cloud before its sink dies: the layer holds a raw pointer to
  // the sink, so tearing them down in the wrong order would leave it dangling.
  if (cloud_layer_ != nullptr) {
    cloud_layer_->setSink(nullptr);
    cloud_layer_->detach();
    cloud_layer_.reset();
  }
  cloud_sink_ = std::make_unique<rhi::RhiPointCloudSink>(view_->pointcloudPass());

  cloud_layer_ = std::make_unique<PointCloudLayer>(topic_id, title, object_type);
  // Routed BEFORE attach so the layer's bootstrap decode lands on the QRhi pass
  // rather than on its own (inert) OpenGL one.
  cloud_layer_->setSink(cloud_sink_.get());

  Scene3DLayerContext ctx;
  ctx.session = session_;
  ctx.tf_buffer = tf_buffer_;
  if (!cloud_layer_->attach(ctx)) {
    qCWarning(lcRhiPreview) << "point cloud layer failed to attach for" << title;
    cloud_layer_.reset();
    cloud_sink_.reset();
    return;
  }
  cloud_layer_->setFixedFrame(QString::fromStdString(fixed_frame_));
  framed_ = false;  // re-frame now that there is a cloud extent to include
  qCInfo(lcRhiPreview) << "showing point cloud topic" << title;
  view_->update();
}

void Scene3DRhiPreviewDock::chooseFixedFrame() {
  if (tf_buffer_ == nullptr) {
    fixed_frame_.clear();
    return;
  }
  const std::vector<FrameRow> hierarchy = tf_buffer_->getFrameHierarchy();
  if (hierarchy.empty()) {
    fixed_frame_.clear();
    return;
  }
  for (const FrameRow& row : hierarchy) {
    if (row.depth == 0) {
      fixed_frame_ = row.name;
      return;
    }
  }
  fixed_frame_ = hierarchy.front().name;
}

void Scene3DRhiPreviewDock::onTrackerTime(double time) {
  tracker_ns_ = toNanoseconds(time);
  refreshTf();
  if (cloud_layer_ != nullptr) {
    // The layer defers its decode to the next paint, so this only marks it dirty.
    cloud_layer_->setTrackerTime(PJ::fromRaw(tracker_ns_));
  }
  if (map_layer_ != nullptr) {
    map_layer_->setTrackerTime(PJ::fromRaw(tracker_ns_));
  }
}

void Scene3DRhiPreviewDock::refreshTf() {
  if (view_ == nullptr) {
    return;
  }
  // The buffer is routinely bound BEFORE its transforms are ingested, so the frame
  // hierarchy read at bind time is empty. Re-choosing here means a late fill is
  // picked up on the next tracker tick instead of the dock staying blank forever.
  if (tf_buffer_ != nullptr && fixed_frame_.empty()) {
    chooseFixedFrame();
  }
  if (tf_buffer_ == nullptr || fixed_frame_.empty()) {
    view_->axisPass().setFrames({});
    view_->tfConnectionsPass().setSegments({});
    view_->update();
    return;
  }

  const std::vector<std::string> frames = tf_buffer_->getAllFrames();

  // Fall back to the newest TF sample when the host has not moved the tracker.
  // Without this a preview opened on a loaded-but-not-played file resolves every
  // frame at t=0, which is before the recording starts, so nothing renders and the
  // dock looks broken. The real dock solves the same problem with
  // driveVisibleLayersToLiveEdge.
  std::int64_t stamp_ns = tracker_ns_;
  if (stamp_ns == 0) {
    for (const std::string& frame : frames) {
      if (const auto latest = tf_buffer_->getLatestSample(frame); latest.has_value()) {
        stamp_ns = std::max(stamp_ns, PJ::toRaw(*latest));
      }
    }
  }
  const auto stamp = PJ::fromRaw(stamp_ns);

  std::vector<glm::mat4> triads;
  triads.reserve(frames.size());
  // Resolved origins, kept so the parent-connection segments reuse the SAME
  // lookups rather than resolving each edge twice.
  std::vector<std::pair<std::string, glm::vec3>> origins;
  origins.reserve(frames.size());

  for (const std::string& frame : frames) {
    const auto resolved = tf_buffer_->tryLookupTransform(fixed_frame_, frame, stamp);
    if (!resolved.has_value()) {
      continue;  // unresolvable at this time: render the resolvable subset
    }
    const glm::mat4 world(resolved.value().matrix());
    triads.push_back(world);
    origins.emplace_back(frame, glm::vec3(world[3]));
  }

  std::vector<glm::vec3> segments;
  segments.reserve(origins.size() * 2);
  for (const auto& [frame, origin] : origins) {
    const std::optional<std::string> parent = tf_buffer_->getParent(frame);
    if (!parent.has_value()) {
      continue;
    }
    const auto parent_it =
        std::find_if(origins.begin(), origins.end(), [&](const auto& entry) { return entry.first == *parent; });
    if (parent_it == origins.end()) {
      continue;  // parent did not resolve; skip the edge rather than draw to origin
    }
    segments.push_back(parent_it->second);
    segments.push_back(origin);
  }

  // Logged only when the outcome CHANGES: refreshTf runs on every tracker tick, so
  // an unconditional line would bury the log during playback. This is the one place
  // that distinguishes "frames failed to resolve" from "failed to render", which is
  // what you want when the view looks empty.
  const auto outcome = std::tuple(frames.size(), triads.size(), segments.size(), fixed_frame_);
  if (outcome != last_logged_outcome_) {
    last_logged_outcome_ = outcome;
    qCDebug(lcRhiPreview) << "TF refresh: fixed_frame" << QString::fromStdString(fixed_frame_) << "frames"
                          << frames.size() << "resolved" << triads.size() << "edges" << (segments.size() / 2);
  }

  // Pump the layers' deferred decode. setTrackerTime only marks them dirty; without
  // this the OpenGL view would be the only thing able to drive them, and a layer
  // would sit frozen at whatever attach() happened to push.
  const FrameContext frame_ctx{*tf_buffer_, fixed_frame_, stamp};
  if (cloud_layer_ != nullptr) {
    cloud_layer_->advance(frame_ctx);
  }
  if (map_layer_ != nullptr) {
    map_layer_->advance(frame_ctx);
  }

  glm::mat4 cloud_world(1.0F);
  if (cloud_layer_ != nullptr) {
    cloud_layer_->setFixedFrame(QString::fromStdString(fixed_frame_));
    // The QRhi cloud pass has no TF access of its own — the OpenGL pass resolves
    // this from its FrameContext per frame — so the transform has to be pushed from
    // here, every refresh, or a cloud in a moving frame freezes at its first pose.
    const std::string source = cloud_layer_->sourceFrame().toStdString();
    if (!source.empty()) {
      if (const auto resolved = tf_buffer_->tryLookupTransform(fixed_frame_, source, stamp); resolved.has_value()) {
        cloud_world = glm::mat4(resolved.value().matrix());
      }
    }
    if (cloud_sink_ != nullptr) {
      cloud_sink_->setFrameTransform(cloud_world);
    }
  }
  if (map_layer_ != nullptr) {
    map_layer_->setFixedFrame(QString::fromStdString(fixed_frame_));
    const std::string map_source = map_layer_->sourceFrame().toStdString();
    glm::mat4 map_world(1.0F);
    if (!map_source.empty()) {
      if (const auto resolved = tf_buffer_->tryLookupTransform(fixed_frame_, map_source, stamp); resolved.has_value()) {
        map_world = glm::mat4(resolved.value().matrix());
      }
    }
    if (map_sink_ != nullptr) {
      map_sink_->setFrameTransform(map_world);
    }
  }
  frameSceneOnce(triads, cloud_world);
  view_->axisPass().setFrames(std::move(triads));
  view_->tfConnectionsPass().setSegments(std::move(segments));
  view_->update();
}

void Scene3DRhiPreviewDock::adoptOccupancyTopic(PJ::ObjectTopicId topic_id, const QString& title) {
  if (session_ == nullptr || tf_buffer_ == nullptr || view_ == nullptr) {
    return;
  }
  // Tear down in this order: the layer holds a raw pointer to its sink.
  if (map_layer_ != nullptr) {
    map_layer_->setSink(nullptr);
    map_layer_->detach();
    map_layer_.reset();
  }
  map_sink_ = std::make_unique<rhi::RhiOccupancyGridSink>(view_->occupancyGridPass());

  map_layer_ = std::make_unique<OccupancyGridLayer>(topic_id, title);
  // Routed BEFORE attach so the bootstrap decode lands on the QRhi pass rather than
  // on the layer's own (inert) OpenGL one.
  map_layer_->setSink(map_sink_.get());

  Scene3DLayerContext ctx;
  ctx.session = session_;
  ctx.tf_buffer = tf_buffer_;
  if (!map_layer_->attach(ctx)) {
    qCWarning(lcRhiPreview) << "occupancy grid layer failed to attach for" << title;
    map_layer_.reset();
    map_sink_.reset();
    return;
  }
  map_layer_->setFixedFrame(QString::fromStdString(fixed_frame_));
  qCInfo(lcRhiPreview) << "showing occupancy grid topic" << title;
  view_->update();
}

void Scene3DRhiPreviewDock::frameSceneOnce(const std::vector<glm::mat4>& triads, const glm::mat4& cloud_world) {
  if (framed_ || triads.empty() || view_->camera() == nullptr) {
    return;
  }
  framed_ = true;
  glm::vec3 lo(std::numeric_limits<float>::max());
  glm::vec3 hi(std::numeric_limits<float>::lowest());
  for (const glm::mat4& t : triads) {
    const glm::vec3 origin(t[3]);
    lo = glm::min(lo, origin);
    hi = glm::max(hi, origin);
  }
  // Include the cloud's extent. Framing on TF origins alone puts the camera a few
  // metres out, which a metres-wide cloud then fills entirely. Its bounds are in the
  // SOURCE frame, so all eight corners go through the frame transform — transforming
  // just min/max would be wrong under rotation.
  if (cloud_layer_ != nullptr) {
    if (const std::optional<AABB> bounds = cloud_layer_->worldBounds(); bounds.has_value() && bounds->valid) {
      for (int corner = 0; corner < 8; ++corner) {
        const glm::vec3 local(
            (corner & 1) != 0 ? bounds->max.x : bounds->min.x, (corner & 2) != 0 ? bounds->max.y : bounds->min.y,
            (corner & 4) != 0 ? bounds->max.z : bounds->min.z);
        const glm::vec3 world(cloud_world * glm::vec4(local, 1.0F));
        lo = glm::min(lo, world);
        hi = glm::max(hi, world);
      }
    }
  }
  const glm::vec3 centre = (lo + hi) * 0.5F;
  // A floor on the radius so a single frame, or a degenerate tree where every
  // origin coincides, still gets a usable viewing distance instead of radius 0.
  const float extent = glm::length(hi - lo);
  CameraState state;
  state.focal = centre;
  state.radius = std::max(extent * 1.8F, 4.0F);
  state.azimuth = glm::radians(35.0F);
  state.elevation = glm::radians(25.0F);
  state.perspective = true;
  view_->camera()->adoptState(state);
}

}  // namespace pj::scene3d
