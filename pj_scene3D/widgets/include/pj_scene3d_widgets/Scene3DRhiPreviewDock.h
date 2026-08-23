#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <QWidget>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <string>
#include <tuple>
#include <vector>

#include "pj_runtime/IDataWidget.h"

namespace PJ {
class SessionManager;
}  // namespace PJ

namespace pj::scene3d {

class TransformBuffer;
class TransformService;

namespace rhi {
class RhiSceneViewWidget;
}  // namespace rhi

/// A DEVELOPER preview that puts the QRhi/Metal scene renderer inside the running
/// app, opt-in via the PJ_SCENE3D_RHI environment variable.
///
/// Why this exists rather than a flag on Scene3DDockWidget: that dock drives its
/// content through Scene3DLayer, and all seven layer types fuse decoding with
/// OpenGL upload — each one owns GL IRenderPass objects and renders itself, with no
/// seam exposing the decoded render structs. Swapping the renderer underneath the
/// real dock therefore means porting the layer system first, which is a workstream
/// of its own. This dock exists to answer the question that blocks that work:
/// **does a QRhiWidget survive the app's docking lifecycle?** The OpenGL view's
/// worst historical bugs were all context-recreation on dock/float/split reparent
/// (see the module CLAUDE.md), and QRhiWidget has its own version of that in
/// releaseResources(). Better to find out before building seven layers on top.
///
/// Scope is deliberately tiny: the TF overlay (axis triads + parent-connection
/// lines) and the reference grid, resolved from the live TransformService at the
/// tracker time. It renders NO topic data — no clouds, meshes, markers or maps —
/// and it accepts no drops.
class Scene3DRhiPreviewDock : public QWidget, public PJ::IDataWidget {
  Q_OBJECT

 public:
  explicit Scene3DRhiPreviewDock(QWidget* parent = nullptr);
  ~Scene3DRhiPreviewDock() override;

  /// Whether the developer opt-in is active. When true the shell builds this dock
  /// in place of the real 3D dock, so the normal 3D view is unavailable for the
  /// session — acceptable for a dev flag, and it means the preview goes through
  /// the app's genuine dock creation, drop, float and layout-restore paths rather
  /// than a side door.
  [[nodiscard]] static bool enabled();

  QWidget* widget() override {
    return this;
  }
  void onTrackerTime(double time) override;

  /// Binds the TF source. Two paths are needed, because ingest and dock creation
  /// race in both directions: the service's datasetTransformsReady signal covers a
  /// dataset filled AFTER this dock exists, and tryAdoptExistingDataset() covers one
  /// already ingested before it (the common case on layout restore, where the file
  /// is loaded first).
  void setTransformService(TransformService* service);

  /// The session, used only to resolve which dataset to take TF from.
  void setSessionManager(PJ::SessionManager* session);

  /// Accepts any dropped 3D topic — not to render it, but because the drop is what
  /// reveals which dataset to bind TF from. Returns true so the host keeps this dock
  /// rather than replacing it.
  bool tryAcceptObjectTopic(
      PJ::ObjectTopicId topic_id, PJ::sdk::BuiltinObjectType object_type, const QString& title) override;

  /// The QRhi view, for tests and diagnostics. Never null after construction.
  [[nodiscard]] rhi::RhiSceneViewWidget* sceneView() {
    return view_;
  }

 private:
  /// Re-resolve every TF frame at the current tracker time and push the result to
  /// the axis and connection passes.
  void refreshTf();
  /// Bind to the dataset of the first object topic the store already holds. No-op
  /// once a buffer is bound, and harmless when the session is empty.
  void tryAdoptExistingDataset();
  void bindDataset(PJ::DatasetId dataset_id);
  /// Point the camera at the TF tree once, the first time frames resolve. Without
  /// it the default orbit pose is nearly edge-on to the ground plane and the scene
  /// reads as an empty grid.
  void frameSceneOnce(const std::vector<glm::mat4>& triads);
  /// Pick a fixed frame: the first root of the TF forest, or the first frame if the
  /// tree has no reachable root. Deliberately not the app's remembered-frame
  /// policy — this dock has no frame picker.
  void chooseFixedFrame();

  rhi::RhiSceneViewWidget* view_ = nullptr;
  TransformService* transform_service_ = nullptr;
  /// Held so a re-bind can drop the previous connection; see setTransformService.
  QMetaObject::Connection tf_ready_conn_;
  std::shared_ptr<TransformBuffer> tf_buffer_;
  /// Which dataset tf_buffer_ came from, so a repeated bind for the same one is a
  /// no-op rather than a redundant re-read.
  std::optional<PJ::DatasetId> bound_dataset_;
  std::string fixed_frame_;
  PJ::SessionManager* session_ = nullptr;
  /// Absolute tracker time in nanoseconds, as last pushed by the host.
  std::int64_t tracker_ns_ = 0;
  bool framed_ = false;
  /// (frames, resolved, segments, fixed_frame) of the last logged refresh, so the
  /// per-tick diagnostic only prints when the outcome actually changes.
  std::tuple<std::size_t, std::size_t, std::size_t, std::string> last_logged_outcome_{};
};

}  // namespace pj::scene3d
