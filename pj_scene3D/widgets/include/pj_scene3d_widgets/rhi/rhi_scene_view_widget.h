#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <QPoint>
#include <QRhiWidget>
#include <memory>
#include <utility>
#include <vector>

#include "pj_scene3d_core/camera/camera.h"
#include "pj_scene3d_widgets/rhi/rhi_axis_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_hdr_target.h"
#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_mesh_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_present_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_ssao_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_tf_connections_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"

namespace pj::scene3d::rhi {

/// QRhi-based 3D scene view: the successor to the OpenGL SceneViewWidget.
///
/// Exists alongside the GL widget rather than replacing it, so the port can land
/// pass by pass while Linux keeps shipping the proven renderer. It is NOT yet
/// wired into Scene3DDockWidget; the demos drive it.
///
/// Why QRhi at all: the GL renderer needs an OpenGL 4.5 core context plus a
/// compute shader, and Apple's OpenGL is frozen at 4.1 with no compute, so the 3D
/// view cannot run on macOS as written. QRhi targets Metal there (and D3D/Vulkan
/// elsewhere) from one set of baked shaders.
class RhiSceneViewWidget : public QRhiWidget {
  Q_OBJECT

 public:
  explicit RhiSceneViewWidget(QWidget* parent = nullptr);
  ~RhiSceneViewWidget() override;

  /// The grid pass, exposed so demos and (later) the dock can set extent/colour.
  RhiGridPass& gridPass() {
    return grid_pass_;
  }
  /// The TF triad pass, exposed so callers can push frame transforms.
  RhiAxisPass& axisPass() {
    return axis_pass_;
  }
  /// The TF parent-connection line pass.
  RhiTfConnectionsPass& tfConnectionsPass() {
    return tf_connections_pass_;
  }
  /// The composite / tonemap present pass, which owns the look knobs (tonemap
  /// mode, exposure, saturation) and the renderer's single sRGB encode.
  RhiPresentPass& presentPass() {
    return present_pass_;
  }
  /// Screen-space ambient occlusion, multiplied into the scene by the composite.
  RhiSsaoPass& ssaoPass() {
    return ssao_pass_;
  }
  /// Whether SSAO contributes this frame. It needs the resolved depth, so it is
  /// off whenever the HDR chain could not produce one.
  void setSsaoEnabled(bool enabled);

  /// Where a layer-contributed pass sits in the draw sequence.
  ///
  /// Ordering is by DEPTH BEHAVIOUR, not by content type, which is why it is an
  /// explicit slot rather than registration order: the ground overlay is translucent
  /// and depth-tests without writing, opaque content must depth-write over it, and
  /// blended annotations must see the finished depth buffer to be occluded correctly.
  enum class LayerPassSlot {
    kGroundOverlay,  ///< Occupancy maps and similar translucent ground decals.
    kOpaque,         ///< Meshes, voxel grids, point clouds.
    kAnnotation,     ///< Blended gizmos: markers, pose triads.
  };

  /// Register a pass contributed by a layer adapter. The view does NOT take
  /// ownership — the caller keeps it alive until removeLayerPass().
  ///
  /// This exists because passes are per-TOPIC, not per-view: a dock can hold several
  /// point-cloud topics, and each needs its own pass. Holding one pass per kind on
  /// the view meant the second topic silently overwrote the first.
  void addLayerPass(IRhiRenderPass* pass, LayerPassSlot slot);
  /// Unregister a pass. MUST be called before destroying it, or the next frame
  /// dereferences a dangling pointer. Safe to call for a pass never added.
  void removeLayerPass(IRhiRenderPass* pass);

  /// Replace the camera model. The new model adopts the outgoing model's pose, so
  /// switching does not move the viewpoint.
  void setCamera(std::unique_ptr<ICamera> camera);
  [[nodiscard]] ICamera* camera() const {
    return camera_.get();
  }

  /// True once at least one frame has been recorded through a live pipeline —
  /// what a headless check should wait on before grabbing the framebuffer.
  [[nodiscard]] bool hasRendered() const {
    return has_rendered_;
  }

  /// Requested MSAA level for the off-screen chain. Independent of the widget's
  /// own sampleCount, which is 1 once composited in a dock — that is precisely
  /// why the anti-aliasing has to live on a target this widget owns.
  void setSceneSamples(int samples);
  [[nodiscard]] int sceneSamples() const {
    return hdr_target_.sampleCount();
  }

  /// True when the frame went through the off-screen HDR chain rather than the
  /// direct-to-widget fallback. Lets a headless check assert which path ran.
  [[nodiscard]] bool usedHdrChain() const {
    return used_hdr_chain_;
  }

 protected:
  void initialize(QRhiCommandBuffer* cb) override;
  void render(QRhiCommandBuffer* cb) override;
  void releaseResources() override;

  void mousePressEvent(QMouseEvent* event) override;
  void mouseMoveEvent(QMouseEvent* event) override;
  void wheelEvent(QWheelEvent* event) override;

 private:
  /// Passes in draw order. Raw pointers to members owned by this widget — the
  /// list only fixes the order, never lifetime.
  [[nodiscard]] std::vector<IRhiRenderPass*> passes();

  /// projection * view for this frame, pre-multiplied by
  /// QRhi::clipSpaceCorrMatrix(). That correction is NOT cosmetic: the backends
  /// disagree on both framebuffer Y direction and depth range (Metal/Vulkan/D3D
  /// use [0,1], desktop GL [-1,1]), and glm's matrices are built for the GL
  /// convention. Applying it once here keeps every pass and shader
  /// backend-agnostic.
  [[nodiscard]] glm::mat4 buildViewProj(const QSize& pixel_size) const;
  /// View -> screen (uv + depth-buffer value, all [0,1]) for the screen-space
  /// passes. See RhiFrameContext::screen_from_view for what it folds in.
  [[nodiscard]] glm::mat4 buildScreenFromView(const QSize& pixel_size) const;

  /// Cached QRhi identity: a change means every pass's GPU objects died with the
  /// old device and must be rebuilt (widget reparent, screen change).
  QRhi* rhi_cached_ = nullptr;
  bool has_rendered_ = false;
  bool ssao_enabled_ = true;

  std::unique_ptr<ICamera> camera_;
  RhiGridPass grid_pass_;
  RhiAxisPass axis_pass_;
  RhiTfConnectionsPass tf_connections_pass_;

  /// Off-screen multisample HDR chain the scene renders into, plus the fullscreen
  /// pass that composites it onto the widget target. When the chain cannot be
  /// built the widget draws the scene straight into its own target instead, which
  /// costs MSAA and HDR but still shows the scene.
  RhiHdrTarget hdr_target_;
  /// Layer-contributed passes, kept per slot so the draw order is the slot order and
  /// registration order only breaks ties within a slot.
  std::vector<std::pair<LayerPassSlot, IRhiRenderPass*>> layer_passes_;

  RhiPresentPass present_pass_;
  RhiSsaoPass ssao_pass_;
  int desired_samples_ = 4;
  bool used_hdr_chain_ = false;
  /// Render pass descriptor the geometry passes were last initialized against.
  /// The HDR chain re-creates its descriptor on resize, and a pipeline is only
  /// valid for the descriptor it was built with, so a change forces a rebuild.
  QRhiRenderPassDescriptor* scene_rpd_ = nullptr;

  /// Last cursor position for drag-to-orbit, in logical pixels.
  QPoint last_cursor_;
};

}  // namespace pj::scene3d::rhi
