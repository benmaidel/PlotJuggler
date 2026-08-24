// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
//
// Drives the QRhi scene view (RhiSceneViewWidget) that is replacing pj_scene3D's
// OpenGL renderer, and can verify itself headlessly.
//
// With a path argument it renders one frame from a fixed camera pose, saves a PNG
// and exits — so the ported renderer is comparable against the committed GL
// reference render (pj_scene3D/tools/reference/) as passes land. With no argument
// it opens an interactive window: drag to orbit, wheel to zoom.
//
//   ./build/pj_scene3D/demos/scene3d_rhi_view out.png   # headless, one frame
//   ./build/pj_scene3D/demos/scene3d_rhi_view           # interactive
//
// Exits non-zero if no frame could be produced.

#include <QApplication>
#include <QColor>
#include <QDeadlineTimer>
#include <QElapsedTimer>
#include <QImage>
#include <QRect>
#include <QSet>
#include <QString>
#include <cmath>
#include <cstdio>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <string>
#include <vector>

#include "pj_scene3d_core/poses_in_frame_render.h"
#include "pj_scene3d_widgets/mesh_primitives.h"
#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_mesh_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"
#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"

// Q_INIT_RESOURCE must sit at global scope: inside an anonymous namespace its
// extern declaration gets internal linkage and never resolves to the static
// library's symbol.
void initScene3dShaders() {
  Q_INIT_RESOURCE(scene3d_shaders);
}

namespace {

// Camera pose chosen to resemble the committed GL reference render: looking down
// at the ground grid from a moderate elevation, so the two are comparable by eye.
pj::scene3d::CameraState referencePose() {
  pj::scene3d::CameraState state;
  state.focal = glm::vec3(0.0F, 0.0F, 0.0F);
  state.radius = 14.0F;
  state.azimuth = glm::radians(35.0F);
  state.elevation = glm::radians(22.0F);
  state.perspective = true;
  return state;
}

}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  initScene3dShaders();

  const bool headless = argc > 1;
  const QString out = headless ? QString::fromLocal8Bit(argv[1]) : QString();

  pj::scene3d::rhi::RhiSceneViewWidget view;
  view.gridPass().setGeometry(20.0F, 20);
  // The same three-frame chain the MCAP fixture publishes on /tf
  // (world -> base_link -> sensor), so this render is comparable with the
  // committed GL reference image.
  // The view owns only the grid and the TF overlay; content passes are per-topic, so
  // a caller creates and registers its own. These live for the whole run.
  using Slot = pj::scene3d::rhi::RhiSceneViewWidget::LayerPassSlot;
  pj::scene3d::rhi::RhiPointcloudPass cloud_pass;
  pj::scene3d::rhi::RhiVoxelGridPass voxel_pass;
  pj::scene3d::rhi::RhiOccupancyGridPass occupancy_pass;
  pj::scene3d::rhi::RhiMeshPass mesh_pass;
  pj::scene3d::rhi::RhiPosesPass poses_pass;
  pj::scene3d::rhi::RhiMarkerPass marker_pass;
  view.addLayerPass(&occupancy_pass, Slot::kGroundOverlay);
  view.addLayerPass(&mesh_pass, Slot::kOpaque);
  view.addLayerPass(&voxel_pass, Slot::kOpaque);
  view.addLayerPass(&cloud_pass, Slot::kOpaque);
  view.addLayerPass(&marker_pass, Slot::kAnnotation);
  view.addLayerPass(&poses_pass, Slot::kAnnotation);

  view.axisPass().setFrames({
      glm::mat4(1.0F),
      glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.0F, 1.0F)),
      glm::translate(glm::mat4(1.0F), glm::vec3(1.2F, 0.6F, 1.8F)),
  });
  view.axisPass().setAxisLength(0.8F);
  // Parent links for the same chain, so the tree structure is visible.
  view.tfConnectionsPass().setSegments({
      glm::vec3(0.0F, 0.0F, 0.0F),
      glm::vec3(0.0F, 0.0F, 1.0F),
      glm::vec3(0.0F, 0.0F, 1.0F),
      glm::vec3(1.2F, 0.6F, 1.8F),
  });

  // A spiral cloud in the SAME record layout the MCAP fixture publishes
  // (contiguous float32 xyz then a float32 scalar, 16-byte stride), so this
  // exercises the layout that allows a wire buffer to be uploaded verbatim.
  struct WirePoint {
    float x;
    float y;
    float z;
    float intensity;
  };
  std::vector<WirePoint> cloud;
  cloud.reserve(2000);
  for (int i = 0; i < 2000; ++i) {
    const float t = static_cast<float>(i) / 2000.0F;
    const float angle = t * 12.0F * 3.14159265F;
    const float radius = 1.2F + (t * 2.5F);
    cloud.push_back({radius * std::cos(angle), radius * std::sin(angle), 0.4F + (t * 3.0F), t});
  }
  cloud_pass.setPoints(cloud.data(), static_cast<int>(cloud.size()), pj::scene3d::rhi::RhiPointcloudPass::Layout{});
  cloud_pass.setScalarRange(0.0F, 1.0F);
  cloud_pass.setColormap(PJ::Colormap::kTurbo);
  cloud_pass.setPointRadius(0.045F);

  // Dense voxel field: a hollow-ish shell whose value ramps with height, drawn
  // with the kAtOrAbove predicate so the shader's degenerate-clip path runs for
  // most of the lattice.
  {
    const int vc = 24;
    const int vr = 24;
    const int vs = 16;
    std::vector<float> field(static_cast<size_t>(vc) * vr * vs, 0.0F);
    for (int z = 0; z < vs; ++z) {
      for (int y = 0; y < vr; ++y) {
        for (int x = 0; x < vc; ++x) {
          const float dx = static_cast<float>(x) - (vc * 0.5F);
          const float dy = static_cast<float>(y) - (vr * 0.5F);
          const float radius = std::sqrt((dx * dx) + (dy * dy));
          // A cone: wider at the bottom, so the shell reads clearly in 3D.
          const float wanted = 10.0F - (static_cast<float>(z) * 0.5F);
          const bool shell = std::abs(radius - wanted) < 1.2F;
          field[(static_cast<size_t>(z) * vr * vc) + (static_cast<size_t>(y) * vc) + static_cast<size_t>(x)] =
              shell ? (0.2F + (static_cast<float>(z) / static_cast<float>(vs))) : 0.0F;
        }
      }
    }
    auto& vox = voxel_pass;
    vox.setField(field.data(), vc, vr, vs);
    vox.setCellSize(glm::vec3(0.22F));
    glm::mat4 vmodel(1.0F);
    vmodel = glm::translate(vmodel, glm::vec3(-2.6F, -2.6F, 0.05F));
    vox.setModelMatrix(vmodel);
    vox.setDrawMode(pj::scene3d::rhi::RhiVoxelGridPass::DrawMode::kAtOrAbove);
    vox.setThreshold(0.01F);
    vox.setColorRange(0.2F, 1.2F);
    vox.setColormap(PJ::Colormap::kViridis);
  }

  // Synthetic occupancy map: free interior, an occupied wall ring, a lethal blob,
  // and an unknown (255) outer border so the transparent-unknown path is exercised.
  {
    const int gw = 120;
    const int gh = 120;
    std::vector<unsigned char> cells(static_cast<size_t>(gw) * gh, 0);
    for (int r = 0; r < gh; ++r) {
      for (int c = 0; c < gw; ++c) {
        unsigned char v = 0;
        const bool border = r < 8 || c < 8 || r >= gh - 8 || c >= gw - 8;
        const bool wall = (r == 30 || r == 90) && c > 20 && c < 100;
        const int dx = c - 75;
        const int dy = r == 0 ? 0 : r - 60;
        const bool blob = (dx * dx) + (dy * dy) < 100;
        if (border) {
          v = 255;  // unknown
        } else if (wall || blob) {
          v = 100;  // fully occupied
        }
        cells[(static_cast<size_t>(r) * gw) + static_cast<size_t>(c)] = v;
      }
    }
    auto& occ = occupancy_pass;
    occ.setGrid(cells.data(), gw, gh);
    // Place a 12 m x 12 m map centred on the origin, just above z=0.
    glm::mat4 model(1.0F);
    model = glm::translate(model, glm::vec3(-6.0F, -6.0F, 0.01F));
    model = glm::scale(model, glm::vec3(12.0F, 12.0F, 1.0F));
    occ.setModelMatrix(model);
    occ.setColorScheme(pj::scene3d::rhi::RhiOccupancyGridPass::ColorScheme::kCostmap);
    occ.setOpacity(0.8F);
    // Exercise the partial-update path the way a live map does: send ONLY the
    // patch, so the pass re-uploads that rectangle instead of the whole grid.
    // Carves a free corridor straight through the lethal blob.
    const QRect patch_rect(20, 55, 80, 10);
    const std::vector<unsigned char> patch(static_cast<size_t>(patch_rect.width()) * patch_rect.height(), 0);
    occ.updateRegion(patch_rect, patch.data());
  }
  // Meshes: a row of spheres sweeping metallic 0 -> 1 at low roughness (the metals
  // must reflect the procedural sky/ground gradient rather than read black, which is
  // what the split-sum specular IBL buys), one rough dielectric box, and a
  // translucent box that has to land in the blended bucket.
  {
    auto& mesh = mesh_pass;
    std::vector<pj::scene3d::rhi::RhiMeshPass::DrawCall> visuals;

    constexpr int kSpheres = 5;
    for (int i = 0; i < kSpheres; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(kSpheres - 1);
      pj::scene3d::MeshData sphere = pj::scene3d::makeSphere();
      // The primitive builders bake a 0.7 grey COLOR_0, which would multiply the
      // material factor and mute it. Real glTF meshes carry white (or no) vertex
      // colour, so whiten it here to see the material alone.
      for (pj::scene3d::Vertex& vertex : sphere.vertices) {
        vertex.color = glm::vec4(1.0F);
      }
      auto material = std::make_shared<pj::scene3d::Material>();
      material->base_color_factor = glm::vec4(0.95F, 0.78F, 0.35F, 1.0F);  // gold-ish
      material->metallic_factor = t;
      material->roughness_factor = 0.18F + (0.10F * t);
      material->has_pbr = true;
      sphere.submeshes.front().material = material;

      const std::string key = "sphere_" + std::to_string(i);
      mesh.setMeshData(key, std::move(sphere));

      pj::scene3d::rhi::RhiMeshPass::DrawCall call;
      call.kind = pj::scene3d::rhi::RhiMeshPass::GeometryKind::kMesh;
      call.mesh_key = key;
      glm::mat4 model(1.0F);
      model = glm::translate(model, glm::vec3(-4.0F + (static_cast<float>(i) * 2.0F), 4.4F, 0.75F));
      model = glm::scale(model, glm::vec3(0.7F));
      call.model = model;
      call.use_vertex_color = true;  // material-driven base colour
      visuals.push_back(call);
    }

    // A rough red dielectric, drawn through the per-draw OVERRIDE tint path
    // (use_vertex_color = false) the way a URDF link colour arrives.
    {
      pj::scene3d::rhi::RhiMeshPass::DrawCall call;
      call.kind = pj::scene3d::rhi::RhiMeshPass::GeometryKind::kBox;
      glm::mat4 model(1.0F);
      model = glm::translate(model, glm::vec3(-4.6F, 1.4F, 0.6F));
      model = glm::rotate(model, glm::radians(20.0F), glm::vec3(0.0F, 0.0F, 1.0F));
      model = glm::scale(model, glm::vec3(1.2F));
      call.model = model;
      call.color = glm::vec4(0.85F, 0.25F, 0.20F, 1.0F);
      call.use_vertex_color = false;
      visuals.push_back(call);
    }
    // Same shape at alpha < 1: must be bucketed translucent and let the grid show
    // through rather than punching a hole in it.
    {
      pj::scene3d::rhi::RhiMeshPass::DrawCall call;
      call.kind = pj::scene3d::rhi::RhiMeshPass::GeometryKind::kCylinder;
      glm::mat4 model(1.0F);
      model = glm::translate(model, glm::vec3(-2.4F, 1.4F, 0.7F));
      model = glm::scale(model, glm::vec3(0.7F, 0.7F, 1.4F));
      call.model = model;
      call.color = glm::vec4(0.30F, 0.55F, 0.95F, 0.45F);
      call.use_vertex_color = false;
      visuals.push_back(call);
    }

    mesh.setVisualDraws(std::move(visuals));
  }

  // Pose array, shaped like an AMCL particle cloud: a fan of single-X-arrow
  // gizmos in one user colour at partial opacity, so the pass's override_color and
  // blended-opacity paths both run. The instances stay FRAME-LOCAL and the frame's
  // placement goes in via setFrameWorld(), which is the whole point of the pass —
  // so a moving frame animates for the cost of one uniform write.
  {
    PJ::sdk::PosesInFrame msg;
    msg.frame_id = "particles";
    constexpr int kParticles = 72;
    for (int i = 0; i < kParticles; ++i) {
      const float t = static_cast<float>(i) / static_cast<float>(kParticles);
      const float angle = t * 6.0F;
      const float radius = 0.4F + (t * 1.9F);
      PJ::sdk::Pose pose;
      pose.position.x = static_cast<double>(radius * std::cos(angle));
      pose.position.y = static_cast<double>(radius * std::sin(angle));
      pose.position.z = 0.15 + (0.9 * static_cast<double>(t));
      // Yaw tangent to the spiral: q = (0, 0, sin(yaw/2), cos(yaw/2)).
      const double yaw = static_cast<double>(angle) + 1.5708;
      pose.orientation.z = std::sin(yaw * 0.5);
      pose.orientation.w = std::cos(yaw * 0.5);
      msg.poses.push_back(pose);
    }

    pj::scene3d::PoseTriadStyle style;
    style.axis_length = 0.42F;
    style.x_arrow_only = true;
    style.override_color = true;
    style.color = glm::vec3(0.95F, 0.45F, 0.10F);
    style.opacity = 0.75F;

    auto& poses = poses_pass;
    poses.setInstances(pj::scene3d::buildPoseTriadInstances(msg, style));
    // A non-identity parent frame: offset and yawed, so a wrong frame_world would
    // be obvious rather than hiding behind the identity.
    glm::mat4 frame_world(1.0F);
    frame_world = glm::translate(frame_world, glm::vec3(3.4F, -2.2F, 0.0F));
    frame_world = glm::rotate(frame_world, glm::radians(25.0F), glm::vec3(0.0F, 0.0F, 1.0F));
    poses.setFrameWorld(frame_world);
  }

  // Markers: one of every ported primitive type, laid out in a row so each is
  // individually checkable. Two frames are used so the frame-index -> world
  // resolution is exercised rather than everything sitting on the identity.
  {
    auto batch = std::make_shared<pj::scene3d::DecodedSceneEntities>();
    batch->frames = {"markers_a", "markers_b"};

    const auto place = [](float x, float y, float z) { return glm::translate(glm::mat4(1.0F), glm::vec3(x, y, z)); };

    // Cubes: one opaque, one translucent (the translucent one must show all 12
    // edges through itself, which is what the no-depth-write rule buys).
    batch->cubes.push_back(
        {place(-5.0F, -5.4F, 0.5F) * glm::scale(glm::mat4(1.0F), glm::vec3(0.9F)), {0.95F, 0.55F, 0.15F, 1.0F}, 0});
    batch->cubes.push_back(
        {place(-3.4F, -5.4F, 0.5F) * glm::scale(glm::mat4(1.0F), glm::vec3(0.9F)), {0.20F, 0.75F, 0.35F, 0.40F}, 0});
    // Sphere, deliberately an ellipsoid so the non-uniform-scale path is covered.
    batch->spheres.push_back(
        {place(-1.8F, -5.4F, 0.5F) * glm::scale(glm::mat4(1.0F), glm::vec3(1.1F, 0.7F, 0.9F)),
         {0.30F, 0.45F, 0.95F, 1.0F},
         0});
    // Cylinder, cone (top collapsed) and truncated cone — the taper path.
    batch->cylinders.push_back({place(-0.4F, -5.4F, 0.5F), {0.85F, 0.30F, 0.75F, 1.0F}, 1.0F, 1.0F, 0});
    batch->cylinders.push_back({place(0.8F, -5.4F, 0.5F), {0.90F, 0.75F, 0.20F, 1.0F}, 1.0F, 0.0F, 0});
    batch->cylinders.push_back({place(2.0F, -5.4F, 0.5F), {0.25F, 0.80F, 0.80F, 1.0F}, 1.0F, 0.35F, 0});
    // Arrow and an axes glyph, both on the SECOND frame.
    batch->arrows.push_back({place(3.4F, -5.4F, 0.4F), {0.95F, 0.20F, 0.20F, 1.0F}, 0.9F, 0.12F, 0.35F, 0.26F, 1});
    batch->axes.push_back({place(5.0F, -5.4F, 0.4F), 0.9F, 0.09F, 1});
    // A line strip (pre-expanded to segment pairs) and a triangle fan.
    {
      pj::scene3d::MarkerLineBatch lines;
      lines.model = place(-5.0F, -1.9F, 0.1F);
      lines.color = {0.10F, 0.10F, 0.12F, 1.0F};
      for (int i = 0; i < 24; ++i) {
        const float t0 = static_cast<float>(i) * 0.28F;
        const float t1 = static_cast<float>(i + 1) * 0.28F;
        lines.vertices.push_back(glm::vec3(t0 * 0.4F, std::sin(t0) * 0.35F, 0.0F));
        lines.vertices.push_back(glm::vec3(t1 * 0.4F, std::sin(t1) * 0.35F, 0.0F));
      }
      batch->lines.push_back(std::move(lines));
    }
    {
      pj::scene3d::MarkerTriangleBatch tris;
      tris.model = place(2.6F, -1.9F, 0.05F);
      tris.color = {0.55F, 0.35F, 0.85F, 1.0F};
      constexpr int kFan = 10;
      for (int i = 0; i < kFan; ++i) {
        const float a0 = static_cast<float>(i) * 0.55F;
        const float a1 = static_cast<float>(i + 1) * 0.55F;
        tris.vertices.push_back(glm::vec3(0.0F, 0.0F, 0.0F));
        tris.vertices.push_back(glm::vec3(std::cos(a0), std::sin(a0), 0.0F));
        tris.vertices.push_back(glm::vec3(std::cos(a1), std::sin(a1), 0.0F));
        for (int k = 0; k < 3; ++k) {
          tris.normals.push_back(glm::vec3(0.0F, 0.0F, 1.0F));
        }
      }
      batch->triangles.push_back(std::move(tris));
    }

    auto& markers = marker_pass;
    markers.setActive(batch);
    // Frame 0 sits at the origin; frame 1 is offset and yawed, so a broken
    // frame-index lookup would misplace the arrow and axes visibly.
    markers.setFrameTransforms(
        {glm::mat4(1.0F), glm::rotate(
                              glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.9F, 0.0F)), glm::radians(30.0F),
                              glm::vec3(0.0F, 0.0F, 1.0F))});
  }

  // Composite look knobs. Overridable so the four tonemap operators can be
  // rendered and compared from one build:
  //   PJ_TONEMAP=0|1|2|3  (None / ACES / AgX / Khronos PBR Neutral)
  {
    auto params = view.presentPass().compositeParams();
    if (!qEnvironmentVariableIsEmpty("PJ_TONEMAP")) {
      params.tonemap_mode = qEnvironmentVariableIntValue("PJ_TONEMAP");
    }
    view.presentPass().setCompositeParams(params);
    // PJ_SSAO=0 disables screen-space occlusion, for A/B and cost comparison.
    if (qEnvironmentVariableIntValue("PJ_SSAO") == 0 && !qEnvironmentVariableIsEmpty("PJ_SSAO")) {
      view.setSsaoEnabled(false);
    }
  }

  if (view.camera() != nullptr) {
    view.camera()->adoptState(referencePose());
  }
  view.resize(900, 640);
  view.show();

  if (!headless) {
    return app.exec();
  }

  // QRhiWidget initializes lazily on its first render, which needs real
  // event-loop turns after show(); a bare processEvents() is not enough.
  QImage frame;
  QDeadlineTimer deadline(10000);
  while (!deadline.hasExpired()) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    if (view.hasRendered()) {
      frame = view.grabFramebuffer();
      if (!frame.isNull() && frame.width() > 1) {
        break;
      }
    }
  }

  if (frame.isNull() || frame.width() <= 1) {
    std::fprintf(stderr, "rhi_view: FAILED to produce a frame\n");
    return 1;
  }

  // PJ_BENCH=<n>: time n further repaints and report ms/frame. Coarse — it includes
  // event-loop overhead and grabFramebuffer's readback — but it is a like-for-like
  // A/B for the cost of a post pass, which is what it exists for.
  if (!qEnvironmentVariableIsEmpty("PJ_BENCH")) {
    const int frames = std::max(1, qEnvironmentVariableIntValue("PJ_BENCH"));
    QElapsedTimer timer;
    timer.start();
    int done = 0;
    QDeadlineTimer bench_deadline(60000);
    while (done < frames && !bench_deadline.hasExpired()) {
      view.update();
      QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
      ++done;
    }
    const double ms = static_cast<double>(timer.elapsed()) / static_cast<double>(done);
    std::fprintf(stdout, "rhi_view: bench %d frames, %.2f ms/frame\n", done, ms);
  }
  if (!frame.save(out)) {
    std::fprintf(stderr, "rhi_view: could not save %s\n", qPrintable(out));
    return 1;
  }

  // Count non-background pixels: a frame that is uniformly the clear colour means
  // the grid pass drew nothing, which a saved PNG alone would not reveal.
  int drawn = 0;
  for (int y = 0; y < frame.height(); y += 2) {
    for (int x = 0; x < frame.width(); x += 2) {
      const QColor c = frame.pixelColor(x, y);
      if (c.red() < 230 || c.green() < 230 || c.blue() < 230) {
        ++drawn;
      }
    }
  }
  // Count distinct greys along a horizontal scanline crossing several grid lines.
  // Aliased lines give ~2 tones (line + background); MSAA fills in intermediates,
  // so this separates "the chain ran" from "the chain actually anti-aliased".
  QSet<int> tones;
  const int probe_y = frame.height() * 3 / 4;
  for (int x = 0; x < frame.width(); ++x) {
    tones.insert(frame.pixelColor(x, probe_y).green());
  }

  std::printf(
      "rhi_view: saved %s (%dx%d), %d non-background sample(s)\n", qPrintable(out), frame.width(), frame.height(),
      drawn);
  std::printf(
      "rhi_view: hdr_chain=%s scene_samples=%d distinct_tones_on_scanline=%d\n",
      view.usedHdrChain() ? "yes" : "NO (direct-to-widget fallback)", view.sceneSamples(),
      static_cast<int>(tones.size()));
  return drawn > 0 ? 0 : 2;
}
