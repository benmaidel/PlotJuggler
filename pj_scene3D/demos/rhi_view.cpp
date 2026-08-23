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
#include <QImage>
#include <QSet>
#include <QString>
#include <cmath>
#include <cstdio>
#include <vector>

#include <glm/gtc/matrix_transform.hpp>

#include "pj_scene3d_widgets/rhi/rhi_scene_view_widget.h"

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
  view.axisPass().setFrames({
      glm::mat4(1.0F),
      glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.0F, 1.0F)),
      glm::translate(glm::mat4(1.0F), glm::vec3(1.2F, 0.6F, 1.8F)),
  });
  view.axisPass().setAxisLength(0.8F);

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
  view.pointcloudPass().setPoints(cloud.data(), static_cast<int>(cloud.size()),
                                  pj::scene3d::rhi::RhiPointcloudPass::Layout{});
  view.pointcloudPass().setScalarRange(0.0F, 1.0F);
  view.pointcloudPass().setColormap(PJ::Colormap::kTurbo);
  view.pointcloudPass().setPointRadius(0.045F);
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

  std::printf("rhi_view: saved %s (%dx%d), %d non-background sample(s)\n", qPrintable(out), frame.width(),
              frame.height(), drawn);
  std::printf("rhi_view: hdr_chain=%s scene_samples=%d distinct_tones_on_scanline=%d\n",
              view.usedHdrChain() ? "yes" : "NO (direct-to-widget fallback)", view.sceneSamples(),
              static_cast<int>(tones.size()));
  return drawn > 0 ? 0 : 2;
}
