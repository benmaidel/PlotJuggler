// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
//
// Offscreen-QRhi coverage for the ported scene passes.
//
// Until this existed, every QRhi pass was verified by eye through a demo, and that
// let a specific class of bug through repeatedly: QRhi reports no error for a
// pipeline built against an empty shader-resource-binding layout, a pipeline whose
// sample count disagrees with its render target, a uniform block smaller than what
// a shader reads, or a depth resolve that never happens. Each renders something
// plausible — or nothing — while looking like a content problem.
//
// So the assertions here are deliberately shaped around that class rather than
// around pixel-exact goldens: "did anything draw at all", "does the background
// survive the composite byte-exact", "is annotation geometry still bypassing the
// tonemap". Those are cheap, stable across drivers, and would have caught the real
// bugs found during the port.
//
// The whole chain under test is production code: the passes render into
// RhiHdrTarget (MSAA + resolve, as in the widget), then RhiPresentPass composites
// into an RGBA8 texture that is read back. Skips cleanly when no QRhi backend can
// be created (a headless box with no GL and no Metal).

#include <gtest/gtest.h>
#include <rhi/qrhi.h>

#include <QGuiApplication>
#include <QImage>
#include <QOffscreenSurface>
#include <QSurfaceFormat>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <memory>
#include <set>
#include <vector>

#include "gtest_skip_exit.h"
#include "pj_scene3d_widgets/rhi/rhi_axis_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_hdr_target.h"
#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_mesh_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_present_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_tf_connections_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"

namespace {

using namespace pj::scene3d;
using namespace pj::scene3d::rhi;

constexpr int kW = 256;
constexpr int kH = 192;
// The scene background, matching RhiSceneViewWidget's light-theme clear.
constexpr float kBg = 0.96F;

/// Owns an offscreen QRhi plus the production HDR chain and composite, and renders
/// one frame to a QImage. Mirrors RhiSceneViewWidget::render() deliberately: a
/// harness that composited differently would not test the thing that keeps breaking.
class Harness {
 public:
  /// False when no backend is available — the caller should GTEST_SKIP().
  bool create() {
#if defined(Q_OS_MACOS)
    QRhiMetalInitParams params;
    rhi_.reset(QRhi::create(QRhi::Metal, &params));
#else
    surface_.reset(new QOffscreenSurface);
    QSurfaceFormat format;
    format.setDepthBufferSize(24);
    surface_->setFormat(format);
    surface_->create();
    QRhiGles2InitParams params;
    params.fallbackSurface = surface_.get();
    rhi_.reset(QRhi::create(QRhi::OpenGLES2, &params));
#endif
    if (rhi_ == nullptr) {
      return false;
    }
    // The composite target: RGBA8 so a readback needs no half-float conversion,
    // which is also what the widget's own target is.
    out_tex_.reset(rhi_->newTexture(QRhiTexture::RGBA8, QSize(kW, kH), 1, QRhiTexture::RenderTarget));
    if (!out_tex_->create()) {
      return false;
    }
    out_rt_.reset(rhi_->newTextureRenderTarget({{out_tex_.get()}}));
    out_rpd_.reset(out_rt_->newCompatibleRenderPassDescriptor());
    out_rt_->setRenderPassDescriptor(out_rpd_.get());
    return out_rt_->create();
  }

  [[nodiscard]] QRhi& rhi() {
    return *rhi_;
  }
  [[nodiscard]] RhiPresentPass& present() {
    return present_;
  }

  /// Render `passes` through the HDR chain and composite. Returns the RGBA8 result,
  /// or a null image when the chain or a readback failed.
  QImage render(const std::vector<IRhiRenderPass*>& passes, const RhiFrameContext& ctx_in, int samples = 4) {
    RhiFrameContext ctx = ctx_in;
    ctx.pixel_size = QSize(kW, kH);

    if (!hdr_.ensure(*rhi_, ctx.pixel_size, samples)) {
      return {};
    }
    if (!present_.initialize(*rhi_, *out_rpd_, 1)) {
      return {};
    }
    for (IRhiRenderPass* pass : passes) {
      if (!pass->initialize(*rhi_, *hdr_.renderPassDescriptor(), hdr_.sampleCount())) {
        return {};
      }
    }
    present_.setSourceTexture(hdr_.resolvedColor());
    present_.setDepthTexture(hdr_.resolvedDepth());

    QRhiCommandBuffer* cb = nullptr;
    if (rhi_->beginOffscreenFrame(&cb) != QRhi::FrameOpSuccess) {
      return {};
    }
    QRhiResourceUpdateBatch* updates = rhi_->nextResourceUpdateBatch();
    for (IRhiRenderPass* pass : passes) {
      pass->prepare(*updates, ctx);
    }
    present_.prepare(*updates, ctx);

    // Alpha follows the widget: 1 with a resolved depth (the far-plane bypass
    // rescues the background), 0 without (the marker path stands in).
    const float clear_alpha = present_.backgroundNeedsAlphaClear() ? 0.0F : 1.0F;
    const auto lin = [](float c) { return std::pow(c, 2.2F); };
    const QColor scene_clear = QColor::fromRgbF(lin(kBg), lin(kBg), lin(kBg), clear_alpha);

    cb->beginPass(hdr_.renderTarget(), scene_clear, {1.0F, 0}, updates);
    cb->setViewport({0.0F, 0.0F, static_cast<float>(kW), static_cast<float>(kH)});
    for (IRhiRenderPass* pass : passes) {
      pass->draw(*cb, ctx);
    }
    cb->endPass();

    QRhiReadbackResult readback;
    QRhiResourceUpdateBatch* post = rhi_->nextResourceUpdateBatch();
    post->readBackTexture({out_tex_.get()}, &readback);

    cb->beginPass(out_rt_.get(), scene_clear, {1.0F, 0});
    cb->setViewport({0.0F, 0.0F, static_cast<float>(kW), static_cast<float>(kH)});
    present_.draw(*cb, ctx);
    cb->endPass(post);
    rhi_->endOffscreenFrame();

    if (readback.data.isEmpty()) {
      return {};
    }
    const QImage img(
        reinterpret_cast<const uchar*>(readback.data.constData()), readback.pixelSize.width(),
        readback.pixelSize.height(), QImage::Format_RGBA8888);
    return img.copy();  // detach from the readback buffer
  }

 private:
  std::unique_ptr<QOffscreenSurface> surface_;
  std::unique_ptr<QRhi> rhi_;
  std::unique_ptr<QRhiTexture> out_tex_;
  std::unique_ptr<QRhiTextureRenderTarget> out_rt_;
  std::unique_ptr<QRhiRenderPassDescriptor> out_rpd_;
  RhiHdrTarget hdr_;
  RhiPresentPass present_;
};

/// A camera looking down at the origin, close enough that a unit-scale gizmo fills
/// a useful part of the frame.
RhiFrameContext makeContext() {
  RhiFrameContext ctx;
  const glm::vec3 eye(3.5F, -3.5F, 2.5F);
  const glm::mat4 view = glm::lookAt(eye, glm::vec3(0.0F), glm::vec3(0.0F, 0.0F, 1.0F));
  const glm::mat4 proj = glm::perspective(glm::radians(45.0F), static_cast<float>(kW) / kH, 0.1F, 100.0F);
  ctx.view_proj = proj * view;
  ctx.camera_pos_world = eye;
  return ctx;
}

int nonBackgroundPixels(const QImage& img) {
  const int bg = static_cast<int>(std::lround(kBg * 255.0F));
  int n = 0;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      if (std::abs(c.red() - bg) > 3 || std::abs(c.green() - bg) > 3 || std::abs(c.blue() - bg) > 3) {
        ++n;
      }
    }
  }
  return n;
}

int saturatedPixels(const QImage& img, int min_spread = 60) {
  int n = 0;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      const int hi = std::max({c.red(), c.green(), c.blue()});
      const int lo = std::min({c.red(), c.green(), c.blue()});
      if (hi - lo >= min_spread) {
        ++n;
      }
    }
  }
  return n;
}

class RhiPassesTest : public ::testing::Test {
 protected:
  void SetUp() override {
    if (!harness_.create()) {
      GTEST_SKIP() << "no QRhi backend available offscreen";
    }
  }
  Harness harness_;
};

// Anything drawing at all is the assertion that matters most: a pipeline built
// against an empty binding layout, or one whose sample count disagrees with its
// target, silently produces an empty or near-empty frame.
TEST_F(RhiPassesTest, GridDraws) {
  RhiGridPass grid;
  grid.setGeometry(8.0F, 8);
  const QImage img = harness_.render({&grid}, makeContext());
  ASSERT_FALSE(img.isNull()) << "offscreen render produced no image";
  EXPECT_GT(nonBackgroundPixels(img), 100) << "the grid drew nothing";
}

// The background must survive linearize-on-clear -> composite bypass -> sRGB encode
// byte-exact. This is the regression guard for the whole linear-light contract: any
// pass writing the wrong transfer function, or a broken far-plane bypass, shifts it.
TEST_F(RhiPassesTest, BackgroundRoundTripsExactly) {
  RhiGridPass grid;
  grid.setGeometry(8.0F, 8);
  const QImage img = harness_.render({&grid}, makeContext());
  ASSERT_FALSE(img.isNull());
  const int expected = static_cast<int>(std::lround(kBg * 255.0F));
  // Sample a corner, which the camera framing leaves as pure background.
  const QColor corner = img.pixelColor(1, 1);
  EXPECT_NEAR(corner.red(), expected, 1);
  EXPECT_NEAR(corner.green(), expected, 1);
  EXPECT_NEAR(corner.blue(), expected, 1);
}

// TF triads are ANNOTATION: they drive the HDR target's alpha marker to 0 so the
// composite passes them through ungraded. Two consequences are asserted together —
// that they draw at all (a uniform block smaller than the shader reads collapsed
// every arrow to a degenerate point during the port), and that their colour does
// not move when the tonemap operator changes.
TEST_F(RhiPassesTest, AxisTriadsDrawAndBypassTheTonemap) {
  RhiAxisPass axis;
  axis.setFrames({glm::mat4(1.0F)});
  axis.setAxisLength(1.0F);

  auto params = harness_.present().compositeParams();
  params.tonemap_mode = 0;  // None
  harness_.present().setCompositeParams(params);
  const QImage none = harness_.render({&axis}, makeContext());
  ASSERT_FALSE(none.isNull());
  const int coloured = saturatedPixels(none);
  EXPECT_GT(coloured, 50) << "the axis triad drew nothing recognisable";

  params.tonemap_mode = 2;  // AgX, which desaturates hard
  harness_.present().setCompositeParams(params);
  const QImage agx = harness_.render({&axis}, makeContext());
  ASSERT_FALSE(agx.isNull());

  // NOT exact equality. The grade marker rides the target's alpha, so the MSAA
  // resolve averages it at silhouettes and those edge pixels are deliberately
  // PARTLY graded — the design calls that feathering the seam. What must hold is
  // that the interior is untouched, so only a thin edge may move. A broken bypass
  // moves essentially every coloured pixel instead, which this still catches.
  int differing = 0;
  for (int y = 0; y < none.height(); ++y) {
    for (int x = 0; x < none.width(); ++x) {
      const QColor a = none.pixelColor(x, y);
      const QColor b = agx.pixelColor(x, y);
      if (std::abs(a.red() - b.red()) > 2 || std::abs(a.green() - b.green()) > 2 || std::abs(a.blue() - b.blue()) > 2) {
        ++differing;
      }
    }
  }
  EXPECT_LT(differing, coloured / 5) << "annotation geometry moved with the tonemap across " << differing << " of "
                                     << coloured
                                     << " coloured pixels: the alpha grade marker is not reaching the composite";
}

// A scalar ramp must come out as a RANGE of colours. A cloud whose vertex attributes
// are misbound still draws — flat, or in one colour — so counting distinct hues is
// what separates "drew" from "drew correctly".
TEST_F(RhiPassesTest, PointCloudAppliesColormapAcrossItsRange) {
  struct Point {
    float x;
    float y;
    float z;
    float scalar;
  };
  std::vector<Point> points;
  constexpr int kCount = 400;
  for (int i = 0; i < kCount; ++i) {
    const float t = static_cast<float>(i) / (kCount - 1);
    points.push_back(Point{-2.0F + (4.0F * t), 0.0F, 0.3F, t});
  }

  RhiPointcloudPass cloud;
  cloud.setPoints(points.data(), static_cast<int>(points.size()), RhiPointcloudPass::Layout{});
  cloud.setScalarRange(0.0F, 1.0F);
  cloud.setColormap(PJ::Colormap::kTurbo);
  cloud.setPointRadius(0.05F);

  const QImage img = harness_.render({&cloud}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 100) << "the point cloud drew nothing";

  std::set<int> hues;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      if (std::max({c.red(), c.green(), c.blue()}) - std::min({c.red(), c.green(), c.blue()}) >= 60) {
        hues.insert(c.hue() / 15);  // coarse buckets: robust to driver rounding
      }
    }
  }
  EXPECT_GE(hues.size(), 4U) << "a 0..1 turbo ramp should span several hues, not one flat colour";
}

// The model matrix must actually place the cloud. Without it the pass draws in raw
// SOURCE-frame coordinates, which looks correct exactly when the source frame sits
// near the origin — the shape the test fixture happens to have, and the reason this
// shipped broken once already. Asserting that a translation MOVES the rendered
// centroid is what makes that falsifiable.
TEST_F(RhiPassesTest, PointCloudHonoursItsModelMatrix) {
  struct Point {
    float x;
    float y;
    float z;
    float scalar;
  };
  std::vector<Point> points;
  for (int i = 0; i < 200; ++i) {
    const float t = static_cast<float>(i) / 199.0F;
    points.push_back(Point{-0.3F + (0.6F * t), 0.0F, 0.4F, t});
  }

  const auto centroidX = [](const QImage& img) {
    double sum = 0.0;
    double weight = 0.0;
    for (int y = 0; y < img.height(); ++y) {
      for (int x = 0; x < img.width(); ++x) {
        const QColor c = img.pixelColor(x, y);
        if (std::max({c.red(), c.green(), c.blue()}) - std::min({c.red(), c.green(), c.blue()}) >= 60) {
          sum += x;
          weight += 1.0;
        }
      }
    }
    return weight > 0.0 ? sum / weight : -1.0;
  };

  RhiPointcloudPass cloud;
  cloud.setPoints(points.data(), static_cast<int>(points.size()), RhiPointcloudPass::Layout{});
  cloud.setScalarRange(0.0F, 1.0F);
  cloud.setPointRadius(0.05F);

  cloud.setModelMatrix(glm::mat4(1.0F));
  const QImage identity = harness_.render({&cloud}, makeContext());
  ASSERT_FALSE(identity.isNull());
  const double centre_identity = centroidX(identity);
  ASSERT_GT(centre_identity, 0.0) << "the cloud drew nothing at identity";

  // A metre of +Y in world space; the reference camera looks along -Y-ish, so this
  // shifts the projection sideways by far more than any antialiasing wobble.
  cloud.setModelMatrix(glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 1.0F, 0.0F)));
  const QImage shifted = harness_.render({&cloud}, makeContext());
  ASSERT_FALSE(shifted.isNull());
  const double centre_shifted = centroidX(shifted);
  ASSERT_GT(centre_shifted, 0.0) << "the cloud drew nothing when translated";

  EXPECT_GT(std::abs(centre_shifted - centre_identity), 10.0)
      << "the model matrix did not move the cloud: source-frame placement is being ignored";
}

// --- Smoke coverage for the remaining passes -------------------------------
//
// Deliberately shallow: each asserts only that the pass DRAWS. That is not
// laziness — it is the assertion that actually catches this port's recurring
// failure mode, where a pipeline built against an empty binding layout or with a
// sample count that disagrees with its target renders nothing at all and QRhi
// reports success. Depth of behaviour per pass belongs in tests of their own.

TEST_F(RhiPassesTest, TfConnectionLinesDraw) {
  RhiTfConnectionsPass lines;
  lines.setSegments({glm::vec3(-1.0F, 0.0F, 0.1F), glm::vec3(1.0F, 0.0F, 1.5F)});
  const QImage img = harness_.render({&lines}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 10);
}

TEST_F(RhiPassesTest, PoseTriadsDraw) {
  RhiPosesPass poses;
  PoseTriadInstance arm;
  arm.model = glm::scale(glm::mat4(1.0F), glm::vec3(1.0F));
  arm.color = glm::vec4(0.95F, 0.45F, 0.10F, 1.0F);
  poses.setInstances({arm});
  poses.setFrameWorld(glm::mat4(1.0F));
  const QImage img = harness_.render({&poses}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 50);
}

TEST_F(RhiPassesTest, VoxelGridDraws) {
  constexpr int kDim = 6;
  std::vector<float> field(static_cast<std::size_t>(kDim) * kDim * kDim, 1.0F);
  RhiVoxelGridPass voxels;
  voxels.setField(field.data(), kDim, kDim, kDim);
  voxels.setCellSize(glm::vec3(0.3F));
  voxels.setModelMatrix(glm::translate(glm::mat4(1.0F), glm::vec3(-0.9F, -0.9F, 0.0F)));
  voxels.setColorRange(0.0F, 1.0F);
  const QImage img = harness_.render({&voxels}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 100);
}

TEST_F(RhiPassesTest, OccupancyGridDraws) {
  constexpr int kW2 = 16;
  constexpr int kH2 = 16;
  std::vector<std::uint8_t> cells(static_cast<std::size_t>(kW2) * kH2, 100);  // fully occupied
  RhiOccupancyGridPass occupancy;
  occupancy.setGrid(cells.data(), kW2, kH2);
  glm::mat4 model(1.0F);
  model = glm::translate(model, glm::vec3(-2.0F, -2.0F, 0.02F));
  model = glm::scale(model, glm::vec3(4.0F, 4.0F, 1.0F));
  occupancy.setModelMatrix(model);
  const QImage img = harness_.render({&occupancy}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 100);
}

TEST_F(RhiPassesTest, MeshPrimitiveDraws) {
  RhiMeshPass mesh;
  RhiMeshPass::DrawCall box;
  box.kind = RhiMeshPass::GeometryKind::kBox;
  box.model = glm::scale(glm::mat4(1.0F), glm::vec3(1.2F));
  box.color = glm::vec4(0.85F, 0.25F, 0.20F, 1.0F);
  box.use_vertex_color = false;
  mesh.setVisualDraws({box});
  const QImage img = harness_.render({&mesh}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 100);
}

// Every link of a robot is placed by DrawCall::model ALONE — IMeshSink carries no
// transform, because RobotModelLayer resolves per-link TF itself and bakes it into
// the matrix. So a pass that ignored the per-draw model, or that shared one uniform
// slot across draws instead of binding each draw's own dynamic offset, would collapse
// an entire robot onto a single spot. That failure is not hypothetical: the marker
// arrows collapsed to a point for exactly that reason (a uniform block whose per-draw
// prefix was the wrong size), and it renders plenty of pixels — so MeshPrimitiveDraws
// above, which only asserts that something drew, cannot see it.
TEST_F(RhiPassesTest, MeshDrawsArePlacedIndependently) {
  const auto boxAt = [](float x) {
    RhiMeshPass::DrawCall draw;
    draw.kind = RhiMeshPass::GeometryKind::kBox;
    draw.model = glm::translate(glm::mat4(1.0F), glm::vec3(x, 0.0F, 0.0F));
    draw.color = glm::vec4(0.85F, 0.25F, 0.20F, 1.0F);
    draw.use_vertex_color = false;
    return draw;
  };

  // Columns of the image that contain any drawn pixel. Column occupancy is enough
  // to tell "two boxes side by side" from "two boxes on top of each other", and is
  // immune to the shading differences that a centroid would pick up.
  const auto occupiedColumns = [](const QImage& img) {
    const int bg = static_cast<int>(std::lround(kBg * 255.0F));
    std::vector<bool> cols(static_cast<std::size_t>(img.width()), false);
    for (int y = 0; y < img.height(); ++y) {
      for (int x = 0; x < img.width(); ++x) {
        const QColor c = img.pixelColor(x, y);
        if (std::abs(c.red() - bg) > 3 || std::abs(c.green() - bg) > 3 || std::abs(c.blue() - bg) > 3) {
          cols[static_cast<std::size_t>(x)] = true;
        }
      }
    }
    return cols;
  };
  const auto span = [](const std::vector<bool>& cols) {
    int lo = -1;
    int hi = -1;
    for (std::size_t i = 0; i < cols.size(); ++i) {
      if (cols[i]) {
        lo = lo < 0 ? static_cast<int>(i) : lo;
        hi = static_cast<int>(i);
      }
    }
    return lo < 0 ? 0 : (hi - lo) + 1;
  };
  // Whether a run of empty columns sits strictly between the leftmost and rightmost
  // drawn column — i.e. the drawn pixels form two separated clusters.
  const auto hasInteriorGap = [](const std::vector<bool>& cols) {
    std::size_t lo = 0;
    while (lo < cols.size() && !cols[lo]) {
      ++lo;
    }
    std::size_t hi = cols.size();
    while (hi > lo && !cols[hi - 1]) {
      --hi;
    }
    for (std::size_t i = lo; i < hi; ++i) {
      if (!cols[i]) {
        return true;
      }
    }
    return false;
  };

  RhiMeshPass one;
  one.setVisualDraws({boxAt(0.0F)});
  const QImage single = harness_.render({&one}, makeContext());
  ASSERT_FALSE(single.isNull());
  const std::vector<bool> single_cols = occupiedColumns(single);
  const int single_span = span(single_cols);
  ASSERT_GT(single_span, 0) << "the box drew nothing at the origin";
  // Guards the gap detector itself: one box must read as ONE cluster, or a gap in
  // the two-box case would prove nothing.
  ASSERT_FALSE(hasInteriorGap(single_cols)) << "a single box already reads as two clusters";

  RhiMeshPass two;
  two.setVisualDraws({boxAt(-1.5F), boxAt(1.5F)});
  const QImage pair = harness_.render({&two}, makeContext());
  ASSERT_FALSE(pair.isNull());
  const std::vector<bool> pair_cols = occupiedColumns(pair);

  EXPECT_GT(span(pair_cols), single_span * 2)
      << "two boxes 3 m apart cover barely more width than one: the per-draw model is being ignored";
  EXPECT_TRUE(hasInteriorGap(pair_cols)) << "the two boxes drew as a single cluster: they share one model matrix";
}

TEST_F(RhiPassesTest, MarkerCubeDraws) {
  auto batch = std::make_shared<DecodedSceneEntities>();
  batch->frames = {"world"};
  MarkerSolid cube;
  cube.model = glm::scale(glm::mat4(1.0F), glm::vec3(1.0F));
  cube.color = glm::vec4(0.95F, 0.55F, 0.15F, 1.0F);
  cube.frame_index = 0;
  batch->cubes.push_back(cube);

  RhiMarkerPass markers;
  markers.setActive(batch);
  markers.setFrameTransforms({glm::mat4(1.0F)});
  const QImage img = harness_.render({&markers}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_GT(nonBackgroundPixels(img), 100);
}

}  // namespace

int main(int argc, char** argv) {
  QGuiApplication app(argc, argv);
  ::testing::InitGoogleTest(&argc, argv);
  return pj::scene3d::test::runTestsReportingSkip();
}
