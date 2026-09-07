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
#include <QOpenGLContext>
#include <QSurfaceFormat>
#include <cmath>
#include <glm/gtc/matrix_transform.hpp>
#include <limits>
#include <memory>
#include <set>
#include <string>
#include <vector>

#include "gtest_skip_exit.h"
#include "pj_scene3d_core/shadow_camera.h"
#include "pj_scene3d_widgets/rhi/rhi_axis_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_hdr_target.h"
#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_mesh_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_occupancy_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_aabb_reducer.h"
#include "pj_scene3d_widgets/rhi/rhi_pointcloud_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_poses_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_present_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_shadow_map_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_tf_connections_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_pass.h"
#include "pj_scene3d_widgets/rhi/rhi_voxel_grid_sink.h"

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
#if !defined(Q_OS_MACOS)
  /// Lowest GLSL target in the qt6_add_shaders bake; keep in sync with CMakeLists.
  /// Only meaningful for the OpenGL backend — Metal takes the MSL variant.
  static constexpr int kMinBakedGlslVersion = 400;
#endif

  /// Why this environment cannot run the suite; empty when it can. Non-empty only
  /// after create() returned false.
  [[nodiscard]] const std::string& unsupportedReason() const {
    return unsupported_reason_;
  }

  /// False when no backend is available, or when the context is too old to have a
  /// baked shader variant — the caller should GTEST_SKIP().
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
      unsupported_reason_ = "no QRhi backend available offscreen";
      return false;
    }
#if !defined(Q_OS_MACOS)
    // A context below the LOWEST baked GLSL target matches no shader variant, so
    // every pipeline fails to create and every test renders a null image. That looks
    // like fourteen broken tests rather than one unsupported driver, which is exactly
    // the false signal SKIP_RETURN_CODE exists to prevent — so detect it up front.
    // Checked against the bake targets in CMakeLists (qt6_add_shaders GLSL "400,...").
    if (const auto* gl = static_cast<const QRhiGles2NativeHandles*>(rhi_->nativeHandles());
        gl != nullptr && gl->context != nullptr) {
      const QSurfaceFormat fmt = gl->context->format();
      const int glsl_version = (fmt.majorVersion() * 100) + (fmt.minorVersion() * 10);
      if (glsl_version < kMinBakedGlslVersion) {
        unsupported_reason_ = "OpenGL " + std::to_string(fmt.majorVersion()) + "." +
                              std::to_string(fmt.minorVersion()) + " is below the GLSL " +
                              std::to_string(kMinBakedGlslVersion) + " floor the shaders are baked for";
        return false;
      }
    }
#endif
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

  /// Force the composite onto its alpha-marker background bypass by withholding the
  /// resolved depth, as if QRhi::ResolveDepthStencil were unsupported.
  void setSuppressDepthBypass(bool suppress) {
    suppress_depth_bypass_ = suppress;
  }

  /// When both are set, render() runs the depth-only caster pass into the shadow map
  /// BEFORE the scene pass, which is the ordering the real view must also use.
  void setShadowSource(RhiShadowMapPass* target, RhiMeshPass* caster) {
    shadow_target_ = target;
    shadow_caster_ = caster;
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
    // Suppressing the binding is how FarPlaneAndAlphaBypassAgree exercises the
    // fallback path; everything else takes the resolved depth the widget takes.
    present_.setDepthTexture(suppress_depth_bypass_ ? nullptr : hdr_.resolvedDepth());

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

    // The shadow map is filled first, in the same frame. beginPass CONSUMES the
    // update batch, so the scene pass takes a fresh one; the uploads already queued
    // in the first batch have been applied by then.
    if (shadow_target_ != nullptr && shadow_caster_ != nullptr) {
      // Per frame, because the scene-side initialize() above may have released it.
      if (!shadow_caster_->initializeDepthOnly(*rhi_, *shadow_target_->renderPassDescriptor())) {
        return {};
      }
      shadow_caster_->prepareDepthOnly(*updates);
      shadow_target_->begin(*cb, updates);
      shadow_caster_->drawDepthOnly(*cb);
      shadow_target_->end(*cb);
      updates = rhi_->nextResourceUpdateBatch();
    }

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
  bool suppress_depth_bypass_ = false;
  RhiShadowMapPass* shadow_target_ = nullptr;
  RhiMeshPass* shadow_caster_ = nullptr;
  std::string unsupported_reason_;
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

// Horizontal centroid of every drawn pixel, or -1 when nothing drew. Coarse on
// purpose: it answers "did the content move", which is what a placement bug breaks.
// The background the image ACTUALLY has, read from a corner. Deliberately not the
// kBg constant: if the composite mis-grades the background (as happens when the
// far-plane bypass fails), every helper keyed to the constant reports that every
// pixel is "drawn" — and the placement tests below then fail with confident messages
// blaming the model matrix, which is not what broke. Calibrating per image keeps a
// background bug reported by BackgroundRoundTripsExactly alone, where it belongs.
int backgroundLevel(const QImage& img) {
  const QColor corner = img.pixelColor(0, 0);
  return std::max({corner.red(), corner.green(), corner.blue()});
}

// QMatrix4x4 (column-major) -> glm::mat4, for QRhi::clipSpaceCorrMatrix().
glm::mat4 toGlmMat(const QMatrix4x4& m) {
  glm::mat4 out{1.0F};
  const float* src = m.constData();
  for (int col = 0; col < 4; ++col) {
    for (int row = 0; row < 4; ++row) {
      out[col][row] = src[(col * 4) + row];
    }
  }
  return out;
}

double drawnCentroidX(const QImage& img) {
  const int bg = backgroundLevel(img);
  double sum = 0.0;
  double weight = 0.0;
  for (int y = 0; y < img.height(); ++y) {
    for (int x = 0; x < img.width(); ++x) {
      const QColor c = img.pixelColor(x, y);
      if (std::abs(c.red() - bg) > 3 || std::abs(c.green() - bg) > 3 || std::abs(c.blue() - bg) > 3) {
        sum += x;
        weight += 1.0;
      }
    }
  }
  return weight > 0.0 ? sum / weight : -1.0;
}

int nonBackgroundPixels(const QImage& img) {
  const int bg = backgroundLevel(img);
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
      GTEST_SKIP() << harness_.unsupportedReason();
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
// The composite has TWO ways to keep the background out of the tonemap: the
// far-plane depth bypass (primary, needs a resolved depth texture) and the
// alpha-marker path (fallback, used when QRhi cannot resolve depth). They must agree
// — they exist to produce the same background, and the choice between them is an
// implementation detail of what the backend supports.
//
// This is the assertion that isolates a backend where the depth resolve is ACCEPTED
// but does not read back as far-plane depth. That is not hypothetical: QRhi reports
// ResolveDepthStencil supported on the OpenGL backend, so the primary path is chosen
// and silently fails, and the background gets graded — sRGB(tonemap(lin(0.96))) lands
// around 236 instead of the theme's 245. Without this test the symptom appears in
// BackgroundRoundTripsExactly with no indication of WHICH mechanism broke.
TEST_F(RhiPassesTest, FarPlaneAndAlphaBypassAgree) {
  RhiGridPass grid;

  harness_.setSuppressDepthBypass(false);
  const QImage with_depth = harness_.render({&grid}, makeContext());
  ASSERT_FALSE(with_depth.isNull());

  harness_.setSuppressDepthBypass(true);
  const QImage with_alpha = harness_.render({&grid}, makeContext());
  ASSERT_FALSE(with_alpha.isNull());
  harness_.setSuppressDepthBypass(false);

  const int depth_bg = backgroundLevel(with_depth);
  const int alpha_bg = backgroundLevel(with_alpha);
  const int expected = static_cast<int>(std::lround(kBg * 255.0F));

  EXPECT_NEAR(alpha_bg, expected, 1) << "the alpha-marker bypass did not preserve the background";
  EXPECT_NEAR(depth_bg, expected, 1)
      << "the far-plane depth bypass did not preserve the background: this backend accepts a resolved "
         "depth texture that does not read back as far-plane depth, so RhiPresentPass must not trust "
         "QRhi::ResolveDepthStencil here and should fall back to the alpha clear";
  EXPECT_NEAR(depth_bg, alpha_bg, 1) << "the two background-bypass mechanisms disagree";
}

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

// Voxel grids are the ONE layer whose placement cannot be checked in the running
// app: no ROS message decodes to a VoxelGrid, so the screenshot fixture cannot carry
// one and the falsification-by-identity check every other adapter got is unavailable.
// This is the substitute, and it goes through RhiVoxelGridSink rather than the pass so
// that the adapter's own model composition — fixed_from_source * grid origin — is what
// gets verified. Both terms are exercised separately: either one silently dropped
// leaves the grid sitting at the world origin, which looks perfectly fine.
TEST_F(RhiPassesTest, VoxelGridSinkPlacesTheGridByFrameAndOrigin) {
  const auto uploadAt = [](double origin_x) {
    VoxelGridUpload upload;
    upload.frame_id = "sensor";
    upload.origin.position.x = origin_x;
    upload.origin.orientation.w = 1.0;
    upload.cell_size = glm::vec3(0.25F);
    upload.column_count = 4;
    upload.row_count = 4;
    upload.slice_count = 4;
    upload.kind = VoxelValueKind::kScalar;
    upload.scalar.assign(4 * 4 * 4, 1.0F);
    return upload;
  };

  RhiVoxelGridPass voxels;
  RhiVoxelGridSink sink(voxels);
  sink.setDrawMode(VoxelDrawMode::kAll);
  sink.setAutoRange(false);
  sink.setManualRange(0.0F, 1.0F);

  sink.setGrid(uploadAt(0.0));
  sink.setFrameTransform(glm::mat4(1.0F));
  const QImage base = harness_.render({&voxels}, makeContext());
  ASSERT_FALSE(base.isNull());
  const double centre_base = drawnCentroidX(base);
  ASSERT_GT(centre_base, 0.0) << "the grid drew nothing at the origin";

  // (1) the FRAME transform must move it.
  sink.setFrameTransform(glm::translate(glm::mat4(1.0F), glm::vec3(1.5F, 0.0F, 0.0F)));
  const QImage framed = harness_.render({&voxels}, makeContext());
  ASSERT_FALSE(framed.isNull());
  const double centre_framed = drawnCentroidX(framed);
  ASSERT_GT(centre_framed, 0.0) << "the grid drew nothing once its frame moved";
  EXPECT_GT(std::abs(centre_framed - centre_base), 10.0)
      << "setFrameTransform did not move the grid: the source frame is being ignored";

  // (2) the grid's own ORIGIN pose must move it, independently of the frame.
  sink.setFrameTransform(glm::mat4(1.0F));
  sink.setGrid(uploadAt(1.5));
  const QImage originated = harness_.render({&voxels}, makeContext());
  ASSERT_FALSE(originated.isNull());
  const double centre_originated = drawnCentroidX(originated);
  ASSERT_GT(centre_originated, 0.0) << "the grid drew nothing with a translated origin";
  EXPECT_GT(std::abs(centre_originated - centre_base), 10.0)
      << "the grid origin pose is being dropped from the model matrix";
}

// An RGBA-valued grid is refused rather than approximated (RhiVoxelGridPass draws
// scalars only). Collapsing it to a luminance scalar would render something
// plausible and wrong, so an empty view is the honest outcome.
TEST_F(RhiPassesTest, VoxelGridSinkRefusesRgbaFields) {
  RhiVoxelGridPass voxels;
  RhiVoxelGridSink sink(voxels);
  VoxelGridUpload upload;
  upload.origin.orientation.w = 1.0;
  upload.cell_size = glm::vec3(0.25F);
  upload.column_count = 4;
  upload.row_count = 4;
  upload.slice_count = 4;
  upload.kind = VoxelValueKind::kRgba;
  upload.rgba.assign(4 * 4 * 4 * 4, 200);
  sink.setGrid(std::move(upload));

  const QImage img = harness_.render({&voxels}, makeContext());
  ASSERT_FALSE(img.isNull());
  EXPECT_EQ(nonBackgroundPixels(img), 0) << "an RGBA voxel grid drew something; it should be refused outright";
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
    const int bg = backgroundLevel(img);
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

// A caster must actually darken the receiver beneath it. This is the assertion the
// whole shadow port exists to satisfy, and it is deliberately end-to-end: the depth
// map is filled by the real caster pipeline and read by the real receiver, because
// every interesting way this breaks is at the seam between them.
//
// The one genuine porting hazard is the NDC z range. The OpenGL receiver maps
// clip.z*0.5+0.5 into stored depth; on Metal clip z is already [0,1], so applying
// that remap would halve every comparison value and shadow the entire scene. The host
// supplies the mapping from QRhi::isClipDepthZeroToOne() instead — so a backend where
// that is wrong fails HERE rather than looking like a bias problem.
TEST_F(RhiPassesTest, MeshShadowDarkensTheReceiverUnderTheCaster) {
  // A wide flat receiver at z=0 and a small caster hovering above its centre.
  RhiMeshPass::DrawCall floor;
  floor.kind = RhiMeshPass::GeometryKind::kBox;
  floor.model = glm::scale(glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.0F, -0.05F)), glm::vec3(6.0F, 6.0F, 0.1F));
  floor.color = glm::vec4(0.85F, 0.85F, 0.85F, 1.0F);
  floor.use_vertex_color = false;

  RhiMeshPass::DrawCall caster;
  caster.kind = RhiMeshPass::GeometryKind::kBox;
  caster.model = glm::scale(glm::translate(glm::mat4(1.0F), glm::vec3(0.0F, 0.0F, 1.2F)), glm::vec3(1.0F));
  caster.color = glm::vec4(0.85F, 0.25F, 0.20F, 1.0F);
  caster.use_vertex_color = false;

  MeshShadingParams shading;
  shading.shadows_enabled = true;

  RhiMeshPass mesh;
  mesh.setShadingParams(shading);
  mesh.setVisualDraws({floor, caster});

  RhiShadowMapPass shadow;
  ASSERT_TRUE(shadow.ensure(harness_.rhi())) << "shadow map target could not be created";
  ASSERT_TRUE(mesh.initializeDepthOnly(harness_.rhi(), *shadow.renderPassDescriptor()));

  // Fit the light frustum with the SAME core helper the OpenGL renderer uses, then
  // apply the backend's clip-space correction — the corrected matrix both writes and
  // reads the map, which is what keeps the xy->UV mapping consistent.
  AABB caster_bounds{};
  expandAABB(caster_bounds, glm::vec3(-0.5F, -0.5F, 0.7F));
  expandAABB(caster_bounds, glm::vec3(0.5F, 0.5F, 1.7F));
  caster_bounds = extendAabbToGroundShadow(caster_bounds, shading.key_light_dir, 0.0F);
  const ShadowCameraFit fit = fitDirectionalShadowCamera(caster_bounds, shading.key_light_dir, kShadowMapSize);
  ASSERT_TRUE(fit.valid) << "the shadow camera fit failed for a plainly valid caster";
  const glm::mat4 corrected = toGlmMat(harness_.rhi().clipSpaceCorrMatrix()) * fit.light_view_proj;

  // Look straight down so the floor fills the frame and the caster occludes its
  // centre; the shadow then lands in a region the camera can see beside the caster.
  RhiFrameContext ctx;
  const glm::vec3 eye(0.0F, -0.35F, 7.0F);
  ctx.view_proj = glm::perspective(glm::radians(45.0F), static_cast<float>(kW) / kH, 0.1F, 100.0F) *
                  glm::lookAt(eye, glm::vec3(0.0F), glm::vec3(0.0F, 1.0F, 0.0F));
  ctx.camera_pos_world = eye;

  // Same scene twice: once with the map bound, once without. Differencing the two
  // isolates the shadow from every other term in the shading, so the assertion cannot
  // be satisfied by the scene merely being dark.
  mesh.setShadowMap(nullptr, glm::mat4(1.0F), 0.0F);
  harness_.setShadowSource(nullptr, nullptr);
  const QImage unshadowed = harness_.render({&mesh}, ctx);
  ASSERT_FALSE(unshadowed.isNull());

  mesh.setShadowMap(shadow.depthTexture(), corrected, fit.world_units_per_texel);
  harness_.setShadowSource(&shadow, &mesh);
  const QImage shadowed = harness_.render({&mesh}, ctx);
  harness_.setShadowSource(nullptr, nullptr);
  ASSERT_FALSE(shadowed.isNull());

  // Count pixels the shadow made materially darker, and check none got brighter by
  // more than noise: a shadow only ever subtracts light.
  int darkened = 0;
  int brightened = 0;
  for (int y = 0; y < shadowed.height(); ++y) {
    for (int x = 0; x < shadowed.width(); ++x) {
      const int before = unshadowed.pixelColor(x, y).red();
      const int after = shadowed.pixelColor(x, y).red();
      if (before - after > 12) {
        ++darkened;
      } else if (after - before > 12) {
        ++brightened;
      }
    }
  }
  EXPECT_GT(darkened, 200) << "no region darkened: the caster wrote no usable depth, or the receiver never sampled it";
  EXPECT_LT(brightened, 50) << "shadowing brightened pixels; the factor is not being applied as attenuation";

  // The caster's own TOP face fills the centre of this framing, faces the light, and
  // has nothing between it and the light — so it must be lit in both renders. This is
  // what actually pins the NDC z mapping: with the OpenGL remap wrongly applied on a
  // [0,1]-clip backend, the comparison breaks everywhere INSIDE the light frustum and
  // the caster self-shadows. A frame-wide bound cannot catch that, because the frustum
  // is fitted tightly to the caster and everything outside it returns lit regardless.
  int centre_darkened = 0;
  for (int y = (kH / 2) - 6; y <= (kH / 2) + 6; ++y) {
    for (int x = (kW / 2) - 6; x <= (kW / 2) + 6; ++x) {
      if (unshadowed.pixelColor(x, y).red() - shadowed.pixelColor(x, y).red() > 12) {
        ++centre_darkened;
      }
    }
  }
  EXPECT_EQ(centre_darkened, 0)
      << centre_darkened << " pixels of the caster's own lit top face darkened: a surface facing the light with "
      << "nothing occluding it cannot be shadowed. This is the signature of a wrong NDC-z -> stored-depth mapping "
      << "(SceneUbo::shadow_depth), which breaks the comparison across the whole light frustum.";
}

// The GPU reduction must agree with a CPU scan of the same points, exactly. Unlike
// every other case here this is checkable against ground truth rather than against
// pixels, so it is worth asserting precisely: the reduction is arithmetic, and the
// ordered-key trick that lets integer atomicMin/atomicMax order floats is the kind of
// thing that works for positives and quietly fails for negatives.
//
// Non-finite points are included on purpose, to pin that they are EXCLUDED rather
// than clamped. Note what this does not prove: removing the shader's per-workgroup
// finite guard leaves both these tests passing, because the ordered-key mapping makes
// the +/-inf seeds the identity element of each atomic. Validity is decided by the
// finite COUNT the host reads back, which is the mechanism actually under test here.
TEST_F(RhiPassesTest, GpuAabbReductionMatchesACpuScan) {
  if (!harness_.rhi().isFeatureSupported(QRhi::Compute)) {
    GTEST_SKIP() << "backend has no compute support; PointCloudLayer keeps its CPU scan";
  }

  struct Point {
    float x;
    float y;
    float z;
    float scalar;
  };
  // Deliberately spanning zero on every axis, so a sign-blind key ordering fails.
  std::vector<Point> points{
      {-3.5F, 0.25F, 7.0F, 0.0F},  {2.0F, -8.75F, -1.5F, 0.0F}, {0.0F, 0.0F, 0.0F, 0.0F},
      {11.25F, 4.0F, -6.5F, 0.0F}, {-0.5F, 9.5F, 2.25F, 0.0F},
  };
  // Must be excluded entirely, not clamped: a NaN reaching min/max would propagate.
  points.push_back({std::numeric_limits<float>::quiet_NaN(), 1.0F, 1.0F, 0.0F});
  points.push_back({std::numeric_limits<float>::infinity(), 1.0F, 1.0F, 0.0F});

  glm::vec3 cpu_lo(std::numeric_limits<float>::max());
  glm::vec3 cpu_hi(std::numeric_limits<float>::lowest());
  for (const Point& p : points) {
    if (!std::isfinite(p.x) || !std::isfinite(p.y) || !std::isfinite(p.z)) {
      continue;
    }
    cpu_lo = glm::min(cpu_lo, glm::vec3(p.x, p.y, p.z));
    cpu_hi = glm::max(cpu_hi, glm::vec3(p.x, p.y, p.z));
  }

  QRhi& rhi = harness_.rhi();
  // StorageBuffer usage is what lets compute read the cloud's own bytes rather than a
  // second copy; RhiPointcloudPass creates its buffer the same way.
  std::unique_ptr<QRhiBuffer> buf(rhi.newBuffer(
      QRhiBuffer::Static, QRhiBuffer::VertexBuffer | QRhiBuffer::StorageBuffer,
      static_cast<quint32>(points.size() * sizeof(Point))));
  ASSERT_TRUE(buf->create());

  RhiPointcloudAabbReducer reducer;
  ASSERT_TRUE(reducer.ensure(rhi)) << "compute is supported but the reducer failed to build";
  EXPECT_TRUE(reducer.probed());

  QRhiCommandBuffer* cb = nullptr;
  ASSERT_EQ(rhi.beginOffscreenFrame(&cb), QRhi::FrameOpSuccess);
  QRhiResourceUpdateBatch* upload = rhi.nextResourceUpdateBatch();
  upload->uploadStaticBuffer(buf.get(), points.data());
  cb->resourceUpdate(upload);  // applied before the compute pass reads it
  reducer.dispatch(rhi, *cb, buf.get(), static_cast<int>(points.size()), static_cast<int>(sizeof(Point)), 0);
  rhi.endOffscreenFrame();

  EXPECT_TRUE(reducer.available()) << "a successful dispatch must leave the reducer available";
  const std::optional<AABB> box = reducer.poll();
  ASSERT_TRUE(box.has_value()) << "the readback did not complete when the frame ended";
  ASSERT_TRUE(box->valid) << "the reduction reported no finite points, but five of seven are finite";

  EXPECT_FLOAT_EQ(box->min.x, cpu_lo.x);
  EXPECT_FLOAT_EQ(box->min.y, cpu_lo.y);
  EXPECT_FLOAT_EQ(box->min.z, cpu_lo.z);
  EXPECT_FLOAT_EQ(box->max.x, cpu_hi.x);
  EXPECT_FLOAT_EQ(box->max.y, cpu_hi.y);
  EXPECT_FLOAT_EQ(box->max.z, cpu_hi.z);

  // A second poll must not re-deliver: PointCloudLayer treats every result as a new
  // one and would re-fit the camera on a repeat.
  EXPECT_FALSE(reducer.poll().has_value()) << "poll() delivered the same reduction twice";
}

// An all-non-finite cloud must report "no bounds", not a garbage box built from the
// +/-inf seeds. PointCloudLayer distinguishes this from "not ready" and leaves the
// camera alone, so conflating them would fling the view to infinity.
TEST_F(RhiPassesTest, GpuAabbReductionReportsNoBoundsForANonFiniteCloud) {
  if (!harness_.rhi().isFeatureSupported(QRhi::Compute)) {
    GTEST_SKIP() << "backend has no compute support";
  }
  const float nan = std::numeric_limits<float>::quiet_NaN();
  const std::vector<float> points{nan, nan, nan, 0.0F, nan, nan, nan, 0.0F};

  QRhi& rhi = harness_.rhi();
  std::unique_ptr<QRhiBuffer> buf(rhi.newBuffer(
      QRhiBuffer::Static, QRhiBuffer::VertexBuffer | QRhiBuffer::StorageBuffer,
      static_cast<quint32>(points.size() * sizeof(float))));
  ASSERT_TRUE(buf->create());

  RhiPointcloudAabbReducer reducer;
  ASSERT_TRUE(reducer.ensure(rhi));
  QRhiCommandBuffer* cb = nullptr;
  ASSERT_EQ(rhi.beginOffscreenFrame(&cb), QRhi::FrameOpSuccess);
  QRhiResourceUpdateBatch* upload = rhi.nextResourceUpdateBatch();
  upload->uploadStaticBuffer(buf.get(), points.data());
  cb->resourceUpdate(upload);
  reducer.dispatch(rhi, *cb, buf.get(), 2, static_cast<int>(sizeof(float)) * 4, 0);
  rhi.endOffscreenFrame();

  const std::optional<AABB> box = reducer.poll();
  ASSERT_TRUE(box.has_value()) << "a reduction that found nothing must still report back";
  EXPECT_FALSE(box->valid) << "an all-NaN cloud produced a 'valid' box; the inf seeds leaked out";
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
