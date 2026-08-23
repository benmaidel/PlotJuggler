// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
//
// Self-verifying probe for the QRhi rendering path that is replacing pj_scene3D's
// hand-written OpenGL renderer.
//
// It is the smallest program that exercises every link in the new chain at once:
// a build-time-baked shader pack (qsb) -> a QRhi graphics pipeline -> a std140
// uniform block -> the platform backend (Metal on macOS, OpenGL elsewhere) ->
// framebuffer readback. If this renders, the toolchain is sound and the port can
// proceed pass by pass; if it does not, nothing further would work either.
//
// It also answers one question that decides real shader code: whether the
// presented image is Y-flipped relative to scene space. QRhi backends disagree on
// framebuffer Y direction, which is why the probe draws an ASYMMETRIC marker at
// the scene-space low-Y corner instead of a symmetric test pattern that could not
// tell a correct render from a flipped one.
//
//   ./build/pj_scene3D/demos/scene3d_rhi_probe out.png
//
// Exits non-zero if the frame could not be produced. Verdict lines go to stdout.

#include <QGuiApplication>
#include <QImage>
#include <QOffscreenSurface>
#include <QRhiWidget>
#include <QWidget>
#include <rhi/qrhi.h>

#include <QApplication>
#include <QColor>
#include <QDeadlineTimer>
#include <QFile>
#include <QTimer>
#include <cstdio>
#include <cstring>

// Resource init for a STATIC library's Qt resource. This MUST live at global
// scope: Q_INIT_RESOURCE expands to an `extern` declaration of
// qInitResources_<name>(), and inside an anonymous namespace that declaration
// gets internal linkage, so it never resolves to the library's real symbol
// (clang reports it as "has internal linkage but is not defined").
void initScene3dShaders() {
  Q_INIT_RESOURCE(scene3d_shaders);
}

namespace {

QShader loadShader(const QString& path) {
  QFile f(path);
  if (!f.open(QIODevice::ReadOnly)) {
    std::fprintf(stderr, "rhi_probe: cannot open %s\n", qPrintable(path));
    return {};
  }
  return QShader::fromSerialized(f.readAll());
}

// Mirrors the ProbeUbo block in shaders/rhi_probe.frag field for field. std140
// puts every vec4 on a 16-byte boundary; the trailing pad keeps the block size a
// multiple of 16, which is what the port's real UBOs will have to do as well.
struct alignas(16) ProbeUbo {
  float bottom_color[4];
  float top_color[4];
  float marker_color[4];
  float marker_extent[2];
  float pad0;
  float pad1;
};

class ProbeWidget : public QRhiWidget {
 public:
  ProbeWidget() {
#if defined(Q_OS_MACOS)
    setApi(Api::Metal);
#else
    setApi(Api::OpenGL);
#endif
  }

  // True once a frame has actually been submitted through a pipeline.
  [[nodiscard]] bool rendered() const { return rendered_; }
  [[nodiscard]] const char* backendName() const {
    switch (api()) {
      case Api::Metal: return "Metal";
      case Api::OpenGL: return "OpenGL";
      case Api::Vulkan: return "Vulkan";
      case Api::Direct3D11: return "D3D11";
      case Api::Direct3D12: return "D3D12";
      default: return "Null";
    }
  }

 protected:
  void initialize(QRhiCommandBuffer*) override {
    QRhi* r = rhi();
    if (r == nullptr) {
      return;
    }
    // A QRhi swap (reparent / screen change) invalidates every resource.
    if (rhi_cached_ != r) {
      releaseResources();
      rhi_cached_ = r;
    }
    if (pipeline_ != nullptr) {
      return;
    }

    const QShader vert = loadShader(QStringLiteral(":/scene3d_shaders/rhi_probe.vert.qsb"));
    const QShader frag = loadShader(QStringLiteral(":/scene3d_shaders/rhi_probe.frag.qsb"));
    if (!vert.isValid() || !frag.isValid()) {
      std::fprintf(stderr, "rhi_probe: shader pack missing or unusable for this backend\n");
      return;
    }

    ubo_ = r->newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(ProbeUbo));
    if (!ubo_->create()) {
      std::fprintf(stderr, "rhi_probe: uniform buffer creation failed\n");
      return;
    }

    srb_ = r->newShaderResourceBindings();
    srb_->setBindings({QRhiShaderResourceBinding::uniformBuffer(
        0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_)});
    if (!srb_->create()) {
      std::fprintf(stderr, "rhi_probe: shader resource bindings failed\n");
      return;
    }

    pipeline_ = r->newGraphicsPipeline();
    pipeline_->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
    pipeline_->setVertexInputLayout({});  // attributeless: position comes from gl_VertexIndex
    pipeline_->setShaderResourceBindings(srb_);
    pipeline_->setRenderPassDescriptor(renderTarget()->renderPassDescriptor());
    if (!pipeline_->create()) {
      std::fprintf(stderr, "rhi_probe: graphics pipeline creation FAILED\n");
      pipeline_ = nullptr;
      return;
    }
    std::printf("rhi_probe: backend=%s pipeline created\n", backendName());
  }

  void render(QRhiCommandBuffer* cb) override {
    if (pipeline_ == nullptr) {
      return;
    }
    QRhi* r = rhi();
    QRhiResourceUpdateBatch* updates = r->nextResourceUpdateBatch();

    ProbeUbo ubo{};
    // Dark blue at scene-space low Y, warm grey at high Y: a gradient makes a
    // flipped or degenerate frame obvious at a glance.
    const float bottom[4] = {0.05f, 0.10f, 0.35f, 1.0f};
    const float top[4] = {0.85f, 0.86f, 0.88f, 1.0f};
    const float marker[4] = {1.0f, 0.45f, 0.0f, 1.0f};
    std::memcpy(ubo.bottom_color, bottom, sizeof(bottom));
    std::memcpy(ubo.top_color, top, sizeof(top));
    std::memcpy(ubo.marker_color, marker, sizeof(marker));
    ubo.marker_extent[0] = 0.25f;
    ubo.marker_extent[1] = 0.25f;
    updates->updateDynamicBuffer(ubo_, 0, sizeof(ProbeUbo), &ubo);

    const QSize size = renderTarget()->pixelSize();
    cb->beginPass(renderTarget(), QColor(0, 0, 0), {1.0f, 0}, updates);
    cb->setGraphicsPipeline(pipeline_);
    cb->setViewport({0.0f, 0.0f, static_cast<float>(size.width()), static_cast<float>(size.height())});
    cb->setShaderResources(srb_);
    cb->draw(3);
    cb->endPass();
    rendered_ = true;
  }

  void releaseResources() override {
    delete pipeline_;
    pipeline_ = nullptr;
    delete srb_;
    srb_ = nullptr;
    delete ubo_;
    ubo_ = nullptr;
  }

 private:
  QRhi* rhi_cached_ = nullptr;
  QRhiGraphicsPipeline* pipeline_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  QRhiBuffer* ubo_ = nullptr;
  bool rendered_ = false;
};

}  // namespace

int main(int argc, char** argv) {
  QApplication app(argc, argv);
  initScene3dShaders();

  const QString out = argc > 1 ? QString::fromLocal8Bit(argv[1]) : QStringLiteral("rhi_probe.png");

  ProbeWidget widget;
  widget.resize(320, 240);
  widget.show();

  // QRhiWidget initializes lazily on its first render, which needs real
  // event-loop turns after show() — a bare processEvents() is not enough.
  QImage frame;
  QDeadlineTimer deadline(8000);
  while (!deadline.hasExpired()) {
    QCoreApplication::processEvents(QEventLoop::AllEvents, 20);
    if (widget.rendered()) {
      frame = widget.grabFramebuffer();
      if (!frame.isNull() && frame.width() > 1) {
        break;
      }
    }
  }

  if (frame.isNull() || frame.width() <= 1) {
    std::fprintf(stderr, "rhi_probe: FAILED to produce a frame (backend=%s)\n", widget.backendName());
    return 1;
  }
  if (!frame.save(out)) {
    std::fprintf(stderr, "rhi_probe: could not save %s\n", qPrintable(out));
    return 1;
  }

  // Report the corners so the Y orientation is machine-readable, not just visual.
  const QColor low_left = frame.pixelColor(frame.width() / 8, frame.height() - frame.height() / 8);
  const QColor up_left = frame.pixelColor(frame.width() / 8, frame.height() / 8);
  std::printf("rhi_probe: saved %s (%dx%d) backend=%s\n", qPrintable(out), frame.width(), frame.height(),
              widget.backendName());
  std::printf("rhi_probe: bottom-left=rgb(%d,%d,%d) top-left=rgb(%d,%d,%d)\n", low_left.red(), low_left.green(),
              low_left.blue(), up_left.red(), up_left.green(), up_left.blue());
  return 0;
}
