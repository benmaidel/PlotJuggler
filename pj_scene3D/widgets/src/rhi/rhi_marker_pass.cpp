// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_marker_pass.h"

#include <QFile>
#include <QLatin1StringView>
#include <QLoggingCategory>
#include <algorithm>
#include <cmath>
#include <cstring>
#include <glm/gtc/matrix_transform.hpp>
#include <numbers>
#include <utility>

#include "pj_scene3d_widgets/gizmos/arrow_mesh.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiMarker, "pj.scene3d.rhi.marker")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiMarker) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

// Cube-edge shading constants, matching the GL pass.
constexpr float kEdgeDarken = 0.6F;
constexpr float kHiddenEdgeMix = 0.45F;
constexpr float kEdgeVisibleEpsilon = 1e-4F;

/// Line indices for a triangle index list: every triangle contributes its three
/// edges. Shared edges are emitted twice, which costs a few duplicate lines but
/// avoids a dedup pass and is invisible in the result.
std::vector<std::uint32_t> trianglesToLines(const std::vector<std::uint32_t>& tris) {
  std::vector<std::uint32_t> lines;
  lines.reserve(tris.size() * 2);
  for (std::size_t i = 0; i + 2 < tris.size(); i += 3) {
    lines.insert(lines.end(), {tris[i], tris[i + 1], tris[i + 1], tris[i + 2], tris[i + 2], tris[i]});
  }
  return lines;
}

/// Unit cube, side 1 (+/-0.5), 24 vertices so each face carries its own flat
/// normal. Interleaved pos.xyz + normal.xyz. Faces wind CCW seen from OUTSIDE,
/// which the back-face culling on closed solids depends on.
void buildCube(std::vector<float>& verts, std::vector<std::uint32_t>& indices) {
  constexpr float kV[24 * 6] = {
      0.5F,  -0.5F, -0.5F, 1,  0,  0,  0.5F,  0.5F,  -0.5F, 1,  0,  0,  0.5F,  0.5F,  0.5F,  1,  0,  0,
      0.5F,  -0.5F, 0.5F,  1,  0,  0,  -0.5F, -0.5F, 0.5F,  -1, 0,  0,  -0.5F, 0.5F,  0.5F,  -1, 0,  0,
      -0.5F, 0.5F,  -0.5F, -1, 0,  0,  -0.5F, -0.5F, -0.5F, -1, 0,  0,  -0.5F, 0.5F,  -0.5F, 0,  1,  0,
      -0.5F, 0.5F,  0.5F,  0,  1,  0,  0.5F,  0.5F,  0.5F,  0,  1,  0,  0.5F,  0.5F,  -0.5F, 0,  1,  0,
      -0.5F, -0.5F, 0.5F,  0,  -1, 0,  -0.5F, -0.5F, -0.5F, 0,  -1, 0,  0.5F,  -0.5F, -0.5F, 0,  -1, 0,
      0.5F,  -0.5F, 0.5F,  0,  -1, 0,  -0.5F, -0.5F, 0.5F,  0,  0,  1,  0.5F,  -0.5F, 0.5F,  0,  0,  1,
      0.5F,  0.5F,  0.5F,  0,  0,  1,  -0.5F, 0.5F,  0.5F,  0,  0,  1,  0.5F,  -0.5F, -0.5F, 0,  0,  -1,
      -0.5F, -0.5F, -0.5F, 0,  0,  -1, -0.5F, 0.5F,  -0.5F, 0,  0,  -1, 0.5F,  0.5F,  -0.5F, 0,  0,  -1,
  };
  verts.assign(std::begin(kV), std::end(kV));
  indices = {0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
             12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23};
}

/// Unit UV sphere, radius 0.5 so it matches the cube's extent under the same
/// model scale. Normal is the position direction; winds CCW from outside.
void buildSphere(std::vector<float>& verts, std::vector<std::uint32_t>& indices, int rings, int sectors) {
  constexpr float kRadius = 0.5F;
  for (int r = 0; r <= rings; ++r) {
    const float phi = std::numbers::pi_v<float> * static_cast<float>(r) / static_cast<float>(rings);
    const float y = std::cos(phi);
    const float ring_r = std::sin(phi);
    for (int s = 0; s <= sectors; ++s) {
      const float theta = 2.0F * std::numbers::pi_v<float> * static_cast<float>(s) / static_cast<float>(sectors);
      const float x = ring_r * std::cos(theta);
      const float z = ring_r * std::sin(theta);
      verts.insert(verts.end(), {kRadius * x, kRadius * y, kRadius * z, x, y, z});
    }
  }
  const auto stride = static_cast<std::uint32_t>(sectors + 1);
  for (std::uint32_t r = 0; r < static_cast<std::uint32_t>(rings); ++r) {
    for (std::uint32_t s = 0; s < static_cast<std::uint32_t>(sectors); ++s) {
      const std::uint32_t a = (r * stride) + s;
      const std::uint32_t b = a + stride;
      indices.insert(indices.end(), {a, a + 1, b, a + 1, b + 1, b});
    }
  }
}

/// Unit cylinder: axis +Z, radius 0.5, height 1. Stride-7 vertices (pos.xyz,
/// normal.xyz, taper_w) — taper_w is 0 on the bottom ring and 1 on the top, and is
/// what lets the vertex stage collapse each end independently.
void buildCylinder(std::vector<float>& verts, std::vector<std::uint32_t>& indices, int seg) {
  constexpr float kRadius = 0.5F;
  constexpr float kHalfH = 0.5F;
  const auto push = [&](float px, float py, float pz, float nx, float ny, float nz, float tw) {
    verts.insert(verts.end(), {px, py, pz, nx, ny, nz, tw});
  };
  const auto circle = [&](int s) {
    const float th = 2.0F * std::numbers::pi_v<float> * static_cast<float>(s) / static_cast<float>(seg);
    return std::pair<float, float>{std::cos(th), std::sin(th)};
  };

  for (int s = 0; s <= seg; ++s) {
    const auto [c, sn] = circle(s);
    push(kRadius * c, kRadius * sn, -kHalfH, c, sn, 0.0F, 0.0F);
  }
  for (int s = 0; s <= seg; ++s) {
    const auto [c, sn] = circle(s);
    push(kRadius * c, kRadius * sn, kHalfH, c, sn, 0.0F, 1.0F);
  }
  const auto ring = static_cast<std::uint32_t>(seg + 1);
  for (std::uint32_t s = 0; s < static_cast<std::uint32_t>(seg); ++s) {
    indices.insert(indices.end(), {s, s + 1, ring + s, ring + s, s + 1, ring + s + 1});
  }

  const auto bottom_center = static_cast<std::uint32_t>(verts.size() / 7);
  push(0.0F, 0.0F, -kHalfH, 0.0F, 0.0F, -1.0F, 0.0F);
  for (int s = 0; s <= seg; ++s) {
    const auto [c, sn] = circle(s);
    push(kRadius * c, kRadius * sn, -kHalfH, 0.0F, 0.0F, -1.0F, 0.0F);
  }
  for (std::uint32_t s = 0; s < static_cast<std::uint32_t>(seg); ++s) {
    indices.insert(indices.end(), {bottom_center, bottom_center + s + 2, bottom_center + s + 1});
  }

  const auto top_center = static_cast<std::uint32_t>(verts.size() / 7);
  push(0.0F, 0.0F, kHalfH, 0.0F, 0.0F, 1.0F, 1.0F);
  for (int s = 0; s <= seg; ++s) {
    const auto [c, sn] = circle(s);
    push(kRadius * c, kRadius * sn, kHalfH, 0.0F, 0.0F, 1.0F, 1.0F);
  }
  for (std::uint32_t s = 0; s < static_cast<std::uint32_t>(seg); ++s) {
    indices.insert(indices.end(), {top_center, top_center + s + 1, top_center + s + 2});
  }
}

/// The 12 cube edges as 24 line endpoints. Each endpoint carries pos.xyz, the two
/// adjacent outward face normals, and the edge centre — 12 floats — which is what
/// the edge shader needs to tell a front edge from a rear one.
std::vector<float> buildCubeEdges() {
  struct Edge {
    glm::vec3 a;
    glm::vec3 b;
    glm::vec3 na;
    glm::vec3 nb;
  };
  constexpr float h = 0.5F;
  const std::vector<Edge> edges = {
      // Bottom face ring (-Z), each shared with one side face.
      {{-h, -h, -h}, {h, -h, -h}, {0, 0, -1}, {0, -1, 0}},
      {{h, -h, -h}, {h, h, -h}, {0, 0, -1}, {1, 0, 0}},
      {{h, h, -h}, {-h, h, -h}, {0, 0, -1}, {0, 1, 0}},
      {{-h, h, -h}, {-h, -h, -h}, {0, 0, -1}, {-1, 0, 0}},
      // Top face ring (+Z).
      {{-h, -h, h}, {h, -h, h}, {0, 0, 1}, {0, -1, 0}},
      {{h, -h, h}, {h, h, h}, {0, 0, 1}, {1, 0, 0}},
      {{h, h, h}, {-h, h, h}, {0, 0, 1}, {0, 1, 0}},
      {{-h, h, h}, {-h, -h, h}, {0, 0, 1}, {-1, 0, 0}},
      // Four verticals, each shared by two side faces.
      {{-h, -h, -h}, {-h, -h, h}, {-1, 0, 0}, {0, -1, 0}},
      {{h, -h, -h}, {h, -h, h}, {1, 0, 0}, {0, -1, 0}},
      {{h, h, -h}, {h, h, h}, {1, 0, 0}, {0, 1, 0}},
      {{-h, h, -h}, {-h, h, h}, {-1, 0, 0}, {0, 1, 0}},
  };

  std::vector<float> out;
  out.reserve(edges.size() * 2 * 12);
  for (const Edge& edge : edges) {
    const glm::vec3 center = (edge.a + edge.b) * 0.5F;
    for (const glm::vec3& p : {edge.a, edge.b}) {
      out.insert(
          out.end(), {p.x, p.y, p.z, edge.na.x, edge.na.y, edge.na.z, edge.nb.x, edge.nb.y, edge.nb.z, center.x,
                      center.y, center.z});
    }
  }
  return out;
}

constexpr int kSphereRings = 16;
constexpr int kSphereSectors = 24;
constexpr int kCylinderSegments = 32;
constexpr std::uint32_t kCubeEdgeVertexCount = 24;

}  // namespace

RhiMarkerPass::RhiMarkerPass() {
  buildCube(cube_.vertices, cube_.tri_indices);
  buildSphere(sphere_.vertices, sphere_.tri_indices, kSphereRings, kSphereSectors);
  buildCylinder(cylinder_.vertices, cylinder_.tri_indices, kCylinderSegments);
  {
    // The unit arrow points +X, matching MarkerArrow's identity orientation; each
    // marker's dimensions are baked into its instance matrix, never re-meshed.
    ArrowMeshParams params;
    params.length = 1.0F;
    const ArrowMeshData mesh = buildArrowMesh(params);
    arrow_.vertices = mesh.vertices;
    arrow_.tri_indices = mesh.indices;
  }
  for (UnitMesh* mesh : {&cube_, &sphere_, &cylinder_, &arrow_}) {
    mesh->line_indices = trianglesToLines(mesh->tri_indices);
  }
}

RhiMarkerPass::~RhiMarkerPass() {
  release();
}

void RhiMarkerPass::setActive(std::shared_ptr<const DecodedSceneEntities> markers) {
  markers_ = std::move(markers);
}

void RhiMarkerPass::setFrameTransforms(std::vector<std::optional<glm::mat4>> fixed_from_frame) {
  frame_world_ = std::move(fixed_from_frame);
}

void RhiMarkerPass::setOverrides(const DisplayOverrides& overrides) {
  overrides_ = overrides;
}

void RhiMarkerPass::setVisible(bool visible) {
  visible_ = visible;
}

glm::vec4 RhiMarkerPass::applyOverride(const glm::vec4& color) const {
  glm::vec4 out = overrides_.color_override ? overrides_.override_color : color;
  out.a *= overrides_.opacity;
  return out;
}

std::optional<glm::mat4> RhiMarkerPass::worldOf(std::uint32_t frame_index, const glm::mat4& model) const {
  if (frame_index >= frame_world_.size() || !frame_world_[frame_index].has_value()) {
    return std::nullopt;
  }
  return *frame_world_[frame_index] * model;
}

bool RhiMarkerPass::ensureBuffer(QRhiBuffer*& buffer, int& capacity_bytes, int bytes, QRhiBuffer::UsageFlags usage) {
  if (bytes <= 0) {
    return true;
  }
  if (buffer != nullptr && bytes <= capacity_bytes) {
    return true;
  }
  delete buffer;
  capacity_bytes = std::max(bytes * 2, 4096);
  buffer = rhi_->newBuffer(QRhiBuffer::Dynamic, usage, static_cast<quint32>(capacity_bytes));
  if (buffer == nullptr || !buffer->create()) {
    capacity_bytes = 0;
    buffer = nullptr;
    return false;
  }
  return true;
}

QRhiGraphicsPipeline* RhiMarkerPass::pipelineFor(const PipelineKey& key) {
  const auto it =
      std::find_if(pipelines_.begin(), pipelines_.end(), [&](const auto& entry) { return entry.first == key; });
  if (it != pipelines_.end()) {
    return it->second;
  }
  if (rhi_ == nullptr || rpd_ == nullptr || srb_ == nullptr) {
    return nullptr;
  }

  QString vert_path;
  switch (key.program) {
    case Program::kSolid:
      vert_path = QStringLiteral(":/scene3d_shaders/marker_solid.vert.qsb");
      break;
    case Program::kCylinder:
      vert_path = QStringLiteral(":/scene3d_shaders/marker_cylinder.vert.qsb");
      break;
    case Program::kEdge:
      vert_path = QStringLiteral(":/scene3d_shaders/marker_edge.vert.qsb");
      break;
    case Program::kFlat:
      vert_path = QStringLiteral(":/scene3d_shaders/marker_flat.vert.qsb");
      break;
    case Program::kArrow:
      vert_path = QLatin1StringView(arrow::kVertShader);
      break;
  }
  // Arrows are the one lit marker geometry: the GL pass draws them through
  // ArrowGizmo, which shades them, so they keep the shared arrow fragment shader
  // while everything else is flat.
  const QString frag_path = key.program == Program::kArrow ? QString(QLatin1StringView(arrow::kFragShader))
                                                           : QStringLiteral(":/scene3d_shaders/marker_flat.frag.qsb");

  const QShader vert = loadBakedShader(vert_path);
  const QShader frag = loadBakedShader(frag_path);
  if (!vert.isValid() || !frag.isValid()) {
    return nullptr;
  }

  QRhiVertexInputLayout layout;
  using Attr = QRhiVertexInputAttribute;
  const auto kInstance = QRhiVertexInputBinding::PerInstance;
  switch (key.program) {
    case Program::kSolid:
      // Binding 0: pos + normal (the flat shader ignores the normal, but the mesh
      // is shared with the wireframe and edge paths, so the stride stays).
      layout.setBindings(
          {QRhiVertexInputBinding(6 * sizeof(float)), QRhiVertexInputBinding(sizeof(Instance), kInstance)});
      layout.setAttributes({
          Attr(0, 0, Attr::Float3, 0),
          Attr(1, 2, Attr::Float4, 0),
          Attr(1, 3, Attr::Float4, 4 * sizeof(float)),
          Attr(1, 4, Attr::Float4, 8 * sizeof(float)),
          Attr(1, 5, Attr::Float4, 12 * sizeof(float)),
          Attr(1, 6, Attr::Float4, 16 * sizeof(float)),
      });
      break;
    case Program::kArrow:
      layout.setBindings(
          {QRhiVertexInputBinding(6 * sizeof(float)), QRhiVertexInputBinding(sizeof(Instance), kInstance)});
      layout.setAttributes({
          Attr(0, 0, Attr::Float3, 0),
          Attr(0, 1, Attr::Float3, 3 * sizeof(float)),
          Attr(1, 2, Attr::Float4, 0),
          Attr(1, 3, Attr::Float4, 4 * sizeof(float)),
          Attr(1, 4, Attr::Float4, 8 * sizeof(float)),
          Attr(1, 5, Attr::Float4, 12 * sizeof(float)),
          Attr(1, 6, Attr::Float4, 16 * sizeof(float)),
      });
      break;
    case Program::kCylinder:
      // Binding 0: pos + normal + taper_w (stride 7 floats).
      layout.setBindings(
          {QRhiVertexInputBinding(7 * sizeof(float)), QRhiVertexInputBinding(sizeof(CylinderInstance), kInstance)});
      layout.setAttributes({
          Attr(0, 0, Attr::Float3, 0),
          Attr(0, 2, Attr::Float, 6 * sizeof(float)),
          Attr(1, 3, Attr::Float4, 0),
          Attr(1, 4, Attr::Float4, 4 * sizeof(float)),
          Attr(1, 5, Attr::Float4, 8 * sizeof(float)),
          Attr(1, 6, Attr::Float4, 12 * sizeof(float)),
          Attr(1, 7, Attr::Float4, 16 * sizeof(float)),
          Attr(1, 8, Attr::Float2, 20 * sizeof(float)),
      });
      break;
    case Program::kEdge:
      // Binding 0: pos + normal_a + normal_b + edge_center (stride 12 floats).
      layout.setBindings(
          {QRhiVertexInputBinding(12 * sizeof(float)), QRhiVertexInputBinding(sizeof(Instance), kInstance)});
      layout.setAttributes({
          Attr(0, 0, Attr::Float3, 0),
          Attr(0, 1, Attr::Float3, 3 * sizeof(float)),
          Attr(0, 7, Attr::Float3, 6 * sizeof(float)),
          Attr(0, 8, Attr::Float3, 9 * sizeof(float)),
          Attr(1, 2, Attr::Float4, 0),
          Attr(1, 3, Attr::Float4, 4 * sizeof(float)),
          Attr(1, 4, Attr::Float4, 8 * sizeof(float)),
          Attr(1, 5, Attr::Float4, 12 * sizeof(float)),
          Attr(1, 6, Attr::Float4, 16 * sizeof(float)),
      });
      break;
    case Program::kFlat:
      layout.setBindings({QRhiVertexInputBinding(sizeof(FlatVertex))});
      layout.setAttributes({
          Attr(0, 0, Attr::Float3, 0),
          Attr(0, 1, Attr::Float4, 3 * sizeof(float)),
      });
      break;
  }

  QRhiGraphicsPipeline* pipeline = rhi_->newGraphicsPipeline();
  pipeline->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
  pipeline->setVertexInputLayout(layout);
  pipeline->setShaderResourceBindings(srb_);
  pipeline->setRenderPassDescriptor(rpd_);
  pipeline->setSampleCount(sample_count_);
  pipeline->setTopology(key.lines ? QRhiGraphicsPipeline::Lines : QRhiGraphicsPipeline::Triangles);
  pipeline->setDepthTest(true);
  pipeline->setDepthWrite(key.depth_write);
  // Closed CCW-outward solids cull back faces so a translucent one tints each pixel
  // through exactly ONE face, instead of stacking front and back alpha in an
  // order-dependent way. Triangle lists (arbitrary user winding), lines and arrows
  // are never culled.
  pipeline->setCullMode(key.cull_back ? QRhiGraphicsPipeline::Back : QRhiGraphicsPipeline::None);
  pipeline->setFrontFace(QRhiGraphicsPipeline::CCW);
  if (key.depth_bias) {
    pipeline->setDepthBias(1);
    pipeline->setSlopeScaledDepthBias(1.0F);
  }
  if (key.blend) {
    QRhiGraphicsPipeline::TargetBlend blend;
    blend.enable = true;
    blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
    blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    // Coverage-union on alpha, matching the GL pass: dstA' = srcA + dstA*(1-srcA).
    blend.srcAlpha = QRhiGraphicsPipeline::One;
    blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
    pipeline->setTargetBlends({blend});
  }
  if (!pipeline->create()) {
    qCWarning(lcRhiMarker) << "marker pipeline creation failed";
    delete pipeline;
    return nullptr;
  }
  pipelines_.emplace_back(key, pipeline);
  return pipeline;
}

bool RhiMarkerPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (srb_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count && rpd_ == &rpd) {
    return true;
  }
  release();
  rhi_ = &rhi;
  rpd_ = &rpd;
  sample_count_ = sample_count;

  ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(MarkerUbo));
  if (!ubo_->create()) {
    release();
    return false;
  }
  // Every marker shader — and the shared arrow shader — declares one uniform block
  // at binding 0, so a single binding set serves every pipeline in the cache.
  srb_ = rhi.newShaderResourceBindings();
  srb_->setBindings({QRhiShaderResourceBinding::uniformBuffer(
      0, QRhiShaderResourceBinding::VertexStage | QRhiShaderResourceBinding::FragmentStage, ubo_)});
  if (!srb_->create()) {
    release();
    return false;
  }
  return true;
}

void RhiMarkerPass::uploadMeshIfNeeded(QRhiResourceUpdateBatch& updates, UnitMesh& mesh) {
  if (mesh.uploaded) {
    return;
  }
  mesh.uploaded = true;
  const auto vbytes = static_cast<quint32>(mesh.vertices.size() * sizeof(float));
  const auto tbytes = static_cast<quint32>(mesh.tri_indices.size() * sizeof(std::uint32_t));
  const auto lbytes = static_cast<quint32>(mesh.line_indices.size() * sizeof(std::uint32_t));
  mesh.vbo = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, vbytes);
  mesh.tri_ibo = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, tbytes);
  mesh.line_ibo = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, lbytes);
  if (mesh.vbo == nullptr || mesh.tri_ibo == nullptr || mesh.line_ibo == nullptr || !mesh.vbo->create() ||
      !mesh.tri_ibo->create() || !mesh.line_ibo->create()) {
    return;
  }
  updates.uploadStaticBuffer(mesh.vbo, mesh.vertices.data());
  updates.uploadStaticBuffer(mesh.tri_ibo, mesh.tri_indices.data());
  updates.uploadStaticBuffer(mesh.line_ibo, mesh.line_indices.data());
  mesh.tri_index_count = static_cast<std::uint32_t>(mesh.tri_indices.size());
  mesh.line_index_count = static_cast<std::uint32_t>(mesh.line_indices.size());
}

void RhiMarkerPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  draws_.clear();
  instance_staging_.clear();
  cylinder_staging_.clear();
  flat_staging_.clear();
  if (srb_ == nullptr || rhi_ == nullptr) {
    return;
  }

  MarkerUbo ubo{};
  std::memcpy(ubo.view_proj, &ctx.view_proj[0][0], sizeof(ubo.view_proj));
  // Identity: primitives are already in world space by the time they are staged.
  const glm::mat4 identity(1.0F);
  std::memcpy(ubo.frame_world, &identity[0][0], sizeof(ubo.frame_world));
  ubo.edge_params[0] = kEdgeDarken;
  ubo.edge_params[1] = kHiddenEdgeMix;
  ubo.edge_params[2] = kEdgeVisibleEpsilon;
  ubo.camera_pos[0] = ctx.camera_pos_world.x;
  ubo.camera_pos[1] = ctx.camera_pos_world.y;
  ubo.camera_pos[2] = ctx.camera_pos_world.z;
  updates.updateDynamicBuffer(ubo_, 0, sizeof(MarkerUbo), &ubo);

  if (!visible_ || markers_ == nullptr || markers_->empty()) {
    return;
  }
  const DecodedSceneEntities& batch = *markers_;

  for (UnitMesh* mesh : {&cube_, &sphere_, &cylinder_, &arrow_}) {
    uploadMeshIfNeeded(updates, *mesh);
  }
  if (!edge_uploaded_) {
    edge_uploaded_ = true;
    const std::vector<float> edges = buildCubeEdges();
    edge_vbo_ = rhi_->newBuffer(
        QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, static_cast<quint32>(edges.size() * sizeof(float)));
    if (edge_vbo_ != nullptr && edge_vbo_->create()) {
      updates.uploadStaticBuffer(edge_vbo_, edges.data());
    }
  }

  // A whole-batch override alpha means every primitive is see-through, so nothing
  // may write depth or the batch would occlude itself in draw order.
  const bool batch_translucent =
      overrides_.opacity < 0.999F || (overrides_.color_override && overrides_.override_color.a < 0.999F);
  const bool wireframe = overrides_.wireframe;

  const auto pushInstances = [this](const std::vector<Instance>& list) {
    const auto offset = static_cast<std::uint32_t>(instance_staging_.size());
    const std::size_t bytes = list.size() * sizeof(Instance);
    instance_staging_.resize(instance_staging_.size() + bytes);
    std::memcpy(instance_staging_.data() + offset, list.data(), bytes);
    return offset;
  };

  const auto buildSolids = [&](const std::vector<MarkerSolid>& list) {
    std::vector<Instance> out;
    out.reserve(list.size());
    for (const MarkerSolid& prim : list) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      Instance inst{};
      std::memcpy(inst.world, &(*world)[0][0], sizeof(inst.world));
      const glm::vec4 color = applyOverride(prim.color);
      std::memcpy(inst.color, &color[0], sizeof(inst.color));
      out.push_back(inst);
    }
    return out;
  };

  // --- Cubes: fill, then the edge overlay replaying the same instances ---
  const std::vector<Instance> cubes = buildSolids(batch.cubes);
  if (!cubes.empty()) {
    const bool any_translucent =
        std::any_of(cubes.begin(), cubes.end(), [](const Instance& inst) { return inst.color[3] < 0.999F; });
    const std::uint32_t offset = pushInstances(cubes);
    // The fill is depth-biased away from the camera so the edge lines, drawn at the
    // exact un-biased depth, win the depth test instead of z-fighting the box's own
    // faces. Biasing the fill rather than the lines keeps line depth exact against
    // the rest of the scene.
    //
    // A translucent fill additionally writes no depth: the bias only settles a
    // fight at a shared surface and cannot bridge the box's depth extent, so a
    // depth-writing front face would hide the rear edges exactly when the box is
    // see-through. Opaque fills keep writing depth — rear edges hidden by a solid
    // box is correct occlusion, not an artifact.
    DrawItem fill;
    fill.key =
        PipelineKey{Program::kSolid, wireframe, !wireframe, !(batch_translucent || any_translucent), true, !wireframe};
    fill.mesh = &cube_;
    fill.buffer_offset = offset;
    fill.instance_count = static_cast<std::uint32_t>(cubes.size());
    fill.indexed = true;
    fill.lines = wireframe;
    draws_.push_back(fill);

    // Edges are opaque (no blend) and never culled; they are the readability aid.
    // In wireframe the fill is already lines, so the overlay would be redundant.
    if (!wireframe && edge_vbo_ != nullptr) {
      DrawItem edge;
      edge.key = PipelineKey{Program::kEdge, true, false, true, false, false};
      edge.mesh = nullptr;
      edge.buffer_offset = offset;
      edge.instance_count = static_cast<std::uint32_t>(cubes.size());
      edge.vertex_count = kCubeEdgeVertexCount;
      edge.indexed = false;
      edge.lines = true;
      draws_.push_back(edge);
    }
  }

  // --- Spheres ---
  const std::vector<Instance> spheres = buildSolids(batch.spheres);
  if (!spheres.empty()) {
    DrawItem item;
    item.key = PipelineKey{Program::kSolid, wireframe, !wireframe, !batch_translucent, true, false};
    item.mesh = &sphere_;
    item.buffer_offset = pushInstances(spheres);
    item.instance_count = static_cast<std::uint32_t>(spheres.size());
    item.lines = wireframe;
    draws_.push_back(item);
  }

  // --- Arrows and axes: one instanced arrow mesh, dimensions baked per instance ---
  {
    std::vector<Instance> arrows;
    arrows.reserve(batch.arrows.size() + (batch.axes.size() * 3));
    const auto push = [&](const glm::mat4& world, const glm::vec4& color) {
      Instance inst{};
      std::memcpy(inst.world, &world[0][0], sizeof(inst.world));
      std::memcpy(inst.color, &color[0], sizeof(inst.color));
      arrows.push_back(inst);
    };
    for (const MarkerArrow& prim : batch.arrows) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      // The unit mesh is a +X arrow of length 1; a non-uniform scale stretches it to
      // the marker's shaft length and diameters.
      const float length = prim.shaft_length + prim.head_length;
      const float girth = std::max(prim.shaft_diameter, prim.head_diameter);
      push(*world * glm::scale(glm::mat4(1.0F), glm::vec3(length, girth, girth)), applyOverride(prim.color));
    }
    // Axes glyphs: three arms in the fixed X/Y/Z colours, which the override still
    // recolours (an override means "make this topic one colour", axes included).
    constexpr glm::vec4 kAxisColors[3] = {
        {0.90F, 0.15F, 0.15F, 1.0F}, {0.15F, 0.75F, 0.20F, 1.0F}, {0.20F, 0.35F, 0.95F, 1.0F}};
    for (const MarkerAxes& prim : batch.axes) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      for (int axis = 0; axis < 3; ++axis) {
        glm::mat4 rotate(1.0F);
        if (axis == 1) {
          rotate = glm::rotate(glm::mat4(1.0F), glm::radians(90.0F), glm::vec3(0.0F, 0.0F, 1.0F));
        } else if (axis == 2) {
          rotate = glm::rotate(glm::mat4(1.0F), glm::radians(-90.0F), glm::vec3(0.0F, 1.0F, 0.0F));
        }
        const glm::mat4 scale = glm::scale(glm::mat4(1.0F), glm::vec3(prim.length, prim.thickness, prim.thickness));
        push(*world * rotate * scale, applyOverride(kAxisColors[axis]));
      }
    }
    if (!arrows.empty()) {
      DrawItem item;
      item.key = PipelineKey{Program::kArrow, false, false, !batch_translucent, true, false};
      item.mesh = &arrow_;
      item.buffer_offset = pushInstances(arrows);
      item.instance_count = static_cast<std::uint32_t>(arrows.size());
      draws_.push_back(item);
    }
  }

  // --- Cylinders / cones ---
  {
    std::vector<CylinderInstance> cyls;
    cyls.reserve(batch.cylinders.size());
    for (const MarkerCylinder& prim : batch.cylinders) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      CylinderInstance inst{};
      std::memcpy(inst.world, &(*world)[0][0], sizeof(inst.world));
      const glm::vec4 color = applyOverride(prim.color);
      std::memcpy(inst.color, &color[0], sizeof(inst.color));
      inst.taper[0] = prim.bottom_scale;
      inst.taper[1] = prim.top_scale;
      cyls.push_back(inst);
    }
    if (!cyls.empty()) {
      const auto offset = static_cast<std::uint32_t>(cylinder_staging_.size());
      const std::size_t bytes = cyls.size() * sizeof(CylinderInstance);
      cylinder_staging_.resize(bytes);
      std::memcpy(cylinder_staging_.data() + offset, cyls.data(), bytes);

      DrawItem item;
      item.key = PipelineKey{Program::kCylinder, wireframe, !wireframe, !batch_translucent, true, false};
      item.mesh = &cylinder_;
      item.buffer_offset = offset;
      item.instance_count = static_cast<std::uint32_t>(cyls.size());
      item.lines = wireframe;
      draws_.push_back(item);
    }
  }

  // --- Line and triangle batches: world-baked, merged into one draw each ---
  {
    const auto stage = [&](const glm::mat4& world, const std::vector<glm::vec3>& verts,
                           const std::vector<glm::vec4>& colors, const glm::vec4& fallback) {
      for (std::size_t i = 0; i < verts.size(); ++i) {
        const glm::vec3 p = glm::vec3(world * glm::vec4(verts[i], 1.0F));
        const glm::vec4 c = applyOverride(i < colors.size() ? colors[i] : fallback);
        flat_staging_.push_back(FlatVertex{{p.x, p.y, p.z}, {c.r, c.g, c.b, c.a}});
      }
    };

    const auto line_start = static_cast<std::uint32_t>(flat_staging_.size());
    for (const MarkerLineBatch& prim : batch.lines) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      // Vertices are already expanded to segment pairs by the decoder. Note that
      // MarkerLineBatch::thickness cannot be honoured: Metal has no wide-line
      // primitive, and a core-profile GL context rejects glLineWidth > 1 as well,
      // so the GL pass ignores it too.
      stage(*world, prim.vertices, prim.colors, prim.color);
    }
    const auto line_count = static_cast<std::uint32_t>(flat_staging_.size()) - line_start;
    if (line_count > 0) {
      DrawItem item;
      item.key = PipelineKey{Program::kFlat, true, false, !batch_translucent, true, false};
      item.buffer_offset = line_start * static_cast<std::uint32_t>(sizeof(FlatVertex));
      item.vertex_count = line_count;
      item.instance_count = 1;
      item.indexed = false;
      item.lines = true;
      draws_.push_back(item);
    }

    const auto tri_start = static_cast<std::uint32_t>(flat_staging_.size());
    for (const MarkerTriangleBatch& prim : batch.triangles) {
      const std::optional<glm::mat4> world = worldOf(prim.frame_index, prim.model);
      if (!world.has_value()) {
        continue;
      }
      if (!wireframe) {
        stage(*world, prim.vertices, prim.colors, prim.color);
        continue;
      }
      // Wireframe: expand each triangle into its three segments here rather than
      // through a polygon mode, since QRhi has none. These vertices are re-streamed
      // every frame anyway, so changing topology on the CPU costs nothing extra.
      for (std::size_t i = 0; i + 2 < prim.vertices.size(); i += 3) {
        constexpr int kEdges[3][2] = {{0, 1}, {1, 2}, {2, 0}};
        for (const auto& edge : kEdges) {
          for (const int corner : edge) {
            const std::size_t v = i + static_cast<std::size_t>(corner);
            const glm::vec3 p = glm::vec3(*world * glm::vec4(prim.vertices[v], 1.0F));
            const glm::vec4 c = applyOverride(v < prim.colors.size() ? prim.colors[v] : prim.color);
            flat_staging_.push_back(FlatVertex{{p.x, p.y, p.z}, {c.r, c.g, c.b, c.a}});
          }
        }
      }
    }
    const auto tri_count = static_cast<std::uint32_t>(flat_staging_.size()) - tri_start;
    if (tri_count > 0) {
      DrawItem item;
      // Triangle batches carry arbitrary user winding, so they are never culled.
      item.key = PipelineKey{Program::kFlat, wireframe, false, !batch_translucent, true, false};
      item.buffer_offset = tri_start * static_cast<std::uint32_t>(sizeof(FlatVertex));
      item.vertex_count = tri_count;
      item.instance_count = 1;
      item.indexed = false;
      item.lines = wireframe;
      draws_.push_back(item);
    }
  }

  const auto instance_bytes = static_cast<int>(instance_staging_.size());
  const auto cylinder_bytes = static_cast<int>(cylinder_staging_.size());
  const auto flat_bytes = static_cast<int>(flat_staging_.size() * sizeof(FlatVertex));
  if (!ensureBuffer(instance_buf_, instance_capacity_, instance_bytes, QRhiBuffer::VertexBuffer) ||
      !ensureBuffer(cylinder_buf_, cylinder_capacity_, cylinder_bytes, QRhiBuffer::VertexBuffer) ||
      !ensureBuffer(flat_buf_, flat_capacity_, flat_bytes, QRhiBuffer::VertexBuffer)) {
    draws_.clear();
    return;
  }
  if (instance_bytes > 0) {
    updates.updateDynamicBuffer(instance_buf_, 0, static_cast<quint32>(instance_bytes), instance_staging_.data());
  }
  if (cylinder_bytes > 0) {
    updates.updateDynamicBuffer(cylinder_buf_, 0, static_cast<quint32>(cylinder_bytes), cylinder_staging_.data());
  }
  if (flat_bytes > 0) {
    updates.updateDynamicBuffer(flat_buf_, 0, static_cast<quint32>(flat_bytes), flat_staging_.data());
  }
}

void RhiMarkerPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (draws_.empty()) {
    return;
  }
  for (const DrawItem& item : draws_) {
    QRhiGraphicsPipeline* pipeline = pipelineFor(item.key);
    if (pipeline == nullptr) {
      continue;
    }
    cb.setGraphicsPipeline(pipeline);
    cb.setShaderResources(srb_);

    switch (item.key.program) {
      case Program::kFlat: {
        if (flat_buf_ == nullptr) {
          continue;
        }
        const QRhiCommandBuffer::VertexInput input{flat_buf_, item.buffer_offset};
        cb.setVertexInput(0, 1, &input);
        cb.draw(item.vertex_count);
        break;
      }
      case Program::kEdge: {
        if (edge_vbo_ == nullptr || instance_buf_ == nullptr) {
          continue;
        }
        const QRhiCommandBuffer::VertexInput inputs[] = {{edge_vbo_, 0}, {instance_buf_, item.buffer_offset}};
        cb.setVertexInput(0, 2, inputs);
        cb.draw(item.vertex_count, item.instance_count);
        break;
      }
      case Program::kCylinder: {
        if (item.mesh == nullptr || item.mesh->vbo == nullptr || cylinder_buf_ == nullptr) {
          continue;
        }
        const QRhiCommandBuffer::VertexInput inputs[] = {{item.mesh->vbo, 0}, {cylinder_buf_, item.buffer_offset}};
        QRhiBuffer* ibo = item.lines ? item.mesh->line_ibo : item.mesh->tri_ibo;
        const std::uint32_t count = item.lines ? item.mesh->line_index_count : item.mesh->tri_index_count;
        cb.setVertexInput(0, 2, inputs, ibo, 0, QRhiCommandBuffer::IndexUInt32);
        cb.drawIndexed(count, item.instance_count);
        break;
      }
      case Program::kSolid:
      case Program::kArrow: {
        if (item.mesh == nullptr || item.mesh->vbo == nullptr || instance_buf_ == nullptr) {
          continue;
        }
        const QRhiCommandBuffer::VertexInput inputs[] = {{item.mesh->vbo, 0}, {instance_buf_, item.buffer_offset}};
        QRhiBuffer* ibo = item.lines ? item.mesh->line_ibo : item.mesh->tri_ibo;
        const std::uint32_t count = item.lines ? item.mesh->line_index_count : item.mesh->tri_index_count;
        cb.setVertexInput(0, 2, inputs, ibo, 0, QRhiCommandBuffer::IndexUInt32);
        cb.drawIndexed(count, item.instance_count);
        break;
      }
    }
  }
}

void RhiMarkerPass::release() {
  for (auto& [key, pipeline] : pipelines_) {
    delete pipeline;
  }
  pipelines_.clear();
  delete srb_;
  srb_ = nullptr;
  delete ubo_;
  ubo_ = nullptr;
  for (UnitMesh* mesh : {&cube_, &sphere_, &cylinder_, &arrow_}) {
    delete mesh->vbo;
    mesh->vbo = nullptr;
    delete mesh->tri_ibo;
    mesh->tri_ibo = nullptr;
    delete mesh->line_ibo;
    mesh->line_ibo = nullptr;
    mesh->tri_index_count = 0;
    mesh->line_index_count = 0;
    mesh->uploaded = false;
  }
  delete edge_vbo_;
  edge_vbo_ = nullptr;
  edge_uploaded_ = false;
  delete instance_buf_;
  instance_buf_ = nullptr;
  delete cylinder_buf_;
  cylinder_buf_ = nullptr;
  delete flat_buf_;
  flat_buf_ = nullptr;
  instance_capacity_ = 0;
  cylinder_capacity_ = 0;
  flat_capacity_ = 0;
  draws_.clear();
  rhi_ = nullptr;
  rpd_ = nullptr;
}

}  // namespace pj::scene3d::rhi
