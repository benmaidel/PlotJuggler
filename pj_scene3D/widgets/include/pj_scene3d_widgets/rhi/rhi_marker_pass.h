#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <optional>
#include <vector>

#include "pj_scene3d_core/scene_entities_render.h"
#include "pj_scene3d_widgets/rhi/rhi_arrow_shading.h"
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// SceneEntities / marker geometry: the QRhi counterpart of MarkerRenderPass.
///
/// Marker geometry is *annotation*, not lit material — every primitive shows its
/// exact colour on every face, so all of it shares one flat fragment shader and
/// none of it is shaded. Ported primitives: cubes and spheres (one instanced
/// unit mesh each), cylinders/cones (instanced, with a vertex-stage taper),
/// arrows and axes (reusing the shared arrow mesh + shaders), and line and
/// triangle batches. Text is not ported — the GL pass does not draw it either.
///
/// TF stays outside the pass. `setActive()` takes the decoded batch, whose
/// primitives are frame-local, and `setFrameTransforms()` takes the resolved
/// fixed_frame<-frame matrix per interned frame index. That keeps this pass
/// unaware of the TF buffer and the datastore, matching the rest of the QRhi
/// renderer; primitives whose frame did not resolve are skipped, so a partially
/// resolvable batch still draws its resolvable subset.
///
/// Three notable differences from the GL pass:
///
/// - **Wireframe is real line geometry.** `glPolygonMode` has no QRhi (or Metal)
///   equivalent, so each solid mesh carries a second, LINES index buffer built
///   from its triangles, and triangle batches expand to segments on the CPU.
/// - **Line and triangle batches merge into one draw each.** Their world
///   placement is baked into the vertices during staging — they are re-streamed
///   every frame regardless — instead of being a per-batch matrix uniform, which
///   is what lets every batch share one buffer. GL issues one draw per batch.
/// - **Pipelines come from a small cache** keyed by the state that actually
///   varies (topology, culling, depth write, blending, depth bias). QRhi bakes
///   all of that into an immutable pipeline object, so the GL pass's ~15
///   glEnable/glDisable transitions would otherwise become that many named
///   members.
class RhiMarkerPass final : public IRhiRenderPass {
 public:
  /// Viewer-side display overrides. These are a per-topic *preference*, not marker
  /// data: the marker protocol has no notion of opacity, recolouring or wireframe.
  struct DisplayOverrides {
    /// Multiplies every primitive's alpha.
    float opacity = 1.0F;
    bool color_override = false;
    glm::vec4 override_color{1.0F};
    /// Draw solids and triangle batches as edges rather than filled faces.
    bool wireframe = false;
  };

  RhiMarkerPass();
  ~RhiMarkerPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Swap the batch to draw. Cheap — it only stores the pointer; the GPU staging
  /// happens in prepare(), because instance data depends on the live TF.
  void setActive(std::shared_ptr<const DecodedSceneEntities> markers);

  /// Resolved fixed_frame <- frame transforms, indexed by the batch's
  /// `frame_index`. An unset entry means that frame did not resolve this frame,
  /// and its primitives are skipped.
  void setFrameTransforms(std::vector<std::optional<glm::mat4>> fixed_from_frame);

  void setOverrides(const DisplayOverrides& overrides);
  [[nodiscard]] const DisplayOverrides& overrides() const {
    return overrides_;
  }

  /// Per-topic visibility. When false nothing draws, but the active batch is kept.
  void setVisible(bool visible);

 private:
  /// Which shader pair and vertex layout a pipeline uses. Each has a different
  /// vertex layout, which is why the pipeline cache is keyed by it.
  enum class Program : std::uint8_t { kSolid, kCylinder, kEdge, kFlat, kArrow };

  /// The pipeline state that actually varies across the GL pass's draws.
  struct PipelineKey {
    Program program = Program::kSolid;
    bool lines = false;
    bool cull_back = false;
    bool depth_write = true;
    bool blend = true;
    /// Push the fill away from the camera so a coincident line overlay wins the
    /// depth test. Only the cube fill uses it.
    bool depth_bias = false;

    [[nodiscard]] bool operator==(const PipelineKey&) const = default;
  };

  /// std140 layout of the shaders' `MarkerUbo`, declared identically by all four
  /// marker shaders so every marker pipeline shares one binding layout.
  ///
  /// The first two matrices are deliberately the SAME PREFIX as arrow.vert's
  /// `ArrowUbo`, which is what lets the arrow and axes markers reuse the shared
  /// arrow shaders off this one buffer. A shader may declare a smaller block than
  /// the buffer holds, but not a larger one: the arrow shaders read bytes 0-127, so
  /// this struct must cover at least that or every arrow silently collapses to a
  /// degenerate point on a zeroed `frame_world`.
  struct MarkerUbo {
    float view_proj[16];
    /// Always identity. Marker primitives bake their frame transform into their
    /// instance matrix / vertices; this exists only for the shared arrow shaders.
    float frame_world[16];
    /// edge darken, hidden mix, visible epsilon, unused.
    float edge_params[4];
    float camera_pos[4];
  };
  static_assert(sizeof(MarkerUbo) == 160, "must cover ArrowUbo's 128-byte prefix — see above");

  /// Per-instance record for the solid and edge programs: a world matrix plus a
  /// colour. Byte-identical to the arrow instance record, so the arrow draws can
  /// share the same staging and buffer path.
  struct Instance {
    float world[16];
    float color[4];
  };
  static_assert(sizeof(Instance) == sizeof(arrow::Instance), "solid and arrow instances must share a stride");

  /// Per-instance record for the cylinder program: the solid record plus the
  /// bottom/top radius scales the vertex stage interpolates between.
  struct CylinderInstance {
    float world[16];
    float color[4];
    float taper[2];
    float pad[2];
  };
  static_assert(sizeof(CylinderInstance) == 96);

  /// One streamed vertex of the line/triangle batches: already world-space.
  struct FlatVertex {
    float pos[3];
    float color[4];
  };
  static_assert(sizeof(FlatVertex) == 28);

  /// A static unit mesh plus its two index buffers — triangles, and the LINES set
  /// used for wireframe (QRhi has no polygon-mode state).
  struct UnitMesh {
    QRhiBuffer* vbo = nullptr;
    QRhiBuffer* tri_ibo = nullptr;
    QRhiBuffer* line_ibo = nullptr;
    std::uint32_t tri_index_count = 0;
    std::uint32_t line_index_count = 0;
    std::vector<float> vertices;
    std::vector<std::uint32_t> tri_indices;
    std::vector<std::uint32_t> line_indices;
    bool uploaded = false;
  };

  /// A recorded draw for this frame: which pipeline state, which geometry, and
  /// where its instances sit in the shared instance buffer.
  struct DrawItem {
    PipelineKey key;
    const UnitMesh* mesh = nullptr;
    /// Byte offset into the matching instance/vertex buffer.
    std::uint32_t buffer_offset = 0;
    std::uint32_t instance_count = 0;
    /// Vertex count for the non-indexed programs (edge lines, flat batches).
    std::uint32_t vertex_count = 0;
    bool indexed = true;
    bool lines = false;
  };

  QRhiGraphicsPipeline* pipelineFor(const PipelineKey& key);
  bool buildMeshes();
  void uploadMeshIfNeeded(QRhiResourceUpdateBatch& updates, UnitMesh& mesh);
  /// Grow `buffer` to at least `bytes`, re-creating it if needed. Returns false if
  /// creation failed.
  bool ensureBuffer(QRhiBuffer*& buffer, int& capacity_bytes, int bytes, QRhiBuffer::UsageFlags usage);
  [[nodiscard]] glm::vec4 applyOverride(const glm::vec4& color) const;
  /// World matrix of a primitive, or nullopt when its frame did not resolve.
  [[nodiscard]] std::optional<glm::mat4> worldOf(std::uint32_t frame_index, const glm::mat4& model) const;

  QRhi* rhi_ = nullptr;
  QRhiRenderPassDescriptor* rpd_ = nullptr;
  int sample_count_ = 1;

  QRhiBuffer* ubo_ = nullptr;
  QRhiShaderResourceBindings* srb_ = nullptr;
  /// Cached pipelines, built on first use of each state combination.
  std::vector<std::pair<PipelineKey, QRhiGraphicsPipeline*>> pipelines_;

  UnitMesh cube_;
  UnitMesh sphere_;
  UnitMesh cylinder_;
  UnitMesh arrow_;
  /// The 12 cube edges as 24 line endpoints, each carrying its two adjacent face
  /// normals and its edge centre. Instanced, not indexed.
  QRhiBuffer* edge_vbo_ = nullptr;
  bool edge_uploaded_ = false;

  /// Per-frame staging. One buffer per vertex-layout family, each holding every
  /// draw of that family back to back so a frame costs one upload apiece.
  std::vector<std::byte> instance_staging_;
  std::vector<std::byte> cylinder_staging_;
  std::vector<FlatVertex> flat_staging_;
  QRhiBuffer* instance_buf_ = nullptr;
  QRhiBuffer* cylinder_buf_ = nullptr;
  QRhiBuffer* flat_buf_ = nullptr;
  int instance_capacity_ = 0;
  int cylinder_capacity_ = 0;
  int flat_capacity_ = 0;

  std::vector<DrawItem> draws_;

  std::shared_ptr<const DecodedSceneEntities> markers_;
  std::vector<std::optional<glm::mat4>> frame_world_;
  DisplayOverrides overrides_;
  bool visible_ = true;
};

}  // namespace pj::scene3d::rhi
