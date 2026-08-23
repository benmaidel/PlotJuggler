#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include <cstdint>
#include <glm/glm.hpp>
#include <memory>
#include <string>
#include <string_view>
#include <vector>

#include "pj_scene3d_widgets/mesh_data.h"
#include "pj_scene3d_widgets/mesh_shading_params.h"
#include "pj_scene3d_widgets/passes/mesh_render_pass.h"  // GeometryKind / TextureColorSpace policy
#include "pj_scene3d_widgets/rhi/rhi_render_pass.h"

namespace pj::scene3d::rhi {

/// URDF / scene meshes with glTF 2.0 metallic-roughness shading: the QRhi
/// counterpart of MeshRenderPass.
///
/// The shading core is ported whole — base colour (material-driven or per-draw
/// override), the five texture maps, normal mapping, a fixed world key light plus a
/// camera-locked fill, and the analytic image-based ambient (diffuse irradiance +
/// split-sum specular reflection of a procedural sky/ground environment) that makes
/// metals read as metal. Opaque and translucent buckets are split the same way, and
/// the collision overlay keeps its own tint and depth behaviour.
///
/// Two structural differences from the GL pass, both forced by QRhi:
///
/// - **Every material binds all five samplers.** A pipeline is compiled against its
///   binding layout, so the layout has to be final at pipeline-creation time. Absent
///   maps therefore bind a 1x1 neutral texel rather than being switched off by a
///   uniform, which also removes four of GL's five `u_has_*_tex` flags: white is the
///   identity for all four multiplicative slot_count. Only the normal map keeps a flag,
///   because its neutral value is a no-op solely when the tangent basis is
///   well-formed, and a mesh with no UVs does not have one.
/// - **Per-draw uniforms ride a dynamic-offset UBO.** GL re-set `u_model` per draw;
///   here one buffer holds every draw's block at an `ubufAlignment()`-aligned
///   stride, and each draw binds its own slot by offset.
///
/// Not ported (tracked in docs/ARCHITECTURE.md): shadow receive and the "is-mesh"
/// mask for EDL, both of which need passes that do not exist on QRhi yet.
class RhiMeshPass final : public IRhiRenderPass {
 public:
  using GeometryKind = MeshRenderPass::GeometryKind;
  using DrawCall = MeshRenderPass::DrawCall;

  RhiMeshPass();
  ~RhiMeshPass() override;

  [[nodiscard]] bool initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) override;
  void prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) override;
  void draw(QRhiCommandBuffer& cb, const RhiFrameContext& ctx) override;
  void release() override;

  /// Store or replace a keyed mesh's CPU data. GPU upload is deferred to the next
  /// prepare(), so this is safe to call from a decode thread.
  void setMeshData(const std::string& key, MeshData data);

  /// Forget every keyed mesh and cached texture. The GPU objects are destroyed on
  /// the next prepare()/release() rather than here, so this needs no live QRhi.
  void clearMeshes();

  /// The visual draws for this frame (URDF links, markers, primitives).
  void setVisualDraws(std::vector<DrawCall> draws);
  /// The collision-hull overlay draws for this frame.
  void setCollisionDraws(std::vector<DrawCall> draws);

  /// Per-view look knobs; `mesh_opacity` / `collision_opacity` also decide which
  /// bucket a draw lands in.
  void setShadingParams(const MeshShadingParams& params);

 private:
  /// std140 layout of the shaders' `SceneUbo` (once per frame).
  struct SceneUbo {
    float view_proj[16];
    float camera_pos[4];
    float key_light_dir[4];
    /// ambient, direct (key), fill, env_intensity.
    float light_scales[4];
    /// Reserved. Held for the SSAO/EDL strengths that land with those passes; the
    /// shader must still declare it so the block layout stays fixed.
    float render_flags[4];
  };
  static_assert(sizeof(SceneUbo) == 128);

  /// std140 layout of the shaders' `DrawUbo` (one slot per draw).
  struct DrawUbo {
    float model[16];
    float normal_mat[16];
    float base_color_factor[4];
    float object_tint[4];
    float emissive_factor[4];
    /// metallic, roughness, dielectric_f0, opacity.
    float material[4];
    /// alpha_mode, alpha_cutoff, has_normal_tex, is_collision.
    float alpha[4];
    /// .x != 0 selects material-driven base colour.
    float use_vertex_color[4];
  };
  static_assert(sizeof(DrawUbo) == 224);

  /// A mesh's GPU buffers plus the CPU data they came from.
  struct MeshResource {
    MeshData data;
    QRhiBuffer* vbo = nullptr;
    QRhiBuffer* ibo = nullptr;
    std::uint32_t index_count = 0;
    bool dirty = true;
    /// Whether any submesh material is glTF kBlend. Cached when the data is set, so
    /// bucketing a draw never re-walks the submesh list.
    bool has_blended_material = false;
  };

  /// One cached texture. The same source key can legally appear twice — once sRGB,
  /// once linear — when one image serves both a colour slot and a data slot.
  struct CachedTexture {
    std::string key;
    TextureColorSpace color_space = TextureColorSpace::kLinear;
    QRhiTexture* texture = nullptr;
  };

  /// One material's resource bindings, plus whether a real normal map ended up
  /// bound. The shader's has_normal_tex flag has to follow what was ACTUALLY bound,
  /// not what the material asked for: a map that failed to decode falls back to the
  /// flat placeholder, and a mesh with no tangents has a degenerate basis that would
  /// turn that into NaN normals if the flag stayed set.
  struct MaterialBindings {
    const Material* material = nullptr;
    QRhiShaderResourceBindings* srb = nullptr;
    bool has_normal_tex = false;
  };

  /// One submesh resolved for this frame: which buffers to draw, which resource
  /// bindings carry its material, and where its uniform slot sits.
  struct ResolvedDraw {
    MeshResource* resource = nullptr;
    const SubMesh* submesh = nullptr;
    QRhiShaderResourceBindings* srb = nullptr;
    std::uint32_t ubo_offset = 0;
    bool translucent = false;
  };

  MeshResource& resourceFor(GeometryKind kind);
  MeshResource* resourceForDraw(const DrawCall& draw);
  void uploadIfNeeded(QRhiResourceUpdateBatch& updates, MeshResource& resource);
  /// Resolve a material map to a cached texture, decoding and uploading on first
  /// use. Returns NULLPTR for an empty or undecodable source so the caller can
  /// substitute the right neutral placeholder for that slot — white and flat-normal
  /// are not interchangeable.
  QRhiTexture* textureFor(QRhiResourceUpdateBatch& updates, const TextureSource& source, TextureColorSpace color_space);
  /// Resource bindings for one material. Cached, since every distinct material needs
  /// its own set and they are layout-identical by construction. Null on failure.
  const MaterialBindings* bindingsFor(QRhiResourceUpdateBatch& updates, const Material& material);
  /// Create the 1x1 neutral textures. Creation and upload are split because a
  /// QRhiResourceUpdateBatch taken inside initialize() is never submitted.
  bool createPlaceholders();
  void uploadPlaceholdersIfNeeded(QRhiResourceUpdateBatch& updates);
  /// Upper bound on this frame's per-draw uniform slot_count, counted from CPU mesh data
  /// so it is already correct before the first upload.
  [[nodiscard]] int slotUpperBound();
  /// Size the per-draw UBO for `slot_count` slots (`slots` is a Qt keyword macro). Must run BEFORE resolving: growing
  /// re-creates the buffer and so invalidates every material binding set.
  bool ensureDrawUboCapacity(int slot_count);
  void resolveBucket(
      QRhiResourceUpdateBatch& updates, const std::vector<DrawCall>& draws, float opacity, bool collision);
  void destroyMeshGpu(MeshResource& resource);

  QRhi* rhi_ = nullptr;
  int sample_count_ = 1;
  /// Aligned stride of one DrawUbo slot; QRhi requires dynamic offsets to be a
  /// multiple of ubufAlignment(), which is 256 on some backends.
  std::uint32_t draw_ubo_stride_ = 0;

  QRhiBuffer* scene_ubo_ = nullptr;
  QRhiBuffer* draw_ubo_ = nullptr;
  /// Capacity of draw_ubo_ in slot_count; grown geometrically.
  int draw_ubo_capacity_ = 0;
  QRhiSampler* sampler_ = nullptr;
  /// Opaque (depth write, no blend) and translucent (blended, depth read-only).
  /// The collision overlay reuses the translucent pipeline.
  QRhiGraphicsPipeline* pipeline_opaque_ = nullptr;
  QRhiGraphicsPipeline* pipeline_blended_ = nullptr;

  /// 1x1 neutral texels standing in for absent maps: white for the multiplicative
  /// slot_count, flat (0.5,0.5,1) for the normal map.
  QRhiTexture* white_tex_ = nullptr;
  QRhiTexture* flat_normal_tex_ = nullptr;

  std::vector<std::pair<std::string, MeshResource>> meshes_;
  std::vector<CachedTexture> textures_;
  /// Keyed by the material's address: SubMesh shares one shared_ptr<Material> across
  /// every submesh that uses it, so identity is the natural key and needs no hashing
  /// of factors and paths.
  std::vector<MaterialBindings> material_srbs_;

  MeshResource cube_;
  MeshResource cylinder_;
  MeshResource sphere_;

  std::vector<DrawCall> visual_draws_;
  std::vector<DrawCall> collision_draws_;
  /// This frame's resolved submesh draws, opaque first. Rebuilt each prepare().
  std::vector<ResolvedDraw> resolved_;
  /// Staging for the per-draw uniform slot_count, one aligned block per resolved draw.
  std::vector<std::byte> draw_ubo_staging_;

  MeshShadingParams shading_;
  bool placeholders_uploaded_ = false;
};

}  // namespace pj::scene3d::rhi
