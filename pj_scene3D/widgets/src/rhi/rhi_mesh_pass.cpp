// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/rhi/rhi_mesh_pass.h"

#include <QFile>
#include <QImage>
#include <QLoggingCategory>
#include <algorithm>
#include <cstring>
#include <glm/gtc/matrix_inverse.hpp>
#include <utility>

#include "pj_scene3d_widgets/mesh_primitives.h"

namespace pj::scene3d::rhi {
namespace {

Q_LOGGING_CATEGORY(lcRhiMesh, "pj.scene3d.rhi.mesh")

QShader loadBakedShader(const QString& path) {
  QFile file(path);
  if (!file.open(QIODevice::ReadOnly)) {
    qCWarning(lcRhiMesh) << "missing baked shader" << path;
    return {};
  }
  return QShader::fromSerialized(file.readAll());
}

/// Round `value` up to the next multiple of `alignment` (a power of two).
std::uint32_t alignUp(std::uint32_t value, std::uint32_t alignment) {
  if (alignment <= 1) {
    return value;
  }
  return (value + alignment - 1) / alignment * alignment;
}

/// Decode a material map to a texture-ready image, from an external file or the
/// inline bytes of an embedded (glTF/GLB) image. Null on failure.
QImage decodeTextureSource(const TextureSource& source) {
  QImage image;
  if (!source.bytes.empty()) {
    image = QImage::fromData(source.bytes.data(), static_cast<int>(source.bytes.size()));
  } else if (!source.path.isEmpty()) {
    image = QImage(source.path);
  }
  if (image.isNull()) {
    return {};
  }
  return image.convertToFormat(QImage::Format_RGBA8888);
}

/// The material a submesh shades with. The loader always populates one, but a
/// default-constructed SubMesh must stay valid, so fall back rather than deref null.
const Material& materialOf(const SubMesh& submesh) {
  static const Material kFallback;
  return submesh.material != nullptr ? *submesh.material : kFallback;
}

}  // namespace

RhiMeshPass::RhiMeshPass() {
  const auto adopt = [](MeshResource& resource, MeshData data) {
    resource.data = std::move(data);
    resource.has_blended_material = MeshRenderPass::meshHasBlendedMaterial(resource.data);
    resource.dirty = true;
  };
  adopt(cube_, makeCube({0.7F, 0.7F, 0.7F, 1.0F}));
  adopt(cylinder_, makeCylinder());
  adopt(sphere_, makeSphere());
}

RhiMeshPass::~RhiMeshPass() {
  release();
}

void RhiMeshPass::setMeshData(const std::string& key, MeshData data) {
  for (auto& entry : meshes_) {
    if (entry.first == key) {
      destroyMeshGpu(entry.second);
      entry.second.data = std::move(data);
      entry.second.has_blended_material = MeshRenderPass::meshHasBlendedMaterial(entry.second.data);
      entry.second.dirty = true;
      return;
    }
  }
  MeshResource resource;
  resource.data = std::move(data);
  resource.has_blended_material = MeshRenderPass::meshHasBlendedMaterial(resource.data);
  resource.dirty = true;
  meshes_.emplace_back(key, std::move(resource));
}

void RhiMeshPass::clearMeshes() {
  for (auto& entry : meshes_) {
    destroyMeshGpu(entry.second);
  }
  meshes_.clear();
  for (MaterialBindings& entry : material_srbs_) {
    delete entry.srb;
  }
  material_srbs_.clear();
  for (auto& cached : textures_) {
    delete cached.texture;
  }
  textures_.clear();
}

void RhiMeshPass::setVisualDraws(std::vector<DrawCall> draws) {
  visual_draws_ = std::move(draws);
}

void RhiMeshPass::setCollisionDraws(std::vector<DrawCall> draws) {
  collision_draws_ = std::move(draws);
}

void RhiMeshPass::setShadingParams(const MeshShadingParams& params) {
  shading_ = params;
}

void RhiMeshPass::destroyMeshGpu(MeshResource& resource) {
  delete resource.vbo;
  resource.vbo = nullptr;
  delete resource.ibo;
  resource.ibo = nullptr;
  resource.index_count = 0;
  resource.dirty = true;
}

RhiMeshPass::MeshResource& RhiMeshPass::resourceFor(GeometryKind kind) {
  switch (kind) {
    case GeometryKind::kCylinder:
      return cylinder_;
    case GeometryKind::kSphere:
      return sphere_;
    case GeometryKind::kBox:
    case GeometryKind::kPlaceholderCube:
    case GeometryKind::kMesh:
      break;
  }
  return cube_;
}

RhiMeshPass::MeshResource* RhiMeshPass::resourceForDraw(const DrawCall& draw) {
  if (draw.kind != GeometryKind::kMesh) {
    return &resourceFor(draw.kind);
  }
  for (auto& entry : meshes_) {
    if (entry.first == draw.mesh_key) {
      // A failed import has no geometry; fall through to the placeholder cube so
      // the object is visibly present rather than silently missing.
      return entry.second.data.ok ? &entry.second : &cube_;
    }
  }
  return &cube_;
}

bool RhiMeshPass::createPlaceholders() {
  const auto make = [&]() -> QRhiTexture* {
    QRhiTexture* tex = rhi_->newTexture(QRhiTexture::RGBA8, QSize(1, 1));
    if (tex == nullptr || !tex->create()) {
      delete tex;
      return nullptr;
    }
    return tex;
  };
  white_tex_ = make();
  flat_normal_tex_ = make();
  placeholders_uploaded_ = false;
  return white_tex_ != nullptr && flat_normal_tex_ != nullptr;
}

void RhiMeshPass::uploadPlaceholdersIfNeeded(QRhiResourceUpdateBatch& updates) {
  if (placeholders_uploaded_ || white_tex_ == nullptr || flat_normal_tex_ == nullptr) {
    return;
  }
  // White is the identity for the four multiplicative slot_count; (0.5,0.5,1) decodes to
  // the +Z tangent-space normal, i.e. "no perturbation". The CONTENT has to be
  // uploaded through a batch the widget will actually submit — a batch taken inside
  // initialize() is never submitted, so creation and upload are deliberately split.
  const auto upload = [&](QRhiTexture* tex, QRgb rgba) {
    QImage px(1, 1, QImage::Format_RGBA8888);
    px.setPixel(0, 0, rgba);
    updates.uploadTexture(tex, px);
  };
  upload(white_tex_, qRgba(255, 255, 255, 255));
  upload(flat_normal_tex_, qRgba(128, 128, 255, 255));
  placeholders_uploaded_ = true;
}

QRhiTexture* RhiMeshPass::textureFor(
    QRhiResourceUpdateBatch& updates, const TextureSource& source, TextureColorSpace color_space) {
  if (source.empty()) {
    return nullptr;
  }
  const auto it = std::find_if(textures_.begin(), textures_.end(), [&](const CachedTexture& cached) {
    return cached.color_space == color_space && cached.key == source.key;
  });
  if (it != textures_.end()) {
    return it->texture;
  }

  const QImage image = decodeTextureSource(source);
  if (image.isNull()) {
    // Cache the failure as a null texture so a broken path is decoded once rather
    // than once per frame; the caller then falls back for that slot.
    textures_.push_back(CachedTexture{source.key, color_space, nullptr});
    return nullptr;
  }

  const QRhiTexture::Flags flags = QRhiTexture::MipMapped | QRhiTexture::UsedWithGenerateMips |
                                   (color_space == TextureColorSpace::kSrgb ? QRhiTexture::sRGB : QRhiTexture::Flags{});
  QRhiTexture* tex = rhi_->newTexture(QRhiTexture::RGBA8, image.size(), 1, flags);
  if (tex == nullptr || !tex->create()) {
    delete tex;
    textures_.push_back(CachedTexture{source.key, color_space, nullptr});
    return nullptr;
  }
  updates.uploadTexture(tex, image);
  updates.generateMips(tex);
  textures_.push_back(CachedTexture{source.key, color_space, tex});
  return tex;
}

const RhiMeshPass::MaterialBindings* RhiMeshPass::bindingsFor(
    QRhiResourceUpdateBatch& updates, const Material& material) {
  const auto it = std::find_if(material_srbs_.begin(), material_srbs_.end(), [&](const MaterialBindings& entry) {
    return entry.material == &material;
  });
  if (it != material_srbs_.end()) {
    return &(*it);
  }

  const auto resolve = [&](const TextureSource& source, MaterialTextureSlot slot, QRhiTexture* fallback) {
    QRhiTexture* tex = textureFor(updates, source, textureColorSpaceForSlot(slot));
    return tex != nullptr ? tex : fallback;
  };

  QRhiTexture* base = resolve(material.base_color, MaterialTextureSlot::kBaseColor, white_tex_);
  QRhiTexture* mr = resolve(material.metallic_roughness, MaterialTextureSlot::kMetallicRoughness, white_tex_);
  QRhiTexture* nrm = resolve(material.normal, MaterialTextureSlot::kNormal, flat_normal_tex_);
  QRhiTexture* ao = resolve(material.occlusion, MaterialTextureSlot::kOcclusion, white_tex_);
  QRhiTexture* emissive = resolve(material.emissive, MaterialTextureSlot::kEmissive, white_tex_);
  // Only claim a normal map when a real one is bound, not merely requested.
  const bool has_normal_tex = nrm != flat_normal_tex_;

  using SRB = QRhiShaderResourceBinding;
  QRhiShaderResourceBindings* srb = rhi_->newShaderResourceBindings();
  srb->setBindings({
      SRB::uniformBuffer(0, SRB::VertexStage | SRB::FragmentStage, scene_ubo_),
      SRB::uniformBufferWithDynamicOffset(1, SRB::VertexStage | SRB::FragmentStage, draw_ubo_, sizeof(DrawUbo)),
      SRB::sampledTexture(2, SRB::FragmentStage, base, sampler_),
      SRB::sampledTexture(3, SRB::FragmentStage, mr, sampler_),
      SRB::sampledTexture(4, SRB::FragmentStage, nrm, sampler_),
      SRB::sampledTexture(5, SRB::FragmentStage, ao, sampler_),
      SRB::sampledTexture(6, SRB::FragmentStage, emissive, sampler_),
  });
  if (!srb->create()) {
    qCWarning(lcRhiMesh) << "material bindings creation failed";
    delete srb;
    return nullptr;
  }
  material_srbs_.push_back(MaterialBindings{&material, srb, has_normal_tex});
  return &material_srbs_.back();
}

void RhiMeshPass::uploadIfNeeded(QRhiResourceUpdateBatch& updates, MeshResource& resource) {
  if (!resource.dirty) {
    return;
  }
  // destroyMeshGpu re-sets the dirty flag, so clear it AFTER, not before: every
  // exit below must leave the resource considered uploaded or this retries forever.
  destroyMeshGpu(resource);
  resource.dirty = false;
  const MeshData& data = resource.data;
  if (!data.ok || data.vertices.empty() || data.indices.empty()) {
    return;
  }

  const auto vbytes = static_cast<quint32>(data.vertices.size() * sizeof(Vertex));
  const auto ibytes = static_cast<quint32>(data.indices.size() * sizeof(std::uint32_t));
  resource.vbo = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::VertexBuffer, vbytes);
  resource.ibo = rhi_->newBuffer(QRhiBuffer::Immutable, QRhiBuffer::IndexBuffer, ibytes);
  if (resource.vbo == nullptr || resource.ibo == nullptr || !resource.vbo->create() || !resource.ibo->create()) {
    destroyMeshGpu(resource);
    resource.dirty = false;  // a failed upload must not retry every frame
    return;
  }
  updates.uploadStaticBuffer(resource.vbo, data.vertices.data());
  updates.uploadStaticBuffer(resource.ibo, data.indices.data());
  resource.index_count = static_cast<std::uint32_t>(data.indices.size());
}

void RhiMeshPass::resolveBucket(
    QRhiResourceUpdateBatch& updates, const std::vector<DrawCall>& draws, float opacity, bool collision) {
  if (opacity <= 0.0F) {
    return;
  }
  for (const DrawCall& draw : draws) {
    MeshResource* resource = resourceForDraw(draw);
    if (resource == nullptr) {
      continue;
    }
    uploadIfNeeded(updates, *resource);
    if (resource->index_count == 0) {
      continue;
    }
    // The collision hull is an overlay and always blends; visual draws are bucketed
    // by the same rule the GL pass uses (layer opacity, tint alpha, glTF kBlend).
    const bool translucent =
        collision || MeshRenderPass::drawNeedsVisualBlending(draw, resource->has_blended_material, opacity);

    for (const SubMesh& submesh : resource->data.submeshes) {
      if (submesh.index_count == 0) {
        continue;
      }
      const Material& material = materialOf(submesh);
      const MaterialBindings* bindings = bindingsFor(updates, material);
      if (bindings == nullptr) {
        continue;
      }

      DrawUbo block{};
      std::memcpy(block.model, &draw.model[0][0], sizeof(block.model));
      // Non-uniform scale is legal on a URDF visual origin, so the normal matrix is
      // the genuine inverse-transpose, not just the upper-left 3x3.
      const glm::mat4 normal_mat(glm::inverseTranspose(glm::mat3(draw.model)));
      std::memcpy(block.normal_mat, &normal_mat[0][0], sizeof(block.normal_mat));
      std::memcpy(block.base_color_factor, &material.base_color_factor[0], sizeof(block.base_color_factor));
      std::memcpy(block.object_tint, &draw.color[0], sizeof(block.object_tint));
      block.emissive_factor[0] = material.emissive_factor.x;
      block.emissive_factor[1] = material.emissive_factor.y;
      block.emissive_factor[2] = material.emissive_factor.z;
      block.material[0] = material.metallic_factor;
      // Sources with no PBR keys (STL, procedural primitives) inherit the per-view
      // look instead of their absent glTF values, preserving the GL behaviour.
      block.material[1] = material.has_pbr ? material.roughness_factor : shading_.roughness;
      block.material[2] = shading_.reflectivity;
      block.material[3] = opacity;
      block.alpha[0] = static_cast<float>(static_cast<int>(material.alpha_mode));
      block.alpha[1] = material.alpha_cutoff;
      block.alpha[2] = bindings->has_normal_tex ? 1.0F : 0.0F;
      block.alpha[3] = collision ? 1.0F : 0.0F;
      block.use_vertex_color[0] = draw.use_vertex_color ? 1.0F : 0.0F;

      const auto slot = static_cast<std::uint32_t>(resolved_.size());
      const std::uint32_t byte_offset = slot * draw_ubo_stride_;
      // slotUpperBound() sized the buffer for every submesh that could resolve, so
      // overrunning here means those two walks disagree — assert rather than grow.
      Q_ASSERT(byte_offset + sizeof(DrawUbo) <= draw_ubo_staging_.size());
      std::memcpy(draw_ubo_staging_.data() + byte_offset, &block, sizeof(DrawUbo));

      resolved_.push_back(ResolvedDraw{resource, &submesh, bindings->srb, byte_offset, translucent});
    }
  }
}

bool RhiMeshPass::initialize(QRhi& rhi, QRhiRenderPassDescriptor& rpd, int sample_count) {
  if (pipeline_opaque_ != nullptr && rhi_ == &rhi && sample_count_ == sample_count) {
    return true;
  }
  release();
  rhi_ = &rhi;
  sample_count_ = sample_count;
  draw_ubo_stride_ = alignUp(sizeof(DrawUbo), static_cast<std::uint32_t>(rhi.ubufAlignment()));

  const QShader vert = loadBakedShader(QStringLiteral(":/scene3d_shaders/mesh.vert.qsb"));
  const QShader frag = loadBakedShader(QStringLiteral(":/scene3d_shaders/mesh.frag.qsb"));
  if (!vert.isValid() || !frag.isValid()) {
    release();
    return false;
  }

  scene_ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, sizeof(SceneUbo));
  if (!scene_ubo_->create()) {
    release();
    return false;
  }
  sampler_ = rhi.newSampler(
      QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Linear, QRhiSampler::Repeat, QRhiSampler::Repeat);
  if (sampler_ == nullptr || !sampler_->create()) {
    release();
    return false;
  }

  QRhiVertexInputLayout layout;
  layout.setBindings({QRhiVertexInputBinding(sizeof(Vertex))});
  layout.setAttributes({
      QRhiVertexInputAttribute(0, 0, QRhiVertexInputAttribute::Float3, offsetof(Vertex, position)),
      QRhiVertexInputAttribute(0, 1, QRhiVertexInputAttribute::Float3, offsetof(Vertex, normal)),
      QRhiVertexInputAttribute(0, 2, QRhiVertexInputAttribute::Float4, offsetof(Vertex, color)),
      QRhiVertexInputAttribute(0, 3, QRhiVertexInputAttribute::Float2, offsetof(Vertex, uv)),
      QRhiVertexInputAttribute(0, 4, QRhiVertexInputAttribute::Float4, offsetof(Vertex, tangent)),
  });

  // A throwaway binding set purely to fix the LAYOUT the pipelines compile against.
  // The per-material sets created later are layout-compatible with it by
  // construction, which is what lets both pipelines serve every material.
  using SRB = QRhiShaderResourceBinding;
  draw_ubo_ = rhi.newBuffer(QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, draw_ubo_stride_);
  if (!draw_ubo_->create()) {
    release();
    return false;
  }
  draw_ubo_capacity_ = 1;

  if (!createPlaceholders()) {
    release();
    return false;
  }

  QRhiShaderResourceBindings* layout_srb = rhi.newShaderResourceBindings();
  layout_srb->setBindings({
      SRB::uniformBuffer(0, SRB::VertexStage | SRB::FragmentStage, scene_ubo_),
      SRB::uniformBufferWithDynamicOffset(1, SRB::VertexStage | SRB::FragmentStage, draw_ubo_, sizeof(DrawUbo)),
      SRB::sampledTexture(2, SRB::FragmentStage, white_tex_, sampler_),
      SRB::sampledTexture(3, SRB::FragmentStage, white_tex_, sampler_),
      SRB::sampledTexture(4, SRB::FragmentStage, flat_normal_tex_, sampler_),
      SRB::sampledTexture(5, SRB::FragmentStage, white_tex_, sampler_),
      SRB::sampledTexture(6, SRB::FragmentStage, white_tex_, sampler_),
  });
  if (!layout_srb->create()) {
    delete layout_srb;
    release();
    return false;
  }

  const auto makePipeline = [&](bool blended) -> QRhiGraphicsPipeline* {
    QRhiGraphicsPipeline* pipeline = rhi.newGraphicsPipeline();
    pipeline->setShaderStages({{QRhiShaderStage::Vertex, vert}, {QRhiShaderStage::Fragment, frag}});
    pipeline->setVertexInputLayout(layout);
    pipeline->setShaderResourceBindings(layout_srb);
    pipeline->setRenderPassDescriptor(&rpd);
    pipeline->setSampleCount(sample_count_);
    pipeline->setDepthTest(true);
    // Robot meshes routinely have sloppy winding, so nothing is culled — matching
    // the GL pass, which is also why Material::double_sided stays latent.
    pipeline->setCullMode(QRhiGraphicsPipeline::None);
    if (blended) {
      // Depth-read-only, not depth-sorted: a best-effort translucency that keeps
      // hulls and blended materials from erasing what is behind them.
      pipeline->setDepthWrite(false);
      QRhiGraphicsPipeline::TargetBlend blend;
      blend.enable = true;
      blend.srcColor = QRhiGraphicsPipeline::SrcAlpha;
      blend.dstColor = QRhiGraphicsPipeline::OneMinusSrcAlpha;
      // Coverage-union on alpha (ONE, not SrcAlpha) so a translucent hull adds its
      // own coverage rather than scaling away what the opaque pass established.
      blend.srcAlpha = QRhiGraphicsPipeline::One;
      blend.dstAlpha = QRhiGraphicsPipeline::OneMinusSrcAlpha;
      pipeline->setTargetBlends({blend});
    } else {
      pipeline->setDepthWrite(true);
    }
    if (!pipeline->create()) {
      delete pipeline;
      return nullptr;
    }
    return pipeline;
  };

  pipeline_opaque_ = makePipeline(false);
  pipeline_blended_ = makePipeline(true);
  // The pipelines hold only the layout, not this object; the per-material sets do
  // the actual binding at draw time.
  delete layout_srb;

  if (pipeline_opaque_ == nullptr || pipeline_blended_ == nullptr) {
    qCWarning(lcRhiMesh) << "mesh pipeline creation failed";
    release();
    return false;
  }

  // A new QRhi invalidated every cached GPU object built against the old one.
  for (auto& entry : meshes_) {
    entry.second.dirty = true;
  }
  cube_.dirty = true;
  cylinder_.dirty = true;
  sphere_.dirty = true;
  return true;
}

int RhiMeshPass::slotUpperBound() {
  // An upper bound on this frame's per-draw uniform slot_count: one per submesh of each
  // draw's resource. Counted from the CPU mesh data, which is populated before
  // upload, so the estimate is already right on the very first frame.
  int slot_count = 0;
  const auto count = [&](const std::vector<DrawCall>& draws) {
    for (const DrawCall& draw : draws) {
      const MeshResource* resource = resourceForDraw(draw);
      if (resource != nullptr) {
        slot_count += static_cast<int>(resource->data.submeshes.size());
      }
    }
  };
  if (shading_.meshes_visible && shading_.mesh_opacity > 0.0F) {
    count(visual_draws_);
  }
  if (shading_.collisions_visible && shading_.collision_opacity > 0.0F) {
    count(collision_draws_);
  }
  return slot_count;
}

bool RhiMeshPass::ensureDrawUboCapacity(int slot_count) {
  if (slot_count <= draw_ubo_capacity_) {
    return true;
  }
  // Re-creating the buffer invalidates every material binding set that referenced
  // the old one, so they are dropped here and rebuilt during this same resolve —
  // which is why capacity is settled BEFORE resolving rather than after.
  delete draw_ubo_;
  draw_ubo_capacity_ = std::max(slot_count * 2, 16);
  draw_ubo_ = rhi_->newBuffer(
      QRhiBuffer::Dynamic, QRhiBuffer::UniformBuffer, static_cast<quint32>(draw_ubo_capacity_) * draw_ubo_stride_);
  if (draw_ubo_ == nullptr || !draw_ubo_->create()) {
    draw_ubo_capacity_ = 0;
    return false;
  }
  for (MaterialBindings& entry : material_srbs_) {
    delete entry.srb;
  }
  material_srbs_.clear();
  draw_ubo_staging_.assign(static_cast<std::size_t>(draw_ubo_capacity_) * draw_ubo_stride_, std::byte{});
  return true;
}

void RhiMeshPass::prepare(QRhiResourceUpdateBatch& updates, const RhiFrameContext& ctx) {
  resolved_.clear();
  if (pipeline_opaque_ == nullptr || rhi_ == nullptr) {
    return;
  }
  uploadPlaceholdersIfNeeded(updates);
  if (!ensureDrawUboCapacity(slotUpperBound())) {
    return;
  }

  if (shading_.meshes_visible) {
    resolveBucket(updates, visual_draws_, shading_.mesh_opacity, false);
  }
  if (shading_.collisions_visible) {
    resolveBucket(updates, collision_draws_, shading_.collision_opacity, true);
  }
  // Opaque first so the translucent bucket tests against a complete depth buffer.
  std::stable_partition(resolved_.begin(), resolved_.end(), [](const ResolvedDraw& draw) { return !draw.translucent; });

  for (const ResolvedDraw& draw : resolved_) {
    updates.updateDynamicBuffer(
        draw_ubo_, draw.ubo_offset, sizeof(DrawUbo), draw_ubo_staging_.data() + draw.ubo_offset);
  }

  SceneUbo scene{};
  std::memcpy(scene.view_proj, &ctx.view_proj[0][0], sizeof(scene.view_proj));
  scene.camera_pos[0] = ctx.camera_pos_world.x;
  scene.camera_pos[1] = ctx.camera_pos_world.y;
  scene.camera_pos[2] = ctx.camera_pos_world.z;
  const glm::vec3 key = glm::normalize(shading_.key_light_dir);
  scene.key_light_dir[0] = key.x;
  scene.key_light_dir[1] = key.y;
  scene.key_light_dir[2] = key.z;
  scene.light_scales[0] = shading_.ambient_scale;
  scene.light_scales[1] = shading_.direct_scale;
  scene.light_scales[2] = shading_.fill_light_scale;
  scene.light_scales[3] = shading_.env_intensity;
  updates.updateDynamicBuffer(scene_ubo_, 0, sizeof(SceneUbo), &scene);
}

void RhiMeshPass::draw(QRhiCommandBuffer& cb, const RhiFrameContext& /*ctx*/) {
  if (pipeline_opaque_ == nullptr || resolved_.empty()) {
    return;
  }
  QRhiGraphicsPipeline* bound = nullptr;
  const MeshResource* bound_resource = nullptr;

  for (const ResolvedDraw& draw : resolved_) {
    QRhiGraphicsPipeline* wanted = draw.translucent ? pipeline_blended_ : pipeline_opaque_;
    if (wanted != bound) {
      cb.setGraphicsPipeline(wanted);
      bound = wanted;
      // A pipeline switch invalidates the recorded vertex input on some backends,
      // so re-bind the buffers for the first draw after it.
      bound_resource = nullptr;
    }
    const QRhiCommandBuffer::DynamicOffset dyn_offset{1, draw.ubo_offset};
    cb.setShaderResources(draw.srb, 1, &dyn_offset);
    if (draw.resource != bound_resource) {
      const QRhiCommandBuffer::VertexInput input{draw.resource->vbo, 0};
      cb.setVertexInput(0, 1, &input, draw.resource->ibo, 0, QRhiCommandBuffer::IndexUInt32);
      bound_resource = draw.resource;
    }
    cb.drawIndexed(
        static_cast<quint32>(draw.submesh->index_count), 1, static_cast<quint32>(draw.submesh->index_offset));
  }
}

void RhiMeshPass::release() {
  delete pipeline_opaque_;
  pipeline_opaque_ = nullptr;
  delete pipeline_blended_;
  pipeline_blended_ = nullptr;
  for (MaterialBindings& entry : material_srbs_) {
    delete entry.srb;
  }
  material_srbs_.clear();
  for (auto& cached : textures_) {
    delete cached.texture;
  }
  textures_.clear();
  delete white_tex_;
  white_tex_ = nullptr;
  delete flat_normal_tex_;
  flat_normal_tex_ = nullptr;
  delete sampler_;
  sampler_ = nullptr;
  delete scene_ubo_;
  scene_ubo_ = nullptr;
  delete draw_ubo_;
  draw_ubo_ = nullptr;
  draw_ubo_capacity_ = 0;
  for (auto& entry : meshes_) {
    destroyMeshGpu(entry.second);
  }
  destroyMeshGpu(cube_);
  destroyMeshGpu(cylinder_);
  destroyMeshGpu(sphere_);
  resolved_.clear();
  rhi_ = nullptr;
}

}  // namespace pj::scene3d::rhi
