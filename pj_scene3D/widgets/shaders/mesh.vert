// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Per-vertex: pj::scene3d::Vertex, uploaded verbatim (64-byte stride).
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;
layout(location = 2) in vec4 a_color;
layout(location = 3) in vec2 a_uv;
// .xyz is the model-space U axis, .w the handedness sign used to rebuild the
// bitangent. Already flipped/computed by the loader; do not re-flip here.
layout(location = 4) in vec4 a_tangent;

layout(location = 0) out vec3 v_world_pos;
layout(location = 1) out vec3 v_world_normal;
layout(location = 2) out vec4 v_vertex_color;
layout(location = 3) out vec2 v_uv;
// mat3 as three vec3s: an interpolated mat3 output survives SPIRV-Cross fine, but
// spelling out the columns keeps the location budget legible next to the rest.
layout(location = 4) out vec3 v_tangent;
layout(location = 5) out vec3 v_bitangent;

layout(std140, binding = 0) uniform SceneUbo {
  mat4 view_proj;
  vec4 camera_pos;      // .xyz world-space eye; .w unused
  vec4 key_light_dir;   // .xyz unit direction TO the key light; .w unused
  vec4 light_scales;    // ambient, direct(key), fill, env_intensity
  vec4 render_flags;    // reserved (SSAO/EDL strengths)
};

layout(std140, binding = 1) uniform DrawUbo {
  mat4 model;
  // Inverse-transpose of model's upper 3x3, widened to a mat4 because a std140
  // mat3 pads each column to 16 bytes anyway and the wider type is less
  // error-prone to pack from the host.
  mat4 normal_mat;
  vec4 base_color_factor;
  vec4 object_tint;
  vec4 emissive_factor;   // .rgb; .w unused
  vec4 material;          // metallic, roughness, dielectric_f0, opacity
  vec4 alpha;             // alpha_mode(0/1/2), alpha_cutoff, has_normal_tex, is_collision
  vec4 use_vertex_color;  // .x != 0 selects material-driven base colour
};

void main() {
  const vec4 world = model * vec4(a_pos, 1.0);
  v_world_pos = world.xyz;

  const mat3 nmat = mat3(normal_mat);
  const vec3 N = normalize(nmat * a_normal);
  v_world_normal = N;

  // Tangent basis for normal mapping: re-orthonormalize T against N (Gram-Schmidt)
  // and rebuild B from the stored handedness.
  vec3 T = nmat * a_tangent.xyz;
  T = normalize(T - (N * dot(N, T)));
  v_tangent = T;
  v_bitangent = cross(N, T) * a_tangent.w;

  v_vertex_color = a_color;
  v_uv = a_uv;
  gl_Position = view_proj * world;
}
