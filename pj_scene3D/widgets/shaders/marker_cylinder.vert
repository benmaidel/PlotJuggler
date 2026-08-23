// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Cylinder / cone / truncated cone. These cannot share the rigid solid path
// because the taper DEFORMS the mesh: each instance independently collapses the
// bottom and top face radii toward the axis, which happens here in the vertex
// stage rather than by uploading a distinct mesh per marker.

layout(location = 0) in vec3 a_pos;
// 0 on the bottom (-Z) ring, 1 on the top (+Z) ring; interpolates the taper.
layout(location = 2) in float a_taper_w;
layout(location = 3) in vec4 a_world_c0;
layout(location = 4) in vec4 a_world_c1;
layout(location = 5) in vec4 a_world_c2;
layout(location = 6) in vec4 a_world_c3;
layout(location = 7) in vec4 a_color;
layout(location = 8) in vec2 a_taper;  // (bottom_scale, top_scale)

layout(location = 0) out vec4 v_color;

layout(std140, binding = 0) uniform MarkerUbo {
  mat4 view_proj;
  // Unused by the marker shaders — each primitive's frame transform is already
  // baked into its instance matrix / vertices. It is declared anyway so this block
  // shares a PREFIX with arrow.vert's ArrowUbo, letting the arrow and axes markers
  // reuse the shared arrow shaders off this very same buffer. Drop it only if
  // arrows stop sharing that pair.
  mat4 frame_world;
  // Cube-edge shading constants: darken, hidden_mix, visible_epsilon. Read only by
  // marker_edge.vert; every marker pipeline declares this identical block so they
  // can all share a single resource-binding layout.
  vec4 edge_params;
  // World-space eye, also only used by marker_edge.vert.
  vec4 camera_pos;
};

void main() {
  const mat4 world = mat4(a_world_c0, a_world_c1, a_world_c2, a_world_c3);
  const float rscale = mix(a_taper.x, a_taper.y, a_taper_w);
  // Only the radial components scale; the axis length stays with the model matrix.
  const vec3 p = vec3(a_pos.x * rscale, a_pos.y * rscale, a_pos.z);
  gl_Position = view_proj * world * vec4(p, 1.0);
  v_color = a_color;
}
