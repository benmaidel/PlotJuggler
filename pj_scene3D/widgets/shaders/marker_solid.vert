// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Instanced unit solids: one mesh drawn once per marker. Non-uniform scale baked
// into the per-instance matrix turns the unit cube into a box and the unit sphere
// into an ellipsoid, so cube and sphere markers share this one program.

layout(location = 0) in vec3 a_pos;
// Per-instance world matrix as four explicit vec4 columns (see arrow.vert for why
// not `in mat4`), then the marker colour with overrides already applied.
layout(location = 2) in vec4 a_world_c0;
layout(location = 3) in vec4 a_world_c1;
layout(location = 4) in vec4 a_world_c2;
layout(location = 5) in vec4 a_world_c3;
layout(location = 6) in vec4 a_color;

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
  gl_Position = view_proj * world * vec4(a_pos, 1.0);
  v_color = a_color;
}
