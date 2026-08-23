// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Line and triangle batches. Unlike the instanced solids these carry per-vertex
// colours and are re-streamed every frame anyway, so the host bakes each batch's
// world placement straight into the vertices. That removes the per-batch matrix
// uniform entirely and lets every batch merge into ONE buffer and ONE draw — where
// the GL renderer issues a draw per batch.

layout(location = 0) in vec3 a_pos;  // already in world space
layout(location = 1) in vec4 a_color;

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
  gl_Position = view_proj * vec4(a_pos, 1.0);
  v_color = a_color;
}
