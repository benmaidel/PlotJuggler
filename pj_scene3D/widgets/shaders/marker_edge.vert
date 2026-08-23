// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Cube edge overlay: every box marker also gets its 12 edges as lines, which is
// what keeps a box readable when it is translucent or seen against a same-coloured
// backdrop.
//
// Each endpoint carries the TWO outward normals of the faces adjacent to its edge
// plus the edge's centre. If neither adjacent face turns toward the camera, the
// edge is on the far side of the box and is drawn in a softer colour, so the
// silhouette still reads as foreground while rear edges stay visible.
//
// The per-instance layout matches marker_solid.vert exactly (world at 2-5, colour
// at 6), so this replays the very same instance buffer the cube fill consumed.

layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal_a;
layout(location = 2) in vec4 a_world_c0;
layout(location = 3) in vec4 a_world_c1;
layout(location = 4) in vec4 a_world_c2;
layout(location = 5) in vec4 a_world_c3;
layout(location = 6) in vec4 a_color;
layout(location = 7) in vec3 a_normal_b;
layout(location = 8) in vec3 a_edge_center;

layout(location = 0) out vec4 v_color;

layout(std140, binding = 0) uniform MarkerUbo {
  mat4 view_proj;
  // Unused by the marker shaders — each primitive's frame transform is already
  // baked into its instance matrix / vertices. It is declared anyway so this block
  // shares a PREFIX with arrow.vert's ArrowUbo, letting the arrow and axes markers
  // reuse the shared arrow shaders off this very same buffer. Drop it only if
  // arrows stop sharing that pair.
  mat4 frame_world;
  // darken, hidden_mix, visible_epsilon.
  vec4 edge_params;
  // World-space eye. The GL shader did this test in view space, where the eye is
  // the origin by construction; here only the combined view_proj is available, so
  // the camera position is passed in and the test happens in world space instead.
  vec4 camera_pos;
};

void main() {
  const mat4 world = mat4(a_world_c0, a_world_c1, a_world_c2, a_world_c3);
  // World-space normals. Instance matrices may carry non-uniform scale (a box is a
  // scaled unit cube), so the inverse-transpose is required, not the plain 3x3.
  const mat3 normal_matrix = transpose(inverse(mat3(world)));
  const vec3 n_a = normalize(normal_matrix * a_normal_a);
  const vec3 n_b = normalize(normal_matrix * a_normal_b);

  const vec3 center_world = (world * vec4(a_edge_center, 1.0)).xyz;
  const vec3 delta = camera_pos.xyz - center_world;
  const float dist = length(delta);
  const vec3 to_camera = dist > 1e-6 ? delta / dist : vec3(0.0, 0.0, 1.0);

  const float facing = max(dot(n_a, to_camera), dot(n_b, to_camera));
  const vec3 front_color = a_color.rgb * edge_params.x;
  const vec3 hidden_color = mix(a_color.rgb, front_color, edge_params.y);

  gl_Position = view_proj * world * vec4(a_pos, 1.0);
  // Edges are always opaque: they are the readability aid, so they must not fade
  // out with the fill's alpha.
  v_color = vec4(facing > edge_params.z ? front_color : hidden_color, 1.0);
}
