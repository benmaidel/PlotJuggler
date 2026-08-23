// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec3 v_normal;
layout(location = 1) in vec4 v_color;
layout(location = 0) out vec4 frag_color;

// Must match arrow.vert's block byte-for-byte: one binding cannot carry two
// different std140 definitions.
layout(std140, binding = 0) uniform ArrowUbo {
  mat4 view_proj;
  mat4 frame_world;
};

void main() {
  // Fixed headlight plus a generous ambient term: these are annotation gizmos, so
  // the axis colours must stay identifiable from any orbit angle rather than
  // falling into shadow. The GL renderer shades ArrowGizmo the same way.
  const vec3 light_dir = normalize(vec3(0.4, 0.5, 0.75));
  const float lambert = max(dot(normalize(v_normal), light_dir), 0.0);
  const float shade = 0.55 + (0.45 * lambert);
  frag_color = vec4(v_color.rgb * shade, v_color.a);
}
