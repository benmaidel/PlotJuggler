// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 frag_color;

// std140 uniform block. The GL renderer sets ~152 loose `uniform` scalars by
// name, which QRhi has no equivalent for, so every ported pass must move to a
// block like this. Field order/padding here is the convention the port follows:
// vec4s first, scalars packed after, explicit padding to a 16-byte boundary.
layout(std140, binding = 0) uniform ProbeUbo {
  vec4 bottom_color;
  vec4 top_color;
  vec4 marker_color;
  vec2 marker_extent;  // fraction of the surface the corner marker covers
  float pad0;
  float pad1;
};

void main() {
  vec3 rgb = mix(bottom_color.rgb, top_color.rgb, v_uv.y);
  // Asymmetric marker at scene-space (0,0) — the LOW-Y corner. A symmetric probe
  // could not distinguish a correct render from a vertically flipped one, which
  // is the failure Metal's opposite framebuffer Y convention actually produces.
  if (v_uv.x < marker_extent.x && v_uv.y < marker_extent.y) {
    rgb = marker_color.rgb;
  }
  frag_color = vec4(rgb, 1.0);
}
