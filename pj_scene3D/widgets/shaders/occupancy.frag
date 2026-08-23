// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 frag_color;

layout(std140, binding = 0) uniform OccupancyUbo {
  mat4 mvp;
  float opacity;
  int color_scheme;
  float pad0;
  float pad1;
};

// R8 cell values. On the backends this renderer targets (Metal, desktop GL) an R8
// texture reads through .r; QRhi::RedOrAlpha8IsRed reports where a backend puts it
// if that ever needs handling.
layout(binding = 1) uniform sampler2D u_grid;

void main() {
  // Byte encoding matches the canonical OccupancyGrid: 0..100 is occupancy
  // percent, 255 means unknown (-1 on the wire).
  const float raw = texture(u_grid, v_uv).r * 255.0;
  if (raw > 100.5) {
    discard;  // unknown / reserved cells are transparent, not black
  }
  const float occ = clamp(raw / 100.0, 0.0, 1.0);

  vec3 color;
  if (color_scheme == 1) {
    color = mix(vec3(0.0, 0.4, 1.0), vec3(1.0, 0.0, 0.0), occ);  // free -> lethal
  } else {
    const float g = 1.0 - occ;  // free = white, occupied = black
    color = vec3(g, g, g);
  }

  // NOTE: deliberately NOT linearized with pow(color, 2.2) as the GL renderer's
  // equivalent does. That linearization exists because the GL composite re-encodes
  // to sRGB; the QRhi present pass is still a passthrough, so linearizing here
  // would just render everything too dark. When the composite operators land
  // (tonemap + sRGB encode), EVERY QRhi pass must start writing linear light,
  // including this one. See docs/ARCHITECTURE.md, "QRhi / Metal port".
  frag_color = vec4(color, opacity);
}
