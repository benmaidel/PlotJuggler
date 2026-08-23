// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// 4x4 box blur over the AO tile (LearnOpenGL, SSAO blur). The tile size matches
// the ssao pass's 4x4 noise tiling exactly, which is what makes the per-pixel
// kernel rotation average out to a smooth field instead of a visible grid.

// Declared to match present.vert's output interface; deliberately unused.
layout(location = 0) in vec2 v_uv;
layout(location = 0) out float blur_out;

layout(std140, binding = 0) uniform BlurUbo {
  vec2 texel;  // 1 / AO texture size
  float pad0;
  float pad1;
};

layout(binding = 1) uniform sampler2D u_ao;

void main() {
  // The fragment's own texel, NOT the interpolated v_uv. gl_FragCoord's origin
  // follows the render target's orientation and the sampled texture's row 0 follows
  // the same convention, so this addresses the matching texel under BOTH OpenGL's
  // y-up framebuffers and Metal's y-down ones — whereas an NDC-derived uv would be
  // vertically mirrored on one of them. It is also the exact convention
  // RhiFrameContext::view_from_screen is built against.
  const vec2 uv = gl_FragCoord.xy * texel;
  float sum = 0.0;
  for (int x = -2; x < 2; ++x) {
    for (int y = -2; y < 2; ++y) {
      sum += texture(u_ao, uv + (vec2(float(x), float(y)) * texel)).r;
    }
  }
  blur_out = sum / 16.0;
}
