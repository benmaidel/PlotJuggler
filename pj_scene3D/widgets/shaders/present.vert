// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Attributeless fullscreen triangle. No vertex buffer is bound; the position and
// UV come from gl_VertexIndex, which survives SPIR-V translation to MSL/HLSL
// (unlike gl_VertexID) and needs an empty QRhiVertexInputLayout.
layout(location = 0) out vec2 v_uv;

void main() {
  float x = float(gl_VertexIndex == 1) * 4.0 - 1.0;
  float y = float(gl_VertexIndex == 2) * 4.0 - 1.0;
  gl_Position = vec4(x, y, 0.0, 1.0);
  v_uv = vec2(x, y) * 0.5 + 0.5;
}
