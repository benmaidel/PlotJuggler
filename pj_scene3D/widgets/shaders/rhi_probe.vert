// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Attributeless fullscreen triangle: the same trick the GL renderer's present
// pass uses (glDrawArrays(GL_TRIANGLES, 0, 3) with no vertex buffer bound), here
// driven by gl_VertexIndex so it survives SPIR-V translation to MSL/HLSL.
layout(location = 0) out vec2 v_uv;

void main() {
  float x = float(gl_VertexIndex == 1) * 4.0 - 1.0;
  float y = float(gl_VertexIndex == 2) * 4.0 - 1.0;
  gl_Position = vec4(x, y, 0.0, 1.0);
  // UV in the [0,1] range with +Y UP in *scene* terms. Whether that lands at the
  // top or bottom of the presented image is exactly what the probe measures.
  v_uv = vec2(x, y) * 0.5 + 0.5;
}
