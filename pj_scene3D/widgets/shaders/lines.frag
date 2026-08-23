// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) out vec4 frag_color;

// Must match lines.vert field for field: QRhi's OpenGL backend maps std140 block
// members BY NAME from the SPIRV-Cross output, so both stages declare the whole
// block even when one of them reads only part of it.
layout(std140, binding = 0) uniform LinesUbo {
  mat4 view_proj;
  vec4 line_color;
};

void main() {
  // The ALPHA is not opacity here — nothing blends this pass. It is the composite's
  // per-pixel grade marker: the grid passes 1 (data, graded) and the TF connection
  // lines pass 0 (annotation, ungraded) so their magenta survives the tonemap.
  // Enabling blending on this pass would silently turn that marker into coverage.
  frag_color = vec4(pow(max(line_color.rgb, vec3(0.0)), vec3(2.2)), line_color.a);
}
