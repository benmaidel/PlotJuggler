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
  frag_color = line_color;
}
