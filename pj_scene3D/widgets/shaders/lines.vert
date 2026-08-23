// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Generic coloured-line vertex, shared by every pass that draws world-space line
// segments (the ground grid, TF parent connections). Only the position is
// consumed, so a caller may use any vertex stride that keeps position at the
// declared offset — the grid's 16-byte GridVertex (pos + parity) and a tight
// 12-byte vec3 both work.
layout(location = 0) in vec3 a_pos;

layout(std140, binding = 0) uniform LinesUbo {
  mat4 view_proj;
  vec4 line_color;
};

void main() {
  gl_Position = view_proj * vec4(a_pos, 1.0);
}
