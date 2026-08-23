// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Ground-grid line vertex. The CPU side is pj::scene3d::GridVertex
// (glm::vec3 pos + float parity, 16-byte stride); this pass draws LINES only, so
// the parity slot is present in the buffer but not consumed here — a future
// checkerboard pass reads it at location 1.
layout(location = 0) in vec3 a_pos;

layout(std140, binding = 0) uniform GridUbo {
  mat4 view_proj;
  vec4 line_color;
};

void main() {
  gl_Position = view_proj * vec4(a_pos, 1.0);
}
