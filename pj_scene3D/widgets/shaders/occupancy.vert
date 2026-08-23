// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) out vec2 v_uv;

layout(std140, binding = 0) uniform OccupancyUbo {
  // Full transform for the unit quad: view_proj * (frame pose * grid extent).
  mat4 mvp;
  float opacity;
  // 0 = Map (grayscale), 1 = Costmap (blue -> red).
  int color_scheme;
  float pad0;
  float pad1;
};

// Unit quad in the local xy-plane, corners from gl_VertexIndex so the pass binds
// no vertex buffer at all. uv doubles as position: the grid's model matrix maps
// [0,1]^2 onto the cell extent, and row-major cell (r,c) maps to uv
// (c/width, r/height).
vec2 cornerFor(int index) {
  const vec2 c[6] = vec2[6](vec2(0.0, 0.0), vec2(1.0, 0.0), vec2(1.0, 1.0),
                            vec2(0.0, 0.0), vec2(1.0, 1.0), vec2(0.0, 1.0));
  return c[index];
}

void main() {
  const vec2 uv = cornerFor(gl_VertexIndex % 6);
  v_uv = uv;
  gl_Position = mvp * vec4(uv, 0.0, 1.0);
}
