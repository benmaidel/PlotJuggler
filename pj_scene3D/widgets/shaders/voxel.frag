// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec3 v_normal;
layout(location = 1) in float v_t;
layout(location = 0) out vec4 frag_color;

layout(std140, binding = 0) uniform VoxelUbo {
  mat4 view_proj;
  mat4 model;
  vec4 cell_size;
  ivec4 dims;
  int draw_mode;
  float threshold;
  float range_hi;
  float color_lo;
  float color_hi;
  float colormap_row;
  float pad0;
  float pad1;
};

layout(binding = 1) uniform sampler3D u_volume;
// Shared colormap LUT (row = colormap id, column = t), the same texture the
// point-cloud pass samples, so a scalar maps to one colour across the whole app.
layout(binding = 2) uniform sampler2D u_colormap;

void main() {
  const vec3 rgb = texture(u_colormap, vec2(v_t, colormap_row)).rgb;
  // Fixed key light plus generous ambient: dense voxel fields are read as bulk
  // shape, so faces must stay distinguishable from every orbit angle.
  const vec3 light_dir = normalize(vec3(0.35, 0.45, 0.82));
  const float lambert = max(dot(normalize(v_normal), light_dir), 0.0);
  frag_color = vec4(rgb * (0.55 + (0.45 * lambert)), 1.0);
}
