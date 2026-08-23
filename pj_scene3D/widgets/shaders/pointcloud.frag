// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec2 v_corner;
layout(location = 1) in float v_t;
layout(location = 0) out vec4 frag_color;

layout(std140, binding = 0) uniform PointcloudUbo {
  mat4 view_proj;
  vec4 cam_right;
  vec4 cam_up;
  float point_radius;
  float scalar_min;
  float scalar_max;
  float colormap_row;
};

// Colormap lookup table: one ROW per colormap, columns sample t in [0,1].
//
// The GL renderer instead concatenates pj_widgets' colormapGlsl() into three of
// its shaders at RUNTIME, which cannot work here — qsb bakes fixed sources ahead
// of time, so a runtime-assembled program has nothing to bake. Sampling the same
// LUT that pj_scene2D's depth path already uses removes the concatenation
// entirely and makes 2D and 3D agree on colour by construction.
layout(binding = 1) uniform sampler2D u_colormap;

void main() {
  // Round sprite: discard outside the inscribed circle so the quad reads as a
  // dot rather than a square.
  const float r2 = dot(v_corner, v_corner);
  if (r2 > 1.0) {
    discard;
  }
  const vec3 rgb = texture(u_colormap, vec2(v_t, colormap_row)).rgb;
  // Cheap sphere-imposter shading: treat the disc as a hemisphere and light it
  // from the view direction, so a dense cloud reads as volumetric instead of flat.
  const float z = sqrt(max(1.0 - r2, 0.0));
  const float shade = 0.55 + (0.45 * z);
  frag_color = vec4(rgb * shade, 1.0);
}
