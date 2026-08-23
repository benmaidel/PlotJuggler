// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Per-instance: one point. Laid out so the pass can bind a point cloud's WIRE
// buffer directly when its layout matches (contiguous float32 xyz followed by a
// float32 scalar, 16-byte stride) — the zero-copy path the GL renderer also has.
// ONE vec4 attribute, not a vec3 plus a float at offset 12.
//
// A separate float attribute at offset 12 of a 16-byte stride read as zero on the
// Metal backend (verified with known data: the CPU record held 0/0.33/0.66/1 and
// the shader saw 0 for every instance, while the same attribute at offset 0 read
// correctly). A point record is exactly {x, y, z, scalar}, i.e. one vec4, so
// fetching it as a single attribute is both simpler and avoids that entirely.
layout(location = 0) in vec4 a_point;

layout(location = 0) out vec2 v_corner;
layout(location = 1) out float v_t;

layout(std140, binding = 0) uniform PointcloudUbo {
  mat4 view_proj;
  // Camera right/up in WORLD space, supplied by the CPU from the view matrix.
  // Expanding the billboard from these avoids passing the view matrix and
  // inverting it per vertex.
  vec4 cam_right;
  vec4 cam_up;
  // World-space point radius. Because the quad is expanded in world space the
  // apparent size falls off with distance under a perspective camera, which is
  // what gl_PointSize gave for free in the GL renderer.
  float point_radius;
  // Scalar range mapped onto the colormap; equal min/max means "flat colour".
  float scalar_min;
  float scalar_max;
  // Row of the colormap LUT texture, in texel centres.
  float colormap_row;
};

// The quad's six corners, derived from gl_VertexIndex instead of a per-vertex
// buffer. There is deliberately NO vertex buffer here: a two-binding layout
// (shared quad + per-instance data) silently delivered only the first component
// of the per-vertex vec2 through SPIRV-Cross/MSL, collapsing every billboard to a
// horizontal sliver. Generating the corner arithmetically removes that binding
// altogether, which is also one less buffer to upload and keeps the instance data
// at binding 0.
vec2 cornerFor(int index) {
  const vec2 corners[6] = vec2[6](vec2(-1.0, -1.0), vec2(1.0, -1.0), vec2(1.0, 1.0),
                                  vec2(-1.0, -1.0), vec2(1.0, 1.0), vec2(-1.0, 1.0));
  return corners[index];
}

void main() {
  const vec2 corner = cornerFor(gl_VertexIndex % 6);

  // Camera-facing billboard. This replaces GL_POINTS + gl_PointSize, which has NO
  // equivalent in QRhi (and none on Metal): a vertex shader cannot set point size
  // there, so round "points" have to be real geometry.
  const vec3 offset = ((corner.x * cam_right.xyz) + (corner.y * cam_up.xyz)) * point_radius;
  gl_Position = view_proj * vec4(a_point.xyz + offset, 1.0);

  v_corner = corner;
  const float span = scalar_max - scalar_min;
  v_t = span > 0.0 ? clamp((a_point.w - scalar_min) / span, 0.0, 1.0) : 0.0;
}
