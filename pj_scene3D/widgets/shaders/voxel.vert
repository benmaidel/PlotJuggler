// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Per-vertex: one corner of the shared unit cube (pj::scene3d::kCubeVertices),
// position on +/-0.5 plus its outward face normal.
layout(location = 0) in vec3 a_corner;
layout(location = 1) in vec3 a_normal;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out float v_t;

layout(std140, binding = 0) uniform VoxelUbo {
  mat4 view_proj;
  mat4 model;           // grid-local -> world (frame pose * grid origin)
  vec4 cell_size;       // metric voxel size, .xyz
  ivec4 dims;           // column, row, slice counts in .xyz
  // 0 = all, 1 = nonzero, 2 = value >= threshold, 3 = threshold <= value <= range_hi
  int draw_mode;
  float threshold;
  float range_hi;
  // Value range mapped across the colormap.
  float color_lo;
  float color_hi;
  float colormap_row;
  float pad0;
  float pad1;
};

// The dense field. texelFetch in a VERTEX shader is the whole trick here: the
// lattice is never expanded on the CPU, so one instanced draw covers every voxel
// and a re-scrub to a cached grid re-uploads nothing.
layout(binding = 1) uniform sampler3D u_volume;

bool keepVoxel(float value) {
  if (draw_mode == 0) {
    return true;
  }
  if (draw_mode == 1) {
    return value != 0.0;
  }
  if (draw_mode == 2) {
    return value >= threshold;
  }
  return value >= threshold && value <= range_hi;
}

void main() {
  // Recover this instance's lattice coordinate. Draw-call cost is therefore
  // independent of voxel count: one instanced draw, no per-voxel CPU work.
  const int w = dims.x;
  const int h = dims.y;
  const int id = gl_InstanceIndex;
  const ivec3 cell = ivec3(id % w, (id / w) % h, id / (w * h));

  const float value = texelFetch(u_volume, cell, 0).r;
  if (!keepVoxel(value)) {
    // Push the vertex outside NDC so the whole cube is clipped away. Cheaper than
    // any CPU-side compaction, and it is what makes the predicate a viewer-side
    // decision the schema does not have to encode.
    gl_Position = vec4(2.0, 2.0, 2.0, 1.0);
    v_normal = vec3(0.0, 0.0, 1.0);
    v_t = 0.0;
    return;
  }

  const float span = max(color_hi - color_lo, 1e-9);
  v_t = clamp((value - color_lo) / span, 0.0, 1.0);

  const vec3 local_center = (vec3(cell) + 0.5) * cell_size.xyz;
  const vec3 local_vertex = local_center + (a_corner * cell_size.xyz);
  gl_Position = view_proj * model * vec4(local_vertex, 1.0);
  // model is a rigid transform plus a per-axis cell scale; for lighting the
  // rotation part is what matters, and the scale is uniform per axis so the
  // direction survives normalization.
  v_normal = normalize(mat3(model) * a_normal);
}
