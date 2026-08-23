// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Shared by every instanced-arrow pass: the TF coordinate triads (RhiAxisPass)
// and the PoseArray triads (RhiPosesPass). They deliberately share one shader
// pair so pose gizmos and TF gizmos are guaranteed to read identically instead of
// relying on two copies staying in sync.

// Per-vertex: the shared arrow mesh (pj::scene3d::buildArrowMesh), interleaved
// position + normal, 24-byte stride.
layout(location = 0) in vec3 a_pos;
layout(location = 1) in vec3 a_normal;

// Per-instance: one arrow per arm.
//
// The model matrix is declared as FOUR vec4 columns rather than `in mat4`.
// GLSL allows a mat4 vertex input (consuming 4 consecutive locations), but the
// GL renderer's instanced passes rely on that and it is the shakiest construct to
// push through SPIRV-Cross to MSL/HLSL. Four explicit vec4s are unambiguous, map
// 1:1 onto QRhiVertexInputAttribute entries, and make the location budget
// obvious at a glance.
layout(location = 2) in vec4 a_model_c0;
layout(location = 3) in vec4 a_model_c1;
layout(location = 4) in vec4 a_model_c2;
layout(location = 5) in vec4 a_model_c3;
layout(location = 6) in vec4 a_color;

layout(location = 0) out vec3 v_normal;
layout(location = 1) out vec4 v_color;

layout(std140, binding = 0) uniform ArrowUbo {
  mat4 view_proj;
  // Placement of the instances' common parent frame, applied on top of each
  // per-instance model. Pose arrays keep their instances FRAME-LOCAL and put the
  // resolved fixed_frame<-source_frame TF here, so TF or camera motion re-uploads
  // nothing; passes whose instances are already world-space pass identity.
  mat4 frame_world;
};

void main() {
  const mat4 model = frame_world * mat4(a_model_c0, a_model_c1, a_model_c2, a_model_c3);
  gl_Position = view_proj * model * vec4(a_pos, 1.0);
  // The instance transforms are rotation + uniform scale + translation only, so
  // the upper-left 3x3 is a valid normal matrix (no inverse-transpose needed).
  v_normal = normalize(mat3(model) * a_normal);
  v_color = a_color;
}
