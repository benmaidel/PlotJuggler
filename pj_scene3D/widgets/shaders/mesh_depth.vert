// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Depth-only caster pass for the directional shadow map: the QRhi counterpart of
// MeshRenderPass::renderDepthOnly. Position is the only attribute read, but the
// pipeline still declares the full mesh vertex STRIDE so the same VBO the visual
// pass uses can be bound unchanged.
layout(location = 0) in vec3 a_pos;

layout(std140, binding = 0) uniform DepthSceneUbo {
  // World -> light clip space, ALREADY multiplied by QRhi's clipSpaceCorrMatrix by
  // the host. So gl_Position.z leaves this shader in the backend's own NDC range,
  // and the receiver must not re-map it — see mesh.frag's shadowFactor().
  mat4 light_view_proj;
};

layout(std140, binding = 1) uniform DepthDrawUbo {
  mat4 model;
};

void main() {
  gl_Position = light_view_proj * model * vec4(a_pos, 1.0);
}
