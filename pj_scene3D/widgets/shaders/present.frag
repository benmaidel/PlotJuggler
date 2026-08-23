// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec2 v_uv;
layout(location = 0) out vec4 frag_color;

layout(std140, binding = 0) uniform PresentUbo {
  float exposure;
  // 1.0 when the source texture must be sampled with V flipped.
  //
  // clipSpaceCorrMatrix() fixes NDC (Y direction and depth range) for the
  // GEOMETRY, but says nothing about how a rendered texture is then SAMPLED:
  // OpenGL's framebuffer row 0 is the bottom, Metal/Vulkan/D3D's is the top. So
  // compositing an off-screen scene texture needs this second, independent
  // correction, driven by QRhi::isYUpInFramebuffer(). Without it the whole scene
  // presents upside down.
  float flip_v;
  float pad0;
  float pad1;
};

// The resolved (single-sample) HDR scene colour.
layout(binding = 1) uniform sampler2D u_scene;

void main() {
  const vec2 uv = vec2(v_uv.x, mix(v_uv.y, 1.0 - v_uv.y, flip_v));
  vec3 rgb = texture(u_scene, uv).rgb * exposure;
  // Deliberately a PASSTHROUGH: no tonemap and no sRGB encode yet. The GL
  // renderer's composite applies exposure -> tonemap (ACES/AgX/Neutral) ->
  // saturation -> manual sRGB encode, and porting those operators is a separate
  // step. Keeping this a straight copy means the HDR chain can be verified
  // against the pre-HDR screenshot with only ONE variable changed (MSAA).
  frag_color = vec4(rgb, 1.0);
}
