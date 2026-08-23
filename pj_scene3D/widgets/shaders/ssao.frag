// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Screen-space ambient occlusion over the resolved single-sample depth.
//
// SSAO adapted from LearnOpenGL (Joey de Vries, CC BY 4.0); depth->normal
// reconstruction adapted from Ben Golus (MIT). See pj_scene3D/THIRDPARTY.md.
//
// Two deliberate deviations, both inherited from the GL pass. Positions
// reconstruct through a matrix rather than a near/far formula, because the camera
// set includes an ORTHOGRAPHIC model the perspective-only formula breaks on. And
// the 4x4 noise texture is a 4x4-tiled in-shader hash — identical tiling semantics
// for the 4x4 blur, with no texture upload.

// Declared to match present.vert's output interface; deliberately unused — see the
// uv derivation in main().
layout(location = 0) in vec2 v_uv;
layout(location = 0) out float ssao_out;

const int kKernelSize = 32;

layout(std140, binding = 0) uniform SsaoUbo {
  // View <-> screen, where screen is (uv, depth) all in [0,1]. Supplied by the host
  // so this shader carries no NDC-convention knowledge; see RhiFrameContext.
  mat4 screen_from_view;
  mat4 view_from_screen;
  // xyz = hemisphere sample, w unused (std140 pads a vec3 array to vec4 anyway, so
  // the padding is spelled out rather than left implicit).
  vec4 kernel[kKernelSize];
  vec2 texel;  // 1 / depth texture size
  float radius;
  float bias;
  float ao_power;
  float pad0;
  float pad1;
  float pad2;
};

layout(binding = 1) uniform sampler2D u_depth;

vec3 viewPos(vec2 uv) {
  const float d = texture(u_depth, uv).r;
  const vec4 v = view_from_screen * vec4(uv, d, 1.0);
  return v.xyz / v.w;
}

// Normals come from the depth buffer rather than a G-buffer. Each axis picks
// whichever neighbour is closer in depth, so the basis follows the near side of a
// silhouette instead of straddling it and producing a normal pointing at nothing.
vec3 normalFromDepth(vec2 uv) {
  const vec3 c = viewPos(uv);
  const vec3 l = viewPos(uv - vec2(texel.x, 0.0));
  const vec3 r = viewPos(uv + vec2(texel.x, 0.0));
  const vec3 d2 = viewPos(uv - vec2(0.0, texel.y));
  const vec3 u2 = viewPos(uv + vec2(0.0, texel.y));
  const vec3 hd = abs(l.z - c.z) < abs(r.z - c.z) ? (c - l) : (r - c);
  const vec3 vd = abs(d2.z - c.z) < abs(u2.z - c.z) ? (c - d2) : (u2 - c);
  return normalize(cross(hd, vd));
}

void main() {
  // The fragment's own texel, NOT the interpolated v_uv. gl_FragCoord's origin
  // follows the render target's orientation and the sampled texture's row 0 follows
  // the same convention, so this addresses the matching texel under BOTH OpenGL's
  // y-up framebuffers and Metal's y-down ones — whereas an NDC-derived uv would be
  // vertically mirrored on one of them. It is also the exact convention
  // RhiFrameContext::view_from_screen is built against.
  const vec2 uv = gl_FragCoord.xy * texel;
  const float d = texture(u_depth, uv).r;
  if (d >= 0.9999) {
    ssao_out = 1.0;  // far plane: background is never occluded
    return;
  }
  const vec3 P = viewPos(uv);
  const vec3 N = normalFromDepth(uv);

  // 4x4-tiled random rotation of the kernel, matching the tile the blur averages
  // over so the noise cancels exactly.
  const vec2 tile = vec2(ivec2(gl_FragCoord.xy) & 3);
  const float angle = fract(sin(dot(tile, vec2(12.9898, 78.233))) * 43758.5453) * 6.2831853;
  const vec3 rv = vec3(cos(angle), sin(angle), 0.0);
  const vec3 T = normalize(rv - (N * dot(rv, N)));
  const mat3 TBN = mat3(T, cross(N, T), N);

  float occ = 0.0;
  for (int i = 0; i < kKernelSize; ++i) {
    const vec3 sp = P + ((TBN * kernel[i].xyz) * radius);
    vec4 o = screen_from_view * vec4(sp, 1.0);
    o.xyz /= o.w;
    if (o.x < 0.0 || o.x > 1.0 || o.y < 0.0 || o.y > 1.0) {
      continue;
    }
    const float sz = viewPos(o.xy).z;
    // Range check: a sample whose depth difference far exceeds the radius belongs
    // to unrelated geometry, so it must not darken this pixel.
    const float rc = smoothstep(0.0, 1.0, radius / max(abs(P.z - sz), 1e-4));
    occ += (sz >= sp.z + bias ? 1.0 : 0.0) * rc;
  }
  ssao_out = pow(clamp(1.0 - (occ / float(kKernelSize)), 0.0, 1.0), ao_power);
}
