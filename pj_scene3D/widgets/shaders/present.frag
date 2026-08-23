// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Composite / tonemap present. The off-screen HDR target holds LINEAR light; this
// pass applies exposure, tonemaps, boosts saturation and performs the single
// manual sRGB encode (the widget target is not sRGB-capable).
//
// AgX: adapted from three.js tonemapping_pars_fragment (MIT; Filament/Sobotka
// derived). ACES: Narkowicz (CC0). Neutral: Khronos PBR Neutral (Apache-2.0, via
// three.js). sRGB OETF: IEC 61966-2-1. Full licence texts: pj_scene3D/THIRDPARTY.md.

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
  // 0 None, 1 ACES, 2 AgX, 3 Khronos PBR Neutral.
  int tonemap_mode;
  // Post-tonemap saturation. Applied AFTER the tonemap on purpose: ACES in
  // particular desaturates, so this restores the punch rather than pre-boosting
  // into the tonemap's shoulder.
  float saturation;
  // 1.0 when u_depth holds a real resolved depth buffer. Without one the
  // far-plane background bypass cannot run; see the note in main().
  float has_depth;
  float pad0;
  float pad1;
  float pad2;
};

// The resolved (single-sample) HDR scene colour. Its ALPHA is not opacity: it is a
// per-pixel "grade this" marker (1 = data, 0 = annotation) — see main().
layout(binding = 1) uniform sampler2D u_scene;
// Resolved single-sample depth, used only for the background bypass.
layout(binding = 2) uniform sampler2D u_depth;

vec3 sRGB(vec3 c) {
  const bvec3 k = lessThanEqual(c, vec3(0.0031308));
  return mix((1.055 * pow(max(c, vec3(0.0)), vec3(1.0 / 2.4))) - 0.055, c * 12.92, vec3(k));
}

vec3 tonemapACES(vec3 x) {
  return clamp((x * ((2.51 * x) + 0.03)) / ((x * ((2.43 * x) + 0.59)) + 0.14), 0.0, 1.0);
}

vec3 agxContrast(vec3 x) {
  const vec3 x2 = x * x;
  const vec3 x4 = x2 * x2;
  return (15.5 * x4 * x2) - (40.14 * x4 * x) + (31.96 * x4) - (6.868 * x2 * x) + (0.4298 * x2) + (0.1191 * x) -
         0.00232;
}

// GLSL mat3 constructors are COLUMN-major; these match three.js's AgXInset/Outset
// columns exactly. Transposing them tints greys blue (the transposed outset's blue
// row sums to ~1.22), which is a subtle enough error to survive a casual look.
const mat3 AGX_IN =
    mat3(0.856627, 0.137319, 0.111898, 0.0951212, 0.761242, 0.0767994, 0.0482516, 0.101439, 0.811302);
const mat3 AGX_OUT =
    mat3(1.127101, -0.141330, -0.141330, -0.110607, 1.157824, -0.110607, -0.016494, -0.016494, 1.251936);
const mat3 S2R = mat3(0.627404, 0.069097, 0.016392, 0.329282, 0.919540, 0.088013, 0.043314, 0.011361, 0.895595);
const mat3 R2S = mat3(1.660500, -0.124551, -0.018151, -0.587641, 1.132900, -0.100579, -0.072850, -0.008349, 1.118730);

vec3 tonemapAgX(vec3 c) {
  c = S2R * c;
  c = AGX_IN * c;
  c = max(c, vec3(1e-10));
  c = log2(c);
  c = clamp((c + 12.47393) / (4.026069 + 12.47393), 0.0, 1.0);
  c = agxContrast(c);
  c = AGX_OUT * c;
  c = pow(max(c, vec3(0.0)), vec3(2.2));
  c = R2S * c;
  return clamp(c, 0.0, 1.0);
}

// Khronos PBR Neutral. Preserves hue and saturation far better than ACES (which
// shifts saturated colours), which matters here because the scene is full of
// turbo/viridis costmaps and colormapped clouds whose HUE IS THE DATA. Operates in
// linear; output is linear [0,1]. Source: Khronos glTF Sample Viewer, via three.js.
vec3 tonemapNeutral(vec3 c) {
  const float kStartCompression = 0.8 - 0.04;
  const float kDesaturation = 0.15;
  const float x = min(c.r, min(c.g, c.b));
  const float offset = x < 0.08 ? x - (6.25 * x * x) : 0.04;
  c -= offset;
  const float peak = max(c.r, max(c.g, c.b));
  if (peak < kStartCompression) {
    return c;
  }
  const float d = 1.0 - kStartCompression;
  const float new_peak = 1.0 - ((d * d) / (peak + d - kStartCompression));
  c *= new_peak / peak;
  const float g = 1.0 - (1.0 / ((kDesaturation * (peak - new_peak)) + 1.0));
  return mix(c, vec3(new_peak), g);
}

void main() {
  const vec2 uv = vec2(v_uv.x, mix(v_uv.y, 1.0 - v_uv.y, flip_v));
  const vec4 scene = texture(u_scene, uv);
  const vec3 hdr = scene.rgb * exposure;

  // SSAO and EDL multiply in here in the GL renderer. Those passes are not ported
  // yet, so there is deliberately nothing to multiply — the slot is noted rather
  // than stubbed, so adding them is a local change.

  vec3 graded = tonemap_mode == 1   ? tonemapACES(hdr)
                : tonemap_mode == 2 ? tonemapAgX(hdr)
                : tonemap_mode == 3 ? tonemapNeutral(hdr)
                                    : clamp(hdr, vec3(0.0), vec3(1.0));
  const float luma = dot(graded, vec3(0.2126, 0.7152, 0.0722));
  graded = clamp(mix(vec3(luma), graded, saturation), vec3(0.0), vec3(1.0));

  // Background bypass: the clear colour is written already-linearized, so grading
  // it would shift the theme background. Far-plane pixels therefore pass straight
  // through and the sRGB encode below returns the exact theme colour.
  //
  // Without a resolved depth buffer this cannot run, and the host instead clears
  // alpha to 0 so the marker path below bypasses the background for us. That
  // fallback grades translucent-data-over-background slightly differently, which
  // is why it is the fallback and not the primary mechanism.
  if (has_depth != 0.0 && texture(u_depth, uv).r >= 0.999999) {
    graded = clamp(scene.rgb, vec3(0.0), vec3(1.0));
  }

  // The scene alpha is a per-pixel GRADE MARKER, not opacity: annotation geometry
  // (TF triads, connection lines) drives it toward 0 so synthetic markers keep
  // their flat vivid colours, while data objects (alpha 1) take the filmic look.
  // The MSAA resolve averages the marker, which feathers the seam between them.
  const vec3 ldr = mix(clamp(scene.rgb, vec3(0.0), vec3(1.0)), graded, scene.a);
  // Alpha forced to 1: the Wayland opaque-surface invariant.
  frag_color = vec4(sRGB(ldr), 1.0);
}
