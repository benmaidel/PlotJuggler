// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

layout(location = 0) in vec3 v_world_pos;
layout(location = 1) in vec3 v_world_normal;
layout(location = 2) in vec4 v_vertex_color;
layout(location = 3) in vec2 v_uv;
layout(location = 4) in vec3 v_tangent;
layout(location = 5) in vec3 v_bitangent;

layout(location = 0) out vec4 frag_color;

layout(std140, binding = 0) uniform SceneUbo {
  mat4 view_proj;
  vec4 camera_pos;
  vec4 key_light_dir;
  vec4 light_scales;  // ambient, direct(key), fill, env_intensity
  vec4 render_flags;  // srgb_encode, unused x3
};

layout(std140, binding = 1) uniform DrawUbo {
  mat4 model;
  mat4 normal_mat;
  vec4 base_color_factor;
  vec4 object_tint;
  vec4 emissive_factor;
  vec4 material;          // metallic, roughness, dielectric_f0, opacity
  vec4 alpha;             // alpha_mode, alpha_cutoff, has_normal_tex, is_collision
  vec4 use_vertex_color;
};

// The five glTF metallic-roughness maps. EVERY material binds all five, because a
// QRhi pipeline is compiled against its binding layout and that layout must be
// final at pipeline-creation time. Absent maps bind a 1x1 NEUTRAL texel instead of
// needing a per-slot "has texture" flag: white is the identity for the four
// multiplicative slots. The normal map is the exception — its neutral (0,0,1) is
// only a no-op when the tangent basis is well-formed, and meshes without UVs have
// a degenerate one — so it keeps an explicit flag (alpha.z).
layout(binding = 2) uniform sampler2D u_base_tex;
layout(binding = 3) uniform sampler2D u_mr_tex;
layout(binding = 4) uniform sampler2D u_normal_tex;
layout(binding = 5) uniform sampler2D u_ao_tex;
layout(binding = 6) uniform sampler2D u_emissive_tex;

const float PI = 3.14159265;

float D_GGX(float NoH, float a) {
  const float a2 = a * a;
  const float d = (NoH * NoH) * (a2 - 1.0) + 1.0;
  return a2 / max(PI * d * d, 1e-5);
}

float V_SmithGGXCorrelated(float NoV, float NoL, float a) {
  const float a2 = a * a;
  const float gv = NoL * sqrt((NoV * NoV * (1.0 - a2)) + a2);
  const float gl = NoV * sqrt((NoL * NoL * (1.0 - a2)) + a2);
  return 0.5 / max(gv + gl, 1e-5);
}

vec3 F_Schlick(vec3 f0, float VoH) {
  return f0 + ((1.0 - f0) * pow(clamp(1.0 - VoH, 0.0, 1.0), 5.0));
}

// One analytic light's outgoing radiance (Lambert diffuse + GGX specular), already
// weighted by N.L. Shared by the key and fill lights.
vec3 shadeLight(vec3 N, vec3 V, float NoV, vec3 L, vec3 diffuse_color, vec3 f0, float a) {
  const vec3 H = normalize(V + L);
  const float NoL = max(dot(N, L), 0.0);
  const float NoH = max(dot(N, H), 0.0);
  const float VoH = max(dot(V, H), 0.0);
  const vec3 F = F_Schlick(f0, VoH);
  const float spec = D_GGX(NoH, a) * V_SmithGGXCorrelated(NoV, NoL, a);
  return (((diffuse_color / PI) * (1.0 - F)) + (F * spec)) * NoL;
}

// Karis' analytic "environment BRDF" — the split-sum DFG term without a LUT
// (B. Karis, "Physically Based Shading on Mobile", Epic Games, 2014).
vec2 envBRDFApprox(float NoV, float roughness) {
  const vec4 c0 = vec4(-1.0, -0.0275, -0.572, 0.022);
  const vec4 c1 = vec4(1.0, 0.0425, 1.04, -0.04);
  const vec4 r = (roughness * c0) + c1;
  const float a004 = (min(r.x * r.x, exp2(-9.28 * NoV)) * r.x) + r.y;
  return (vec2(-1.04, 1.04) * a004) + r.zw;
}

// Procedural environment radiance for a world direction (Z-up): a ground->sky
// vertical gradient. The same hemisphere the diffuse ambient integrates, so the
// reflection and the ambient agree. An analytic stand-in for a prefiltered IBL
// cube — no texture, no extra pass.
const vec3 kEnvGround = vec3(0.28, 0.27, 0.25);
const vec3 kEnvSky = vec3(0.50, 0.52, 0.55);
vec3 envRadiance(vec3 dir) {
  return mix(kEnvGround, kEnvSky, clamp((dir.z * 0.5) + 0.5, 0.0, 1.0));
}

void main() {
  const float metallic_in = material.x;
  const float roughness_in = material.y;
  const float dielectric_f0 = material.z;
  const float opacity = material.w;
  const int alpha_mode = int(alpha.x);
  const float alpha_cutoff = alpha.y;
  const bool has_normal_tex = alpha.z != 0.0;
  const bool is_collision = alpha.w != 0.0;

  // Base colour, two modes matching the draw-call contract:
  //  - material-driven: glTF baseColorFactor is LINEAR, times the genuine
  //    per-vertex COLOR_0 (display-sRGB, linearized; white when absent).
  //  - override: the per-draw colour (URDF link / marker / placeholder) is
  //    display-sRGB and REPLACES the material factor.
  vec3 base_rgb;
  float base_a;
  if (use_vertex_color.x != 0.0) {
    base_rgb = base_color_factor.rgb * pow(max(v_vertex_color.rgb, vec3(0.0)), vec3(2.2));
    base_a = base_color_factor.a * v_vertex_color.a;
  } else {
    base_rgb = pow(max(object_tint.rgb, vec3(0.0)), vec3(2.2));
    base_a = object_tint.a;
  }
  const vec4 base_tex = texture(u_base_tex, v_uv);
  base_rgb *= base_tex.rgb;
  base_a *= base_tex.a;

  if (alpha_mode == 1 && base_a < alpha_cutoff) {
    discard;
  }
  const vec3 base = max(base_rgb, vec3(0.0));
  const float out_alpha = clamp(base_a * opacity, 0.0, 1.0);

  // Metallic-roughness, glTF channel packing: G = roughness, B = metallic.
  const vec3 mr = texture(u_mr_tex, v_uv).rgb;
  float metal = clamp(metallic_in, 0.0, 1.0) * mr.b;
  float roughness = (is_collision ? 0.85 : roughness_in) * mr.g;
  roughness = clamp(roughness, 0.045, 1.0);
  const float a = roughness * roughness;

  vec3 N = normalize(v_world_normal);
  if (has_normal_tex) {
    const mat3 tbn = mat3(normalize(v_tangent), normalize(v_bitangent), N);
    N = normalize(tbn * ((texture(u_normal_tex, v_uv).xyz * 2.0) - 1.0));
  }
  const vec3 V = normalize(camera_pos.xyz - v_world_pos);
  const float NoV = max(dot(N, V), 0.0);

  // Metalness workflow: metals take f0 from albedo and have no diffuse lobe.
  const vec3 f0 = mix(vec3(dielectric_f0), base, metal);
  const vec3 diffuse_color = base * (1.0 - metal);

  // Two analytic lights: a fixed world-space KEY ("sun") so shape reads the same
  // as the camera orbits, plus a dimmer camera-locked FILL headlight so the
  // viewer-facing side never goes black.
  const vec3 Lkey = key_light_dir.xyz;
  const vec3 Lfill = normalize(V + vec3(0.0, 0.0, 0.25));
  const vec3 direct = (shadeLight(N, V, NoV, Lkey, diffuse_color, f0, a) * light_scales.y) +
                      (shadeLight(N, V, NoV, Lfill, diffuse_color, f0, a) * light_scales.z);

  // Image-based ambient (analytic IBL): diffuse irradiance from the hemisphere plus
  // the split-sum specular reflection of the procedural environment. The specular
  // term is what makes metals read as metal — a metal has ~no diffuse and would
  // otherwise be almost black under ambient alone.
  const float ao = texture(u_ao_tex, v_uv).r;
  const vec3 diffuse_ibl = envRadiance(N) * diffuse_color;
  const vec3 refl = reflect(-V, N);
  const vec3 prefiltered = envRadiance(mix(refl, N, roughness));  // roughness blur
  const vec2 dfg = envBRDFApprox(NoV, roughness);
  const vec3 specular_ibl = prefiltered * ((f0 * dfg.x) + dfg.y) * light_scales.w;
  const vec3 ambient = (diffuse_ibl + specular_ibl) * ao * light_scales.x;

  vec3 color = ambient + direct;
  if (is_collision) {
    color = mix(color, base, 0.35);
  }
  color += emissive_factor.rgb * texture(u_emissive_tex, v_uv).rgb;

  // The shading above is genuine linear-light PBR, but the QRhi present pass is
  // still a passthrough (its tonemap + sRGB encode are a separate port step). So
  // encode here while that is true, driven by a host flag rather than by editing
  // this shader later: when the composite operators land, the host clears
  // render_flags.x and this becomes a no-op with no shader change.
  if (render_flags.x != 0.0) {
    color = pow(max(color, vec3(0.0)), vec3(1.0 / 2.2));
  }
  frag_color = vec4(color, out_alpha);
}
