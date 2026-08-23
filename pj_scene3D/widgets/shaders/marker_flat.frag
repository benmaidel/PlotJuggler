// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0
#version 440

// Shared by EVERY marker pipeline. Marker geometry is annotation, not lit
// material: the exact marker colour must show on every face regardless of normal
// direction or camera orientation, so there is no shading term at all. The GL
// renderer pairs all five of its marker programs with this same flat output.
layout(location = 0) in vec4 v_color;
layout(location = 0) out vec4 frag_color;

void main() {
  frag_color = vec4(pow(max(v_color.rgb, vec3(0.0)), vec3(2.2)), v_color.a);
}
