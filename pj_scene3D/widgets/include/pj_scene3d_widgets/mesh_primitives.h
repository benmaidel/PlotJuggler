#pragma once
// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

// Procedural MeshData for the URDF/marker primitive shapes. Pure CPU geometry —
// no Qt, no GL, no QRhi — so both renderer backends draw the identical box,
// cylinder and sphere instead of each carrying its own copy.

#include <glm/glm.hpp>

#include "pj_scene3d_widgets/mesh_data.h"

namespace pj::scene3d {

/// A solid, untextured material. `has_pbr` stays false, which is what makes the
/// primitives inherit the scene-wide MeshShadingParams roughness/reflectivity
/// rather than glTF material values they do not have.
[[nodiscard]] Material solidColorMaterial(glm::vec4 color);

/// Unit cube centred on the origin, flat-shaded (24 vertices: per-face normals,
/// so the edges stay crisp). No UVs or tangents.
[[nodiscard]] MeshData makeCube(glm::vec4 color);

/// Unit-radius, unit-height cylinder about +Z, centred on the origin. Smooth side
/// normals, flat caps.
[[nodiscard]] MeshData makeCylinder();

/// Unit-radius sphere centred on the origin, smooth-shaded.
[[nodiscard]] MeshData makeSphere();

}  // namespace pj::scene3d
