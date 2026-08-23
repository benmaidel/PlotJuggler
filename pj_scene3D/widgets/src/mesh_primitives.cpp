// Copyright 2026 Davide Faconti
// SPDX-License-Identifier: MPL-2.0

#include "pj_scene3d_widgets/mesh_primitives.h"

#include <cmath>
#include <cstdint>
#include <iterator>
#include <memory>
#include <numbers>

namespace pj::scene3d {

Material solidColorMaterial(glm::vec4 color) {
  Material material;
  material.base_color_factor = color;
  return material;
}

MeshData makeCube(glm::vec4 color) {
  constexpr glm::vec3 kPositions[] = {
      {0.5f, -0.5f, -0.5f},  {0.5f, -0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},   {0.5f, 0.5f, -0.5f},   {-0.5f, -0.5f, 0.5f},
      {-0.5f, -0.5f, -0.5f}, {-0.5f, 0.5f, -0.5f}, {-0.5f, 0.5f, 0.5f},  {-0.5f, 0.5f, -0.5f},  {0.5f, 0.5f, -0.5f},
      {0.5f, 0.5f, 0.5f},    {-0.5f, 0.5f, 0.5f},  {-0.5f, -0.5f, 0.5f}, {0.5f, -0.5f, 0.5f},   {0.5f, -0.5f, -0.5f},
      {-0.5f, -0.5f, -0.5f}, {-0.5f, -0.5f, 0.5f}, {-0.5f, 0.5f, 0.5f},  {0.5f, 0.5f, 0.5f},    {0.5f, -0.5f, 0.5f},
      {0.5f, -0.5f, -0.5f},  {0.5f, 0.5f, -0.5f},  {-0.5f, 0.5f, -0.5f}, {-0.5f, -0.5f, -0.5f},
  };
  constexpr glm::vec3 kNormals[] = {
      {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0}, {-1, 0, 0},
      {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, 1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0}, {0, -1, 0},
      {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, 1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1}, {0, 0, -1},
  };
  constexpr std::uint32_t kIndices[] = {
      0,  1,  2,  0,  2,  3,  4,  5,  6,  4,  6,  7,  8,  9,  10, 8,  10, 11,
      12, 13, 14, 12, 14, 15, 16, 17, 18, 16, 18, 19, 20, 21, 22, 20, 22, 23,
  };
  MeshData out;
  out.ok = true;
  out.vertices.reserve(std::size(kPositions));
  for (std::size_t i = 0; i < std::size(kPositions); ++i) {
    out.vertices.push_back(Vertex{kPositions[i], kNormals[i], color});
  }
  out.indices.assign(std::begin(kIndices), std::end(kIndices));
  out.submeshes.push_back(SubMesh{0, out.indices.size(), std::make_shared<const Material>(solidColorMaterial(color))});
  return out;
}

MeshData makeCylinder() {
  constexpr int kSegments = 40;
  MeshData out;
  out.ok = true;
  const glm::vec4 color{0.7f, 0.7f, 0.7f, 1.0f};
  for (int i = 0; i < kSegments; ++i) {
    const float a = static_cast<float>(i) * 2.0f * std::numbers::pi_v<float> / static_cast<float>(kSegments);
    const float x = std::cos(a);
    const float y = std::sin(a);
    out.vertices.push_back(Vertex{{x, y, -0.5f}, glm::normalize(glm::vec3{x, y, 0.0f}), color});
    out.vertices.push_back(Vertex{{x, y, 0.5f}, glm::normalize(glm::vec3{x, y, 0.0f}), color});
  }
  const std::uint32_t top_center = static_cast<std::uint32_t>(out.vertices.size());
  out.vertices.push_back(Vertex{{0, 0, 0.5f}, {0, 0, 1}, color});
  const std::uint32_t bottom_center = static_cast<std::uint32_t>(out.vertices.size());
  out.vertices.push_back(Vertex{{0, 0, -0.5f}, {0, 0, -1}, color});
  for (int i = 0; i < kSegments; ++i) {
    const auto a = static_cast<std::uint32_t>(2 * i);
    const auto b = static_cast<std::uint32_t>(2 * ((i + 1) % kSegments));
    out.indices.insert(out.indices.end(), {a, b, a + 1, b, b + 1, a + 1});
    out.indices.insert(out.indices.end(), {top_center, a + 1, b + 1, bottom_center, b, a});
  }
  out.submeshes.push_back(SubMesh{0, out.indices.size(), std::make_shared<const Material>(solidColorMaterial(color))});
  return out;
}

MeshData makeSphere() {
  constexpr int kLat = 16;
  constexpr int kLon = 32;
  MeshData out;
  out.ok = true;
  const glm::vec4 color{0.7f, 0.7f, 0.7f, 1.0f};
  for (int lat = 0; lat <= kLat; ++lat) {
    const float theta = static_cast<float>(lat) * std::numbers::pi_v<float> / static_cast<float>(kLat);
    const float z = std::cos(theta);
    const float r = std::sin(theta);
    for (int lon = 0; lon <= kLon; ++lon) {
      const float phi = static_cast<float>(lon) * 2.0f * std::numbers::pi_v<float> / static_cast<float>(kLon);
      const glm::vec3 p{r * std::cos(phi), r * std::sin(phi), z};
      out.vertices.push_back(Vertex{p, glm::normalize(p), color});
    }
  }
  for (int lat = 0; lat < kLat; ++lat) {
    for (int lon = 0; lon < kLon; ++lon) {
      const auto a = static_cast<std::uint32_t>(lat * (kLon + 1) + lon);
      const auto b = static_cast<std::uint32_t>((lat + 1) * (kLon + 1) + lon);
      out.indices.insert(out.indices.end(), {a, b, a + 1, a + 1, b, b + 1});
    }
  }
  out.submeshes.push_back(SubMesh{0, out.indices.size(), std::make_shared<const Material>(solidColorMaterial(color))});
  return out;
}

}  // namespace pj::scene3d
