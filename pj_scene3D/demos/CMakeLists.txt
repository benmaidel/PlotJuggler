# ---------------------------------------------------------------------------
# pj_scene3D demos — standalone visual tests (no app, no store, no parser)
# ---------------------------------------------------------------------------

set(CMAKE_AUTOMOC ON)

# GuiPrivate (for <rhi/qrhi.h> in the QRhi probe) must be requested in THIS
# scope: Qt 6.11 no longer leaks the imported target in from another
# subdirectory's find_package the way 6.8 did.
find_package(Qt6 REQUIRED COMPONENTS GuiPrivate)

if(TARGET pj_scene3d_widgets)
  # Self-verifying probe for the QRhi renderer that is replacing the GL path:
  # baked shader pack -> QRhi pipeline -> std140 UBO -> platform backend (Metal on
  # macOS) -> framebuffer readback, saved to a PNG. Also reports the corner colours
  # so the presented Y orientation is machine-checkable, since QRhi backends
  # disagree on framebuffer Y direction. Needs GuiPrivate for <rhi/qrhi.h>.
  add_executable(scene3d_rhi_probe rhi_probe.cpp)
  target_compile_features(scene3d_rhi_probe PRIVATE cxx_std_20)
  target_compile_options(scene3d_rhi_probe PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  target_link_libraries(scene3d_rhi_probe PRIVATE
    pj_scene3d_widgets Qt6::Widgets Qt6::GuiPrivate
  )

  add_executable(marker_gallery_demo marker_gallery_demo.cpp)
  target_compile_features(marker_gallery_demo PRIVATE cxx_std_20)
  target_link_libraries(marker_gallery_demo PRIVATE
    pj_scene3d_widgets pj_scene3d_core Qt6::Widgets
  )

  # PosesInFrame triad gizmos (PosesRenderPass / PosesInFrameLayer render path),
  # with a --screenshot mode for offscreen visual checks.
  add_executable(poses_gallery_demo poses_gallery_demo.cpp)
  target_compile_features(poses_gallery_demo PRIVATE cxx_std_20)
  target_link_libraries(poses_gallery_demo PRIVATE
    pj_scene3d_widgets pj_scene3d_core Qt6::Widgets
  )

  # Standalone URDF mesh viewer — fast look-development loop for the 3D mesh
  # pipeline without loading an MCAP. Reaches the private widgets/src headers
  # (urdf_package_resolver.h) the same way the module tests do.
  # resources.qrc gives the demo the same themed SVG icons the app registers
  # (AUTORCC embeds + auto-registers them for an executable target).
  set(CMAKE_AUTORCC ON)
  add_executable(scene3d_mesh_viewer mesh_viewer.cpp ${CMAKE_SOURCE_DIR}/resources/resources.qrc)
  target_compile_features(scene3d_mesh_viewer PRIVATE cxx_std_20)
  target_compile_options(scene3d_mesh_viewer PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  target_include_directories(scene3d_mesh_viewer PRIVATE ../widgets/src)
  target_link_libraries(scene3d_mesh_viewer PRIVATE pj_scene3d_widgets Qt6::Xml)

  # Self-verifying GPU-vs-CPU AABB-reduction benchmark (offscreen GL, no window).
  # Prints GL_RENDERER + timings and exits non-zero if the GPU result disagrees
  # with the CPU scan. Run with QT_QPA_PLATFORM=offscreen for headless CI.
  add_executable(pointcloud_aabb_benchmark pointcloud_aabb_benchmark.cpp)
  target_compile_features(pointcloud_aabb_benchmark PRIVATE cxx_std_20)
  target_compile_options(pointcloud_aabb_benchmark PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  target_link_libraries(pointcloud_aabb_benchmark PRIVATE pj_scene3d_widgets pj_scene3d_core Qt6::Gui)
endif()
