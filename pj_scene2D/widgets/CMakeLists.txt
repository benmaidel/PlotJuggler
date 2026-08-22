# ---------------------------------------------------------------------------
# pj_scene2d_widgets — Qt widget layer for media display (requires Qt 6.11+)
# ---------------------------------------------------------------------------

# GuiPrivate (for QRhi private headers used by MediaViewerWidget) must be
# requested in THIS scope: Qt 6.11 no longer leaks the Qt6::GuiPrivate imported
# target in from another subdirectory's find_package the way 6.8 did.
# ShaderTools provides qt6_add_shaders() / the `qsb` baker used below. Install it
# with the Qt SDK (`install_qt6.sh` passes `-m qtshadertools`).
find_package(Qt6 6.11.1 QUIET COMPONENTS Widgets Gui GuiPrivate ShaderTools)
if(NOT Qt6_FOUND)
  message(STATUS "pj_scene2d_widgets: Qt 6.11+ not found, skipping")
  return()
endif()
if(NOT TARGET Qt6::ShaderTools)
  message(FATAL_ERROR
    "pj_scene2d_widgets: Qt6 ShaderTools (the `qsb` shader baker) not found. "
    "Reinstall Qt with the qtshadertools module: ./install_qt6.sh")
endif()
message(STATUS "pj_scene2d_widgets: using Qt ${Qt6_VERSION}")

set(CMAKE_AUTOMOC ON)
set(CMAKE_AUTORCC ON)

# --- pj_scene2d_widgets: image viewer (QRhiWidget), no mpv dependency ---
set(PJ_SCENE2D_WIDGETS_SOURCES
  src/Scene2DDockWidget.cpp
  src/layers/depth_image_layer.cpp
  src/layers/image_layer.cpp
  src/layers/scene2d_layer.cpp
  src/layers/scene_decoder_layer.cpp
  src/media_viewer_widget.cpp
  src/pixel_inspector.cpp
  src/scene2d_pipelines.cpp
  include/pj_scene2d_widgets/Scene2DDockWidget.h
  include/pj_scene2d_widgets/layers/depth_image_layer.h
  include/pj_scene2d_widgets/layers/image_layer.h
  include/pj_scene2d_widgets/layers/scene2d_layer.h
  include/pj_scene2d_widgets/layers/scene_decoder_layer.h
  include/pj_scene2d_widgets/media_viewer_widget.h
  include/pj_scene2d_widgets/pixel_inspector.h
  include/pj_scene2d_widgets/scene2d_pipelines.h
)

get_target_property(_pj_scene2d_core_public_defs pj_scene2d_core INTERFACE_COMPILE_DEFINITIONS)
set(PJ_SCENE2D_WIDGETS_HAS_FFMPEG FALSE)
if(_pj_scene2d_core_public_defs AND "PJ_HAS_FFMPEG" IN_LIST _pj_scene2d_core_public_defs)
  set(PJ_SCENE2D_WIDGETS_HAS_FFMPEG TRUE)
  list(APPEND PJ_SCENE2D_WIDGETS_SOURCES
    src/layers/video_layer.cpp
    include/pj_scene2d_widgets/layers/video_layer.h
  )
endif()

add_library(pj_scene2d_widgets STATIC ${PJ_SCENE2D_WIDGETS_SOURCES})
target_include_directories(pj_scene2d_widgets PUBLIC include)
target_compile_features(pj_scene2d_widgets PUBLIC cxx_std_20)
target_compile_options(pj_scene2d_widgets PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
target_link_libraries(pj_scene2d_widgets
  PUBLIC pj_scene_common pj_scene2d_core pj_runtime pj_widgets Qt6::Widgets Qt6::Gui Qt6::GuiPrivate
)

# ---------------------------------------------------------------------------
# Shaders
# ---------------------------------------------------------------------------
#
# Bake the GLSL sources to Qt shader packs (.qsb) at BUILD time and embed them in
# a Qt resource. The `.qsb` blobs used to be hand-baked artifacts committed to
# git with no build rule, which meant (a) editing a `.vert`/`.frag` had no effect
# until someone remembered to re-run `qsb` by hand, and (b) the image shaders
# shipped a GLSL-440-only pack — unusable on macOS, whose OpenGL is frozen at 4.1.
#
# Targets: GLSL 410 + 440 for the OpenGL backend (410 is what a macOS GL context
# could offer; 440 matches the source level on desktop Linux) and MSL 12 for
# Metal. HLSL is deliberately NOT requested: nothing uses the D3D backends, and
# `scene_text.frag` does not survive HLSL translation, which would fail the build.
#
# PREFIX + BASE keep the runtime resource paths at ":/shaders/<name>.<stage>.qsb",
# and naming the resource `shaders` preserves the existing Q_INIT_RESOURCE(shaders)
# call that a STATIC library needs (see media_viewer_widget.cpp).
qt6_add_shaders(pj_scene2d_widgets "shaders"
  PREFIX "/shaders"
  BASE "shaders"
  GLSL "410,440"
  MSL 12
  FILES
    shaders/yuv_to_rgb.vert
    shaders/yuv_to_rgb.frag
    shaders/scene_lines.vert
    shaders/scene_lines.frag
    shaders/scene_quads.vert
    shaders/scene_quads.frag
    shaders/scene_text.vert
    shaders/scene_text.frag
)

# ---------------------------------------------------------------------------
# Tests
# ---------------------------------------------------------------------------

if(PJ_BUILD_TESTS)
  # Dispatch-table unit test: verifies makeScene2DPipelineFor() returns the right
  # built-in pipeline (or nullptr) per object type. Does not construct the
  # widget, so it does not require an RHI context.
  add_executable(scene2d_dock_widget_dispatch_test
    ../tests/scene2d_dock_widget_dispatch_test.cpp
  )
  target_link_libraries(scene2d_dock_widget_dispatch_test PRIVATE
    pj_scene2d_widgets
    GTest::gtest
  )
  target_compile_options(scene2d_dock_widget_dispatch_test PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  add_test(NAME scene2d_dock_widget_dispatch_test COMMAND scene2d_dock_widget_dispatch_test)

  add_executable(pixel_inspector_test
    ../tests/pixel_inspector_test.cpp
  )
  target_link_libraries(pixel_inspector_test PRIVATE
    pj_scene2d_widgets
    GTest::gtest_main
  )
  target_compile_options(pixel_inspector_test PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  add_test(NAME PixelInspector COMMAND pixel_inspector_test)

  add_executable(scene2d_dock_widget_test
    ../tests/scene2d_dock_widget_test.cpp
  )
  target_link_libraries(scene2d_dock_widget_test PRIVATE
    pj_scene2d_widgets
    GTest::gtest
  )
  target_compile_options(scene2d_dock_widget_test PRIVATE ${PJ_WARNING_FLAGS} ${PJ_SANITIZER_FLAGS})
  add_test(NAME scene2d_dock_widget_test COMMAND scene2d_dock_widget_test)
endif()
