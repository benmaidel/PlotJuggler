# pj_scene3D — Architecture (as built)

How the 3D scene renders and how robot models get on screen. This is the
distilled, as-built successor to the executed planning docs
(`PHOTOREALISM_AND_URDF_MESH_PLAN.md`, `URDF_MESH_ASSET_RESOLUTION_DESIGN.md`,
`CODEX_PROMPTS_URDF_MESH.md` — all removed 2026-06-10; see git history for the
full design rationale). The WHAT lives in [REQUIREMENTS.md](./REQUIREMENTS.md).

## QRhi / Metal port (in progress)

The renderer described below is the **OpenGL** one, and it is still the shipping
path on Linux. It cannot run on macOS at all: it needs an OpenGL 4.5 core context
plus a compute shader, and Apple's OpenGL is frozen at 4.1 with no compute. A
second renderer is therefore being built on Qt's **QRhi** (Metal on macOS, OpenGL
elsewhere) under `widgets/{include/pj_scene3d_widgets,src}/rhi/`, landing pass by
pass so Linux keeps the proven path until parity is reached.

Ported so far:

| Area | Class | Notes |
|---|---|---|
| Host | `RhiSceneViewWidget` | camera, depth, clip-space correction, pass ordering |
| Off-screen chain | `RhiHdrTarget` | MSAA RGBA16F + multisample depth texture + resolves |
| Composite / present | `RhiPresentPass` | exposure -> AO -> tonemap -> saturation -> sRGB encode |
| SSAO | `RhiSsaoPass` | 32-tap hemisphere kernel + 4x4 blur, off resolved depth |
| Ground grid | `RhiGridPass` | shares `shaders/lines.*` |
| TF connection lines | `RhiTfConnectionsPass` | shares `shaders/lines.*` |
| TF triads | `RhiAxisPass` | instanced; shares `shaders/arrow.*` |
| Pose arrays | `RhiPosesPass` | instanced; shares `shaders/arrow.*` |
| Point clouds | `RhiPointcloudPass` | instanced billboards, colormap LUT texture |
| Occupancy grid | `RhiOccupancyGridPass` | R8 texture, partial `updateRegion` uploads |
| Voxel grid | `RhiVoxelGridPass` | 3D R32F texture, `texelFetch` in the *vertex* stage |
| Meshes / PBR | `RhiMeshPass` | glTF metallic-roughness, 5 maps, analytic IBL |
| Markers | `RhiMarkerPass` | SceneEntities primitives; flat-shaded annotation |

Drive it with `demos/rhi_view.cpp` (`scene3d_rhi_view`), which renders one frame
headlessly to a PNG for comparison against `tools/reference/`.

**Coverage:** `tests/rhi_passes_test.cpp` renders every ported pass through an
offscreen QRhi using the production HDR chain and composite. Its assertions are
shaped around this port's actual failure mode rather than pixel goldens — "did
anything draw at all", "does the background survive the composite byte-exact", "is
annotation geometry still bypassing the tonemap" — because QRhi reports success for a
pipeline built against an empty binding layout, one whose sample count disagrees with
its target, or a uniform block smaller than the shader reads.

Two things make it worth more than its size suggests. It needs no OpenGL 4.5, so it
runs on macOS/Metal — where every `*_gl_test` reports *Passed* to ctest while
internally skipping every case, meaning a green suite there says nothing about the
OpenGL renderer. And it immediately found a latent crash: `RhiMeshPass` sized its
per-draw uniform staging only when GROWING past the capacity `initialize()` had
already set, so a scene with exactly one submesh wrote into an empty vector. Every
demo had seven or more, so nothing had ever reached it.

Two shader pairs are deliberately **shared** by more than one pass, because the
alternative is two copies of a std140 block or a vertex stride drifting apart —
exactly the class of bug QRhi does not report:

- `shaders/lines.*` — the grid and the TF connection lines.
- `shaders/arrow.*` — the TF triads and the pose-array gizmos. The GL renderer
  states as an intent that pose gizmos read identically to the TF "Frames" gizmos;
  one shader pair enforces that instead of trusting two copies. Their common GPU
  contract (the UBO and per-instance layouts) lives in `rhi/rhi_arrow_shading.h`.
  The UBO carries a `frame_world` matrix so a pass can keep its instances
  *frame-local* and place them with one uniform write — what makes a streaming
  pose array cheap to animate. `RhiAxisPass`, whose instances are already
  world-space, writes identity there.

**Still to do**, roughly in order of value:

1. *(Geometry passes, the composite operators and SSAO are complete. EDL is
   deliberately not ported — see below.)*
3. Screen-space passes: SSAO and EDL. Both need the resolved single-sample depth
   the HDR chain already produces — `QRhi::ResolveDepthStencil` is supported on
   Metal, so the design carries over unchanged.
4. The compute AABB reducer → `QRhiComputePipeline`, keeping the documented CPU
   fallback for backends without compute.
5. **Making the QRhi renderer render topic DATA in the app.** It is already
   reachable in the app — `Scene3DRhiPreviewDock` (opt-in via `PJ_SCENE3D_RHI`)
   hosts it in a real dock and draws the TF overlay plus the grid from the live
   `TransformService`. What is missing is everything driven by layers, and that is
   blocked on a genuine architectural prerequisite rather than on wiring:

   `Scene3DDockWidget` drives its content through `Scene3DLayer`, and the layer types
   **fuse decoding with OpenGL upload** — each implements `initializeGL()` /
   `render(ViewParams, FrameContext)` and owns GL `IRenderPass` objects internally,
   with no seam exposing the decoded render structs. The unlock is to split decode
   from upload per layer; see "The layer decode/upload split" below for the shape and
   for which layers are done.

### The layer decode/upload split

**Done: point clouds. Remaining: meshes/URDF, scene entities, occupancy grid, voxel
grid, poses-in-frame, depth cloud.**

The split point is not arbitrary — `PointCloudLayer`'s own header already named it:
`pushCloud()` is *"the single point where a cloud reaches the GPU"*. Everything above
that line (async Draco/Cloudini decode on the thread pool, sample-identity caching,
latest-wins coalescing, colour-field discovery, auto-range, XML state) is
backend-agnostic; only the final upload is not. So the seam goes exactly there:

- **`pointcloud_sink.h`** declares `IPointCloudSink` plus the display enums and
  `FastCloudData`, free of both OpenGL and QRhi. The enums used to be nested in
  `PointcloudRenderPass`; they moved here with `using` aliases left behind, so every
  `PointcloudRenderPass::Shape` call site still compiles untouched.
- **`PointcloudRenderPass` implements it**, and remains the layer's *default* sink.
  That is what makes the refactor behaviour-neutral for OpenGL: display state and
  data go through `sink()`, while the render-context lifecycle
  (`initializeGL`/`render`/`releaseGL`) stays on the concrete pass, since that part
  genuinely is backend-shaped.
- **`RhiPointCloudSink` adapts the same output onto `RhiPointcloudPass`**, so the
  QRhi renderer gets all of the decode machinery for free.

`PointCloudLayer::setSink()` redirects the output; passing nullptr restores the owned
OpenGL pass. Note the layer still *owns* a GL pass even when redirected — inert,
because the QRhi view never calls its GL hooks. Inverting that ownership is a later
step; it is called out here so nobody mistakes it for the intended end state.

**The QRhi pass is not at feature parity, and the adapter is where that shows.**
Supported: geometry (both interleaved and verbatim-wire), the source-frame placement,
colormap, range, radius, visibility. Silently ignored for want of a counterpart:
per-point RGB and solid colour, point shape, pixel sizing, LUT inversion, and the
spatial-axis auto-range — so an RGB cloud currently renders through the colormap.
Acceptable for a developer preview; first thing to fix when bringing the pass up to
parity. The GPU AABB reduction reports unavailable, which is the documented way for a
backend to say "keep your CPU bounds scan".

**TF placement is pushed, not resolved.** `RhiPointCloudSink::setFrameTransform()` is
deliberately NOT part of `IPointCloudSink`: the OpenGL pass resolves the
fixed_frame<-source_frame transform itself per frame from its `FrameContext`, whereas
the QRhi pass has no TF access, so whoever owns the TF buffer must push it and must
keep pushing it as TF moves. Every frame-placed layer ported after this one inherits
the same asymmetry.

This shipped broken once, and the reason is worth recording because it was NOT a weak
fixture — the fixture's `sensor` frame is a full 2 m from the origin, circling and
spinning. The cloud was rendering 2 m out of place, plainly visible in the harness
screenshot, and it was missed because the check was "is a cloud present" rather than
"is it centred on the sensor triad". Presence is the cheap thing to look at and the
one that proves least. `PointCloudHonoursItsModelMatrix` in `rhi_passes_test` now
asserts placement mechanically, which is what should have been guarding it.

### Scene3DRhiPreviewDock — what it is for

A developer preview, opt-in via `PJ_SCENE3D_RHI`, that puts the QRhi renderer inside
the running app. It deliberately hijacks the `scene3d` dock kind rather than adding a
visualization family, so it travels the app's genuine dock creation, drop,
float/split and layout-restore paths instead of a side door — which is the entire
point of it.

It exists to de-risk the item above. The OpenGL view's worst historical bugs were all
context recreation on ADS reparent (see the module `CLAUDE.md`), and `QRhiWidget` has
its own version of that in `releaseResources()`. Confirming a QRhiWidget survives the
app's docking lifecycle is worth doing *before* porting seven layers on top of it.

Verified on macOS/Metal, both headlessly via `tools/screenshot_3d.sh` with
`PJ_SCENE3D_RHI=1` and interactively: the view renders inside a restored ADS dock,
resolves the fixture's real TF tree (3 frames, 2 parent edges) against the live
`TransformService`, and **survives splitting the dock horizontally and vertically** —
the reparent path that historically broke the OpenGL view. Note that floating is not
a case to test: `DockWidget` disables it outright
(`setFeature(DockWidgetFloatable, false)`), so split, tab switch, close/recreate and
layout restore are the only reparent paths PJ4 has.

**It also fixes a real macOS defect, not just a portability gap.** The OpenGL 3D dock
on macOS corrupts the whole window — the entire UI is composited a second time,
vertically mirrored, over itself. That artifact is absent from the `QOpenGLWidget`'s
own `grabFramebuffer()` output, which localises it to how macOS/Qt composites that
widget into the ADS dock rather than to anything the renderer draws; switching the
same dock to `QRhiWidget`/Metal makes it disappear. So the payoff for finishing the
port is a *working* macOS 3D view, not parity for its own sake.

Two lifecycle traps it already surfaced, both worth knowing generally:

- A TF buffer is routinely bound *before* its transforms are ingested, so the frame
  hierarchy read at bind time is empty. The fixed frame therefore has to be
  re-chosen on later refreshes, or the view stays blank forever.
- `Qt::UniqueConnection` **silently does nothing for a lambda slot** (it needs a
  pointer-to-member, and Qt only emits a runtime warning). Connections to lambdas
  must be de-duplicated by holding the `QMetaObject::Connection` and disconnecting
  explicitly, which is what `Scene3DDockWidget` already does.

### EDL is deliberately not ported (decision, 2026-08-23)

The OpenGL renderer has an eye-dome-lighting pass, on by default. The QRhi renderer
does not, and this is a decision rather than a gap. Anyone reaching for it should
read this first.

**What it would buy.** EDL darkens a pixel by the log-depth gap to its 8 neighbours,
drawing contours at depth discontinuities. It comes from point-cloud viewers
(CloudCompare, Potree), where an unlit cloud has no shape cues at all and the
contour is the only depth information available. But PJ4's implementation is
deliberately **mesh-only** — clouds, grid, occupancy and axes neither receive nor
cast the contour — so it is not doing that job. What it delivers here is mesh
silhouette definition plus emphasis on the near side of surface creases, which
substantially overlaps what `RhiSsaoPass` already provides.

**What it would cost, and why QRhi is worse than GL here.** EDL needs the scene's R8
"is-mesh" mask. GL produces that by toggling `glDrawBuffers`, and gates the whole
thing on `edl_enabled` so nothing is paid when it is off. QRhi cannot: attachment
count is baked into the `QRhiRenderPassDescriptor` and every pipeline is compiled
against it. Supporting "EDL off ⇒ no mask attachment" therefore needs two render-pass
descriptors and two pipeline variants across **all nine passes**; the alternative is
attaching the mask permanently and paying its MSAA + resolve bandwidth every frame
regardless. On top of that, every fragment shader would have to declare and write
output location 1, since an attachment a shader leaves undeclared has undefined
contents.

So the port is both broader than GL's and less able to opt out of its own cost, for
the effect most redundant with SSAO.

**Consequence to be aware of:** scenes with meshes will not match the GL renderer's
look — no silhouette contour. EDL has no app UI and is not in `REQUIREMENTS.md`, so
nothing else depends on it.

**If it is ever revisited, consider retargeting it rather than porting it.** Applying
EDL to point clouds — what it is actually good at, and PJ4 renders a great many —
needs no is-mesh mask at all, which makes it a small self-contained pass on the
shape of `RhiSsaoPass`. That is a deliberate look change from 3.x, not a port, so it
needs sign-off.

### Screen-space passes

`RhiSsaoPass` is deliberately **not** an `IRhiRenderPass`. That interface records
draws into a pass the widget has already begun, whereas a screen-space pass owns
render passes of its own and must run *between* the scene pass and the composite —
it reads the resolved depth, which does not exist until the scene pass has ended.
Forcing it into the geometry-pass shape would mean pretending an off-screen chain is
a draw call.

Two things make screen-space work tractable across backends:

- **`RhiFrameContext::screen_from_view` / `view_from_screen`.** "Screen space" is
  defined as (u, v, depth) in the off-screen textures' own coordinates, all [0,1].
  Every backend difference — the clip-space correction, whether NDC z is [-1,1]
  (OpenGL) or [0,1], and whether texture row 0 is the framebuffer's bottom or top —
  is folded into these two host-built matrices, so a screen-space shader
  reconstructs a view position with no NDC knowledge at all. Getting any one of the
  three wrong produces plausible-looking but wrong occlusion, which is exactly the
  kind of error that survives review.
- **`uv = gl_FragCoord.xy * texel`, not the interpolated NDC-derived uv.**
  `gl_FragCoord`'s origin follows the render target's orientation and the sampled
  texture's row 0 follows the same convention, so this addresses the matching texel
  under both. An NDC-derived uv is vertically mirrored on one of the two backends —
  and because the pass would then read *and* write mirrored, it looks
  self-consistent while being wrong relative to the scene colour.

The occlusion multiply lands in LINEAR light **before** the tonemap. Darkening after
it would compress the shadowed range twice and read as flat grey. Note that
annotation pixels never receive AO, because the marker bypass hands them the raw
scene colour — matching GL.

Cost: at 1800x1280 the harness holds ~60 Hz with SSAO on (16.7 ms/frame),
indistinguishable from off (16.9 ms/frame). The harness is vsync-locked, so that
bounds the added cost inside the frame budget rather than measuring it in isolation.

### The composite chain, and why it touches every pass

`RhiPresentPass` applies exposure → tonemap (None/ACES/AgX/Khronos PBR Neutral) →
saturation → the single manual sRGB encode. The ordering is load-bearing: ACES
desaturates, so saturation follows it rather than pre-boosting colour into the
tonemap's shoulder.

This is what makes the renderer linear-light, and that is a **whole-renderer
invariant, not a property of one pass**: every geometry shader writes LINEAR colour
into the HDR target precisely because the encode happens here, exactly once. Pass
*APIs* still take display-referred sRGB colours — each fragment shader converts at
the last moment — so callers are unaffected. The background is likewise cleared
already-linearized and then bypasses grading, so the two conversions cancel and the
theme colour survives byte-exact.

Two bypasses decide which pixels get the filmic look:

- **Far-plane background**, via the resolved depth buffer. Grading the background
  would shift the theme colour.
- **Per-pixel annotation marker**, carried in the HDR target's ALPHA channel. This
  is the subtle one: in that target alpha is *not* opacity. Data pixels drive it to
  1 and take the filmic look; annotation geometry (TF triads, TF connection lines,
  pose gizmos) drives it to 0 and passes through ungraded, so magenta connection
  lines and axis colours stay vivid instead of being desaturated by AgX. The MSAA
  resolve averages the marker, which feathers the seam.

  How a pass sets it depends on whether it blends. Un-blended passes simply write
  the marker as their fragment alpha — which is why `RhiAxisPass`'s axis colours and
  `RhiTfConnectionsPass`'s magenta carry **alpha 0**, and why enabling blending on
  either would silently turn that marker into coverage. Blended passes instead
  choose alpha blend *factors*: `(One, OneMinusSrcAlpha)` raises the marker (data),
  `(Zero, OneMinusSrcAlpha)` drives it down by coverage (annotation).

**Known deviations from the GL renderer** (deliberate, revisit at parity):

- Gizmo shading is world-space Lambertian; GL shades in *view* space (a headlight,
  so nothing is ever fully dark). With the shared shader's 0.55 ambient floor
  nothing goes black either, so this is a look difference, not a legibility one.
- Line width is capped at 1 px — Metal has no wide-line primitive.
- `RhiVoxelGridPass` handles scalar (R32F) fields only; the direct-RGBA field and
  the per-cube edge outline are not ported.
- `RhiPosesPass` blends with depth-write on and no depth sort, matching the GL
  pass: translucent arms occlude each other in draw order. Acceptable for gizmos,
  and it avoids a per-frame sort over a particle cloud.
- `RhiMeshPass` omits shadow receive and the R8 "is-mesh" mask for EDL, because the
  passes that produce those inputs are themselves unported.
- The direct-to-widget fallback (used only when the HDR chain cannot be created at
  all) has **no composite**, so the linear-light output the passes write is never
  encoded and the scene renders dark. GL sidesteps this by not linearizing on its
  equivalent path. Acceptable while that path is a "beats a blank dock" last
  resort, but it is a real defect if it ever becomes reachable in practice.
- `RhiMarkerPass` does not draw `MarkerText`; neither does the GL pass, so this is
  not a regression. `MarkerLineBatch::thickness` is ignored, as in GL — Metal has no
  wide-line primitive and a core-profile GL context rejects `glLineWidth > 1`.
  Wireframe also does not apply to arrows/axes, where GL's `glPolygonMode` did
  affect them.

### RhiMeshPass — two structural departures from the GL pass

Both are forced by QRhi rather than chosen, and both are worth knowing before
touching the other passes:

- **Every material binds all five samplers.** A pipeline is compiled against its
  binding layout, so that layout must be final at pipeline-creation time and cannot
  vary per material. Absent maps therefore bind a 1×1 *neutral* texel rather than
  being switched off by a uniform — which also deletes four of GL's five
  `u_has_*_tex` flags, since white is the identity for all four multiplicative
  slots. The normal map is the one exception and keeps a flag: its neutral value is
  a no-op **only** when the tangent basis is well-formed, and a mesh with no UVs
  does not have one, so a flat placeholder plus a set flag would yield NaN normals.
  The flag follows what was *actually bound*, not what the material requested, so a
  map that fails to decode correctly degrades instead of corrupting shading.
- **Per-draw uniforms ride a dynamic-offset UBO.** GL re-set `u_model` per draw;
  QRhi has no loose uniforms, so one buffer holds every draw's block at an
  `ubufAlignment()`-aligned stride and each draw binds its own slot by offset. The
  consequence to remember: *growing that buffer invalidates every binding set that
  referenced it*, so capacity is settled from an upper bound **before** resolving
  the frame's draws, not after counting them.

Note also that `slots` is a Qt keyword macro (`qobjectdefs.h`) — naming a local
variable that produces a baffling "expected unqualified-id".

### RhiMarkerPass — three departures from the GL pass

- **Wireframe is real line geometry.** `glPolygonMode` has no QRhi *or* Metal
  equivalent, so every unit mesh carries a second LINES index buffer derived from
  its triangles, and triangle batches expand to segments on the CPU (they are
  re-streamed each frame anyway, so changing topology there is free).
- **Line and triangle batches merge into one draw each.** Their world placement is
  baked into the vertices during staging rather than passed as a per-batch matrix,
  which lets every batch share one buffer. GL issues a draw per batch.
- **Pipelines come from a small cache** keyed by what actually varies (topology,
  culling, depth write, blending, depth bias). QRhi bakes all of that into
  immutable pipeline objects, so the GL pass's ~15 `glEnable`/`glDisable`
  transitions would otherwise become that many named members.

**A uniform-block trap worth remembering.** The arrow and axes markers reuse the
shared `arrow.{vert,frag}` off the marker pass's own uniform buffer, so
`MarkerUbo` deliberately starts with the same two matrices as `ArrowUbo`. A shader
may declare a *smaller* block than the buffer holds, but not a larger one: while
`MarkerUbo` was 96 bytes the arrow shaders read `frame_world` from bytes 64-127,
got zeros, and multiplied every vertex by a zero matrix — so all arrows collapsed
to a degenerate point, with no validation error and no warning anywhere. Isolating
the pass (temporarily returning only it from `passes()`) is what found it, and is
the fastest tool for this class of bug.

**Two QRhi rules this port learned the hard way**, both silently accepted by Metal
and both producing convincing-but-wrong output rather than an error:

- A pipeline is compiled against the resource **layout** of the
  `QRhiShaderResourceBindings` it is created with. Bind placeholders so the layout
  is final at pipeline-creation time; swapping one texture for another at the same
  binding afterwards is fine.
- A pipeline's **sample count must equal its render target's**. A mismatch writes
  only a fraction of the samples, which reads as a uniformly washed-out,
  semi-transparent draw — not as an obvious failure.

Also note `clipSpaceCorrMatrix()` fixes NDC (Y direction *and* depth range) for
geometry, but says nothing about how a rendered texture is later **sampled**: the
present pass needs a separate flip driven by `QRhi::isYUpInFramebuffer()`.

A third such trap, found when the background bypass read all-zero depth: **QRhi can
only resolve depth out of a multisample depth TEXTURE**. `RhiHdrTarget` originally
attached depth as a `QRhiRenderBuffer`, and against that
`setDepthResolveTexture()` is accepted, reports no error, and silently never
writes — so `resolvedDepth()` handed out a texture that was uniformly 0. The chain
now attaches a multisample `D32F` texture (gated on `QRhi::MultisampleTexture`)
and keeps the renderbuffer only as a no-resolved-depth fallback. Note that
`isFeatureSupported(ResolveDepthStencil)` returning true says nothing about whether
your attachment can actually be resolved from.

## Rendering pipeline

Per frame, `SceneViewWidget::paintGL`:

```
layers + passes → SceneHdrFbo (multisample RGBA16F + DEPTH32F + R8 is-mesh mask)
                → resolve blit (single-sample color + depth + mask)
                → SsaoPass (R16F AO, box-blurred)   [reads resolved depth]
                → EdlPass  (R16F shade factor)      [reads resolved depth + mask]
                → composite/present (fullscreen triangle → backing FBO)
```

- **HDR chain (0A).** The render FBO is created at a fixed MSAA sample count
  (`kDefaultMsaaSamples`, clamped to `GL_MAX_*_TEXTURE_SAMPLES`) — INDEPENDENT of
  the QOpenGLWidget's negotiated `context()->format().samples()`, which is 0 once
  the view is composited inside an ADS dock (the backing store is single-sample).
  Because SceneHdrFbo owns its own multisample textures and the present is a
  fullscreen draw, the single-sample backing FBO never undoes the already-resolved
  anti-aliasing — so the scene is 4x MSAA docked, not just in the demo. (Seeding
  the chain from the context's samples was the bug that made MSAA silently vanish
  in the app.) MSAA→MSAA blits with mismatched formats are illegal, so the resolve
  target is always single-sample; the present pass draws into
  `defaultFramebufferObject()` (never FBO 0 — QOpenGLWidget renders off-screen).
  When the chain is unavailable the widget falls back to direct-to-backing
  rendering (one warning per context).
- **Linear light + tonemap (0B).** All color inputs are linearized (colormaps,
  occupancy LUT, vertex/base colors, theme colors; mesh diffuse textures upload
  as `GL_SRGB8_ALPHA8`). The composite applies exposure → tonemap
  (None/ACES/AgX/Neutral; AgX matrices are **column-major** GLSL `mat3` ctors;
  Neutral is the Khronos PBR Neutral mapper, which preserves colormap hue better
  than ACES) → saturation → manual sRGB encode.
- **Annotation alpha-marker.** The scene FBO's alpha is a per-pixel
  tonemap-bypass marker, not coverage: annotation passes (TF triads via
  `ArrowGizmo`, HUD overlay) write alpha 0 so they present flat and vivid,
  while data draws restore the marker with coverage-union blending
  `(GL_ONE, GL_ONE_MINUS_SRC_ALPHA)` on the alpha channel (preserving
  destination alpha instead would ghost occluded annotations through opaque
  meshes). Gizmo opacity rides the gizmo color's alpha through
  `glBlendFuncSeparate(SRC_ALPHA, ONE_MINUS_SRC_ALPHA, ZERO, ONE_MINUS_SRC_ALPHA)`.
- **Background bypass.** Far-plane pixels (depth == 1) keep the exact theme
  color, ungraded (no tonemap/saturation) — the background never shifts with the
  tonemap.
- **Is-mesh mask (EDL gating).** EDL is restricted to **mesh surfaces** — point
  clouds, grid, occupancy, axes and the empty background get no eye-dome contour.
  The scene FBO carries a second color attachment (`SceneHdrFbo::kMaskAttachment`,
  R8), cleared to 0 each frame; only `MeshRenderPass` enables that draw buffer (its
  fragment shader writes 1.0 to `location = 1`) — driven by `ViewParams::write_mesh_mask`,
  set on the off-screen + EDL-on path. The mask is resolved alongside color/depth
  and sampled by EDL. There is no separate object-class G-buffer: the alpha channel
  is the tonemap marker and is identical (1) for meshes and point clouds, so the
  mask is the only per-pixel mesh signal.
- **SSAO** reconstructs view position via `u_inv_proj` (works under the
  ortho camera; the perspective near/far formula does not) with an in-shader
  4×4-tiled hash as the rotation noise; it still applies scene-wide. **EDL** is
  Potree-derived (8 circular neighbours, log-depth response) but **mesh-only**: a
  mesh pixel is darkened when a neighbour is FARTHER — or is non-mesh, which the
  shader reads as "far" (`FAR_SENTINEL`) via the mask. So the mesh's silhouette
  against the void, point clouds, or farther meshes, plus the near side of its
  creases, get the contour; point clouds neither receive it nor cast it. Each
  per-neighbour log-depth gap is clamped to `look::kEdlMaxGap` so the huge
  silhouette gap reads as a graded outline rather than a solid black band; creases
  (much smaller gaps) are unaffected. With no mask bound EDL falls back to
  unrestricted (every pixel treated as mesh). Both passes multiply into the
  composite and degrade to no-ops when unavailable (`u_has_ao` / `u_has_edl`); EDL's
  darkening is floored (`CompositeParams::edl_floor`) so it bottoms out at a
  hue-preserving dark grey, `floor·color`, rather than pure black (floor 0 = the
  original multiply-to-black).
- **Defaults** (look-dev, 2026-06-13). The authoritative numbers live in
  [`scene_look_defaults.h`](../widgets/include/pj_scene3d_widgets/scene_look_defaults.h)
  — treat that header as the source of truth and this paragraph as a summary
  (it has drifted before). As of this writing: ACES, exposure 1.3, saturation
  1.3, SSAO on (strength 1, radius 0.4 m), EDL on (strength 1, radius 0.6 px,
  floor 0.3, max gap 0.02).
  Runtime knobs: `SceneViewWidget::compositeParams()`, `ssaoPass()`,
  `edlPass()`, and the per-view `meshShadingParams()` (roughness 0.6, f0 0.06,
  ambient 0.5, key/"sun" 1.6, fill 0.5, env-reflection 1.0, key-light dir
  azimuth 40° / elevation 55° ≈ high +X+Y — a stopgap for a future per-scene
  lighting object). Shader provenance/licenses: [`../THIRDPARTY.md`](../THIRDPARTY.md).
- **No shadows (yet).** Mesh shading has no shadow term: the fixed key light, the
  camera-locked fill, and the IBL ambient are all unoccluded. Adding mesh shadows
  would be a geometry **depth pre-pass** rendered from the light *before*
  `renderScene` — structurally unlike the screen-space SSAO/EDL **post-passes** —
  with the shadow factor multiplied into the key-light term only (the first summand
  of `direct` in `mesh_render_pass.cpp`). Don't mistake the SSAO/EDL passes as the
  template for shadows.

**GL context lifecycle (don't regress).** `QOpenGLWidget` recreates its context
on every ADS reparent. Every pass, layer, the HDR chain, and the present
program implement `releaseGL()` (wired to the dying context's
`aboutToBeDestroyed`) and rebuild lazily — VAOs/FBOs/textures are per-context,
never shared. The app deliberately does NOT set `AA_ShareOpenGLContexts`.

**TF frame hover labels.** Hovering a TF axis triad shows the frame's name in a
small HUD box and draws that triad brighter. The pick is pure screen-space and
lives in `SceneViewWidget`: `paintGL` caches `proj*view`, and a button-free
`mouseMoveEvent` projects every resolvable frame origin through the GL-free,
unit-tested `core/tf/frame_picking.h` (`projectFrameOrigin` + `pickNearestFrame`)
and takes the one closest to the cursor within ~20 logical px (first on an exact
tie). The label is drawn at the tail of `paintGL` as a 2D overlay — the same path
as the perf HUD — re-projecting the live origin so it stays glued to the frame as
the scene streams. Its text and panel are **CPU-rasterized into a `QImage` and
blitted with `drawImage`** (`hud_overlay.h::renderHudPanel`), *not* painted as
`QPainter` text on the GL widget: the latter relies on Qt's per-GL-context glyph
atlas, which does not survive the context recreation ADS triggers on dock reparent
/ **layout restore**, leaving glyphs doubled/garbled while vector fills stayed
correct (the original #214 bug). `drawImage` is a plain textured quad — no glyph
atlas, immune to recreation and DPR changes. The highlight is the one place this touches
the draw pass: `paintGL` pushes the hovered frame to `AxisRenderPass::setHighlightedFrame`,
which luminance-boosts that frame's triad colors. Gated on the triads being
visible; cleared on a camera gesture and on leave. Occlusion is ignored for now
(a frame hidden behind geometry still labels); a one-texel depth-reject is the
planned refinement.

## Mesh shadows

Real-time directional shadows for **meshes only**, from the existing fixed key/"sun"
light (`MeshShadingParams::key_light_dir`). **Casters: meshes only** — URDF/robot
meshes (`RobotModelLayer` visual links) and scene-entity `ModelPrimitive` meshes
(`SceneEntitiesLayer`); point clouds, voxel/occupancy grids, axes, TF triads,
markers, and collision hulls never cast. **Receivers: meshes and the solid grid
floor** (`GridRenderPass` filled-cell mode). Off by default, per-dock
(`MeshShadingParams::shadows_enabled`).

Unlike the screen-space SSAO/EDL post-passes, a shadow map is a **geometry depth
pre-pass** that runs in `paintGL` *before* `renderScene` (between the `FrameContext`
build and the scene render):

```
fit light frustum to mesh-caster bounds  ──►  ShadowMapPass.begin() (depth FBO)
  ──►  each mesh layer renderShadowCasters() (depth-only, from the light)
  ──►  end + rebind scene FBO  ──►  renderScene (receivers PCF-sample the map)
```

- **GL-free core** (`core/shadow_camera.h`, headless-tested). `fitDirectionalShadowCamera`
  builds the orthographic world→light-clip matrix: bounding-**sphere** extents
  (rotation-invariant, no resolution pulsing), padded, and **texel-snapped** along a
  light-only basis so the shadow edge does not crawl as the scene jitters sub-texel.
  `extendAabbToGroundShadow` grows the caster AABB to enclose where the shadow lands
  on `z=0`, so the frustum also covers the **receiving floor** (the floor never casts,
  so it is otherwise absent from caster bounds — and the shadow would fall outside a
  caster-only frustum). `kShadowMapSize` (2048) sizes the map independently of
  `render_scale`.
- **Caster bounds, distinct from `worldBounds()`.** `Scene3DLayer::meshShadowBounds`
  is a separate hook from `worldBounds()` on purpose: `RobotModelLayer` stays
  bounds-less for the **camera** (a moving robot must not yank the view), but the
  shadow frustum *must* enclose the robot mesh. `meshShadowBounds` reports the visual
  caster AABB via `MeshRenderPass::worldBoundsOfDraws` (each resource caches its local
  AABB; lifted by the draw model matrix).
- **Casting.** `Scene3DLayer::renderShadowCasters` (default no-op; overridden by the
  two mesh layers) forwards the visual draws to `MeshRenderPass::renderDepthOnly` — a
  depth-only program (position → light clip, empty fragment) reusing each mesh's
  existing VAO. Collision hulls are not casters (they would double-darken the
  silhouette).
- **Receiving.** The mesh fragment shader and the grid filled-cell shader each project
  the world position into light space and do manual **3×3 PCF** over a plain
  `sampler2D` (the `gl::Texture` wrapper has no compare-mode; the shader guards
  out-of-frustum/beyond-far UVs as *lit*, so the CLAMP_TO_EDGE border can't smear). The
  shadow factor multiplies **only the key-light term** — the camera-locked fill and the
  IBL ambient stay lit, so shadowed surfaces read as shaded, not black. The floor
  darkens toward a floor value (not pure black) so the grid stays legible. A caster-side
  `glPolygonOffset` plus a receiver-side world **normal offset** (sized in shadow
  texels) control acne / peter-panning.
- **Lifecycle / per-dock.** `ShadowMapPass` (depth `GL_DEPTH_COMPONENT32F` FBO) is a
  per-`SceneViewWidget` member released in `releaseGlResources()` and rebuilt lazily —
  the same per-context contract as every pass. Each dock reads its own
  `key_light_dir`, so two docks shadow independently. `shadow_map_id == 0` (feature off,
  or an invalid frustum fit) makes every receiver render fully lit — a safe no-op
  degrade.
- **Gated on live data.** The whole pre-pass is skipped unless a `TransformBuffer` is
  bound (`tf_ != nullptr`), mirroring the layer color pass (`if (tf_)`): layers only
  pose geometry through TF, and a layer keeps its last draw cache after its dataset is
  deleted (the dock clears the binding via `setTransformBuffer(nullptr)` but does not
  dirty surviving layers), so an ungated pre-pass would keep depth-drawing that stale
  geometry and the floor would keep sampling a shadow whose mesh has vanished. Deleting
  the data therefore clears the floor shadow. Regression: `shadow_persistence_gl_test`.
- **Demo / verification.** `scene3d_mesh_viewer --shadows on|off` (switches the floor
  to filled cells) renders A/B screenshots; `fixtures/shadow_demo.urdf` is an elevated
  box over the ground. Cost on Iris Xe ≈ 4 ms/frame for a single-caster scene
  (well under the 16.7 ms / 60 Hz budget).

## Camera system

The camera is an interchangeable controller over a shared, serializable pose,
deliberately in `pj_scene3d_core` (Qt/GL-free) so the math is headless unit-
testable (`camera_near_far_test`, `camera_zoom_to_cursor_test`,
`camera_state_transfer_test`, `camera_follow_test`).

- **`ICamera` + `CameraState`** (`core/include/pj_scene3d_core/camera/camera.h`):
  `CameraState` (focal, radius, azimuth, elevation, fov_y, ortho_scale,
  perspective) is the pose every model exports via `state()` and adopts via
  `adoptState()`, so switching models preserves where you were looking.
  `SceneViewWidget` holds a `std::unique_ptr<ICamera>`; `setCameraModel` does
  capture → construct → `adoptState` → swap.
- **Four models, selectable.** `SceneViewWidget::CameraModel` enumerates
  `{ Orbit, XYOrbit, Fly, TopDownOrtho }` — that order is load-bearing because
  the overlay combo's index casts directly to it. Inheritance: `OrbitCamera`
  (perspective spherical orbit, Z-up; non-`final`), `XYOrbitCamera : OrbitCamera`
  (orbit pivot locked to the z=0 ground plane), `TopDownOrthoCamera : ICamera`
  (orthographic bird's-eye, rotatable about +Z — the robotics "2D mode" for
  costmaps/grids), `FlyCamera : ICamera` (first-person free eye + yaw/pitch, no
  orbit pivot: LEFT looks around, pan strafes, wheel dollies forward).
- **Adaptive near/far** (`core/src/camera/camera_math.cpp`, `adaptiveNearFar`):
  `near = max(d·1e-2, 1e-3)` from the working distance only (so close inspection
  never clips), `far = max(d·4, scene_reach)·1.5` (reaches the whole scene), then
  `near = max(near, far/1e5)` — a ratio cap that bounds depth precision.
  `scene_reach` is the scene AABB diagonal plus the focal-to-center distance.
  `TopDownOrthoCamera` uses its own flat-scene guard instead (near/far measured
  against the eye height above the focal plus the scene's vertical extent), so a
  zoomed-in costmap never clips the ground.
- **Zoom-to-cursor** (wheel): a homothety about the world point under the cursor —
  the eye and focal scale toward the cursor target by `pow(0.9, ticks)`, so the
  hovered point stays pixel-locked. A homothety is a uniform scaling, so it cannot
  rotate the orbit: `OrbitCamera::zoomToCursor` keeps azimuth/elevation **untouched**,
  scales the radius (clamped to `[lo, hi]`), and slides the pivot along the cursor
  lever by the *effective post-clamp* factor `new_radius / radius` (so the lock holds
  even when the radius clamps). It deliberately does **not** re-derive the angles from
  `eye − focal` — at large world coordinates that subtraction of two ~1e6 vectors
  loses its low bits to float32 cancellation and spuriously spins the view a fraction
  of a degree on every tick (~1.9°/tick at 5e6). `XYOrbitCamera` re-locks its pivot to
  z=0 the same cancellation-free way (re-pivot along the fixed view ray, never
  `setEyeFocal`). A degenerate ray (no ground/focal-plane hit) falls back to
  center-of-view `zoom()`; right-drag stays center-of-view zoom; on `FlyCamera`
  zoom-to-cursor degenerates to a forward dolly by design.
- **Frame follow (Position-only).** `ICamera::followShift(world_delta)` shifts the
  camera's anchor by a world delta without touching orbit angle / zoom — Orbit and
  TopDownOrtho move the focal, Fly the eye, and `XYOrbitCamera` overrides it to drop
  the z component (ground-locked). `SceneViewWidget` holds the follow target
  (`setFollowFrame`, "" = off) and applies it each tracker tick in `applyFollow`:
  look up the target's origin in the fixed frame at `render_time_`, and shift the
  camera by the delta against the previous tick's origin (`follow_prev_origin_`).
  The first tick after enabling / a fixed-frame change / a lookup gap only *seeds*
  the baseline (no shift), so enabling never jumps and the user's orbit/zoom/pan is
  preserved (follow only *adds* the target's motion). `followRenderKey(time)` hashes
  the followed origin into the dock's `viewRenderKey` so a moving target isn't
  coalesced away by the repaint gate even when the TF overlay is hidden. The UI is
  the "Camera" section's **Follow frame** picker in `Scene3DConfigPanel`, with a
  trailing **recenter** button (`SceneViewWidget::recenterOnFollowFrame` →
  `followShift(target − current focal)`) that snaps the camera onto the target on
  demand. Heading / Pose (rotation-following) modes are future work; the
  `FollowMode` enum already reserves the slot.
- **Large-coordinate precision (camera-relative rendering).** Following a frame in a
  UTM/GPS-scale map parks the camera at ~1e5–1e7 m, where float32 keeps only ~0.1–0.6 m
  of resolution and `view × world-geometry` (subtracting the ~1e6 eye from a ~1e6
  vertex) collapses to cancellation noise — the whole scene visibly *swims* on zoom.
  The scene therefore renders **camera-relative**: each `paintGL` picks a `render_origin`
  (the camera focal, in double), builds the view via
  `ICamera::viewMatrixRelativeTo(render_origin)` (eye/focal offset by the origin in
  double; bit-exact `viewMatrix()` at origin 0, pinned by a test), and threads the origin
  through `FrameContext::render_origin`. `FrameContext::lookup()` subtracts it from every
  resolved transform **in double, before the float downcast** (`toRenderSpace`, in
  `camera_math.h`) — so all TF-placed geometry (every pass/layer), the async hover
  hit-test, and the eye fed to mesh lighting land near the origin and stay precise. The
  camera *state* and *scene bounds* stay in absolute world (follow and near/far need them
  there); only the per-frame render path goes relative. Three passes that don't resolve a
  TF frame take the origin explicitly: the grid (model `= translate(−origin)`; it lives at
  the world origin), TF-connection lines (`buildTfConnectionSegments` takes it), and
  point-cloud **colour-by-axis** — position renders relative for precision, but the colour
  scalar adds the origin back (`u_color_axis_offset`) so a point's colour stays in the
  fixed frame, independent of the camera (the colormap range is lifted by the same offset,
  and algebraically cancels in the shader's normalize). `render_origin == {0,0,0}` (the
  default for hand-built `FrameContext`s in tests/demos) reproduces absolute-world
  rendering verbatim.
- **Frame-list freshness.** The frame pickers (fixed-frame + follow) are fed by
  `SceneViewWidget::refreshAvailableFrames` → `getFrameHierarchy()`, which is
  time-independent (whole buffer). That refresh used to fire only from
  `setTransformBuffer` / `setTrackerTime`, so TF folded into the buffer while the
  playhead was paused (a file load) left the combos stale until the user pressed
  play. `Scene3DDockWidget` now also re-enumerates on `datasetTransformsReady`
  (file ingest), decoupled from the time-gated repaint path, so the full tree is
  listed as soon as it loads.
- **Scene bounds.** `Scene3DLayer::worldBounds()` returns an optional source-frame
  `AABB`; `PointCloudLayer`, `OccupancyGridLayer`, `DepthCloudLayer`, and
  `VoxelGridLayer` override it. `RobotModelLayer` deliberately does **not** (it is
  bounds-less by design — a robot is posed by the live TF tree the camera already
  frames through the other store-backed layers, so adding its links would pull the
  camera around as joints move). `Scene3DDockWidget::updateSceneBounds` unions
  **every** layer's box (`unionAABB` over all layers that report one — there is no
  per-layer visibility filter) and pushes the result to
  `SceneViewWidget::setSceneBounds` → the active camera, feeding the adaptive
  near/far above. No reporting layer → invalid AABB → working-distance fallback.
  (Note: because `RobotModelLayer` reports no bounds, the scene AABB excludes the
  robot mesh — relevant to any future light-frustum fit for shadows.)
- **Overlay UI.** `Scene3DDockWidget` overlays a `camera_model_combo_`
  (`{Orbit, XYOrbit, Fly, Top-down ortho}`) and a `home_button_` (Home icon,
  resets the active model to its default view — not fit-to-scene), anchored flush
  to the top-right edge. The orientation gizmo (`AxisOverlayPass`) sits in the
  bottom-right corner (set in the `SceneViewWidget` ctor) so it no longer crowds
  these controls. They are children of the dock, not the `QOpenGLWidget`, and
  `raise()`'d above it (ADS native-window z-order).
- **Persistence.** `xmlSaveState` writes the active model as a stable string id
  (`orbit` / `xy_orbit` / `fly` / `top_down_ortho` — independent of the enum
  integer / combo order) plus the `CameraState` as JSON (`cameraStateToJson`) and
  the `follow_frame` attribute (empty = off); `xmlLoadState` restores all three,
  sanitizing the state so a corrupt layout can never drive a degenerate view. A
  restored follow target absent from the current TF tree stays inert until it
  appears (`applyFollow` holds on lookup failure).

## Pose-array layer (`PosesInFrame`)

`PosesInFrameLayer` renders a `PJ.PosesInFrame` topic (`geometry_msgs/PoseArray` /
`foxglove.PosesInFrame` equivalent — a flat list of poses in **one** `frame_id`,
not a TF tree) as a per-pose coordinate-axis **triad** gizmo, with per-layer params:
**arrow length** (m), **opacity**, an **X-arrow-only** geometry mode (draw a single
X arrow per pose instead of the triad — useful for dense pose arrays where full
triads clutter), and an orthogonal **color override** (recolor every arm with one
shared color). Geometry and coloring are independent: you can recolor a full triad
or keep a single X arm in its natural red. Defaults (0.15 m, 1.0, off, off) and the
desaturated X/Y/Z colors match the TF "Frames" gizmos.

- **Pure expansion (core, GL-free, unit-tested).** `buildPoseTriadInstances(msg,
  PoseTriadStyle{axis_length, opacity, x_arrow_only, override_color, color})`
  (`core/poses_in_frame_render.{h,cpp}`) turns each pose into three
  `PoseTriadInstance{mat4 model, vec4 color}` arms — `poseToMat4(pose) · arm-rotation ·
  scale(size)`, frame-LOCAL, with `opacity` in the color alpha. The arm rotations
  (+X→+Y / +X→+Z) and colors mirror `renderTriadBound` / `AxisRenderPass`. The style
  struct keeps geometry (`x_arrow_only` → 1 arm vs 3) separate from coloring
  (`override_color` → every produced arm takes the shared `color`, else its natural
  per-axis R/G/B), so an un-overridden X-only arm is still red.
- **GPU-instanced draw.** `PosesRenderPass` (`passes/poses_render_pass.{h,cpp}`) owns
  one unit-arrow mesh (shared with `ArrowGizmo` via `gizmos/arrow_mesh.{h,cpp}`) plus
  an instance VBO (per-instance `mat4 model` at locs 2–5 + `vec4 color` at loc 6,
  divisor 1 — the `MarkerRenderPass` solid-instancing layout). **Key efficiency
  choice:** the per-instance model is frame-LOCAL and the fixed-frame TF transform is
  a `u_frame_world` uniform, so the instance buffer re-uploads only when the sample /
  size / opacity changes — TF and camera motion cost nothing. All `3·N` arms draw in
  one `glDrawElementsInstanced`, so a thousands-of-poses AMCL particle cloud stays one
  draw call. Lit shading + annotation blend (bracketed like the TF axes) keep the look
  identical to the frame gizmos.
- **Ingest.** Mirrors `OccupancyGridLayer`: `attach` bootstraps the source frame;
  `setTrackerTime` defers to `render()`, which decodes `store.latestAt(t)` via a
  **per-use** `parseLocked` binding (never cached — the reload-UAF rule), expands, and
  stages into the pass. A `SequentialUID`-keyed coalescing guard skips redundant
  decodes while scrubbing within one message; a `style_revision_` bump forces the
  current sample to re-expand after any style edit. `render` resolves
  `frame_ctx.lookup(frame_id)` once — an unresolvable frame draws nothing (orphan).
- **Params persistence.** `xmlSaveState`/`xmlLoadState` round-trip `<poses_in_frame
  gizmo_size gizmo_opacity x_arrow_only override_color override_color_value>`, which
  also makes the params copy/paste/apply-to-family-able through `pj_scene_common`'s
  `serializeLayerParams`/`applyLayerParams`. The config widget's "Override color"
  checkbox + always-visible "Color:" swatch reuse `SceneEntitiesLayer`'s marker-recolor
  text and layout (picking a color auto-ticks the box) for cross-layer consistency.
  No host edits.

## Pointcloud layer (`PointCloudLayer`)

`PointCloudLayer` keeps the existing `convertCanonical()` -> `DecodedPointCloud` ->
`CloudVertex` fallback for layouts that need CPU decoding, and adds a narrow zero-copy
fast path for little-endian clouds whose xyz fields are contiguous float32 values. The
fast path uploads the canonical `sdk::PointCloud::data` buffer verbatim and binds the
shader attributes with the point's native stride/offset/type; `FastCloudData` retains the
wire cloud by value so its `BufferAnchor` keeps the bytes alive across GL-context
recreation, letting `releaseGL()`/`initializeGL()` re-upload from the same source just as
the fallback re-uploads from retained decoded vectors.

RGB-direct (`kRgb`) clouds also take the fast path when the cloud carries a packed,
contiguous `rgba`/`rgb` uint32 field (`checkFastPath(..., want_rgba=true)`): the four colour
bytes upload verbatim and bind as a normalized `vec4` straight at the field offset —
pixel-identical to the CPU `convertCanonical(extract_rgba)` path, with no extraction.
Scattered separate `red`/`green`/`blue` channels fall back to the CPU packer.

The geometry AABB (`world_bounds_`) feeds the camera scene-fit every frame, so it must refresh
on every sample. On the bounds-only cases (RGB-direct, solid, spatial-axis, auto-off, non-dirty —
i.e. when no colormap scalar pass is needed) with a 4-byte-aligned fast-path layout, that
reduction runs on the **GPU** rather than the CPU: `PointcloudAabbReducer` dispatches a compute
shader over the already-resident fast-path VBO (no extra upload), reducing min/max via an
order-preserving float→uint key (`aabb_gpu_key.h`) and `atomicMin`/`atomicMax`, and reads the
6-value result back **asynchronously** (fence + non-blocking `poll()` in the next frame's
`render()`). The completed AABB flows to the layer through a bounds callback (`onGpuAabb`) which
updates `world_bounds_` and requests a repaint, so the existing `repaintRequested →
updateSceneBounds` path re-fits the camera. The result lags a frame or two — invisible to the
auto-fit — so the per-sample bounds cost leaves the GUI thread entirely (≈80–170× less GUI-thread
work than the CPU scan at 1–5 M points; see `demos/pointcloud_aabb_benchmark`).

Fallbacks keep the CPU scan: the **first** sample after attach/reset (to seed `world_bounds_`
synchronously and probe compute support), the kField auto-range-dirty case (it needs the scalar
min/max anyway, getting bounds for free in the same pass), a misaligned layout, the decoded
`convertCanonical` path (bounds come free there), and any context without compute (GL < 4.3,
e.g. Windows software GL). The layer only drops the CPU scan once the pass confirms
`gpuAabbAvailable()`, so unsupported drivers degrade safely.

## Depth-cloud layer (`DepthCloudLayer`)

Back-projects a depth image into a 3D point cloud (one point per valid pixel),
colored by depth. Sibling of the 2D depth view — same data, different geometry.

- **No `kDepthImage` producer.** Depth arrives as an `sdk::Image` with a depth
  `encoding`; the parser can't tell depth from color at schema-classification
  time. So `DepthCloudLayer` is registered on `kImage` and the dock **gates** a
  topic by peeking the first sample's `encoding`
  (`isDepthEncoding`: `16UC1` / `32FC1` / `compressedDepth`) — color images are
  refused. Because the two share `kImage`, an empty-placeholder image drop opens
  the **2D** viewer (host policy in `pj_app`); a depth image reaches a 3D dock by
  being dropped onto an existing one or via the 3D family switch.
- **Decode (`toDepthView`).** `16UC1`/`32FC1` alias the image bytes (zero-copy).
  `compressedDepth` is PNG-decoded via `QImage` to a `16UC1` (millimetre) view —
  including a **bare-PNG signature repair**: RealSense bags carry a headerless PNG
  that begins at the `IHDR` chunk, so the 8-byte signature + IHDR length are
  restored before decode (mirrors the 2D path's `toDepthImage`; the matching
  parser-side ConfigHeader handling is `parser_ros`). 32FC1 inverse-quantized
  compressedDepth is not yet handled.
- **Intrinsics by `frame_id`.** A `CameraInfo` is joined to the image by
  **`frame_id`** (authoritative, never the topic name — matches Foxglove's rule /
  Rerun's camera hierarchy). An exact match wins; with none, `resolveIntrinsics`
  falls back to a lone `CameraInfo` only when unambiguous, else refuses rather
  than pair the wrong camera. The result is memoized by `(frame_id, CameraInfo
  SampleId)` and revalidated parse-free via `latestAt`.
- **Back-projection.** The math is the Qt-free core `depth_backproject.{h,cpp}`
  (`depthToPoints`); a single pass yields positions, the per-point depth scalar,
  the world AABB, and the colormap range together (via the optional
  `scalar_out` / `bounds_out` / `scalar_range_out` out-params).
- **Render + color.** The POC feeds the points through the existing
  `PointcloudRenderPass` (a dedicated GPU attributeless / `texelFetch` pass is a
  planned follow-up). Depth is colored through the **shared** `PJ::Colormap`
  (turbo / viridis / plasma / grayscale, `pj_widgets/Colormap.h`) — the same
  source of truth as the 2D depth view and the pointcloud field coloring.
- **Config.** Per-layer colormap, point size, and a min/max depth range.

## Voxel-grid layer (`VoxelGridLayer`)

Renders a dense `sdk::VoxelGrid` (SDK ≥ 0.10.0) — a dense 3D lattice whose
per-voxel value is generic via `fields` (occupancy / cost / ESDF / semantic, or a
direct RGBA channel) — as GPU-instanced cubes. Sibling of the pointcloud layer for
volumetric data.

- **Dense→cubes expansion is entirely GPU-side.** `VoxelGridRenderPass` uploads
  the selected field as a **3D texture** and issues **one** `glDrawElementsInstanced`
  over a unit cube (`column*row*slice` instances). The vertex shader derives each
  voxel from `gl_InstanceID`, `texelFetch`es its value, evaluates the viewer-side
  draw predicate (which the schema does **not** encode), and degenerate-clips culled
  voxels. So the **CPU / draw-call** cost is independent of voxel count (one draw),
  and a re-scrub to a cached grid re-uploads nothing — though the GPU vertex shader
  still runs once per voxel.
- **Qt-free core.** The coordinate/value math (`core/voxel_grid_view.{h,cpp}`,
  `core/voxel_grid_value.{h,cpp}`) is headless unit-tested.
- **Follow-up.** A GPU compute-shader compaction path (`glDrawElementsIndirect` over
  only the accepted voxels, GL ≥ 4.3) is a documented follow-up for very large dense
  grids; not built.

## Repaint coalescing & autoplay

- **Per-tick repaint coalescing.** During playback the scene must not free-run at
  60 Hz redrawing identical frames. `ISceneLayer::renderKey` produces a decode-free
  per-layer fingerprint (sample stamp ⊕ transform), and `SceneDockWidget::onTrackerTime`
  skips the repaint when nothing a layer would draw has changed — holding the
  ≤ 60 Hz / idle→~0 CPU budget the performance goals require (PR #259). `renderKey`
  must derive from index/entry timestamps, **never** from a `latestAt` that would
  decompress cold chunks inside the gate.
- **`--autoplay`.** The app's `--autoplay` CLI flag starts hands-free looped
  playback (PR #260); it doubles as the `PJ_AUTOPLAY` profiling hook used to measure
  the repaint/render cost above.

## URDF / robot-model subsystem

- **Parser** (`widgets/src/urdf_parser.{h,cpp}`): `QDomDocument`-based;
  reads `<link>` visuals/collisions (origin, geometry, material). Geometry is
  the `GeomShape` variant — box/cylinder/sphere primitives **and** meshes.
  `<joint>` is parsed into `RobotModel::joints` (name/type/parent/child/origin),
  but **TF still owns kinematics**: link poses come from the live TF tree. xacro is
  detected (element prefix or filename) and rejected with an actionable error,
  never a cryptic XML failure.
- **Fixed-joint TF bridges** (`core/robot_model_bridges.{h,cpp}`): a URDF may
  contain a frame the data's `/tf` never publishes — classically a gripper rigidly
  mounted to an arm flange, where the mount transform lives only in the URDF (the
  DROID arm-droid dataset is exactly this). `fixedJointStaticTransforms()` turns
  each **fixed** joint into a static (`/tf_static`-style, epoch-stamped)
  `StampedTransform`; `RobotModelLayer` caches them (frame-prefixed) and
  `ensureStaticBridges()` re-asserts them into the dataset `TransformBuffer` every
  tracker tick via `injectMissingStaticTransforms()`. The inject is **guarded**
  (`getParent(child)==nullopt` — never overrides a frame the data publishes),
  **idempotent**, and **self-healing** (a same-file reload that clears the buffer
  re-bridges on the next tick). Once a gripper base is bridged, the movable
  knuckle frames the data *does* publish under it cascade into resolvability on
  their own. Movable-joint articulation (a `JointState` source, `<mimic>`,
  default-to-0) is deferred.
- **Sources.** A `RobotModelLayer` reads its URDF from a store topic
  (`kRobotDescription`, decoded via the topic's parser — payload bytes are
  CDR-framed, never cast to a string), a local file, or an http(s) URL.
  File/URL layers are dock-local: synthetic registry ids that never touch the
  ObjectStore (`Scene3DDockWidget::addRobotModelLayer{,FromUrl}`).
- **Mesh loading** is async (`QtConcurrent::run` → futures polled in
  `render()`); the GL thread never blocks. assimp covers STL/DAE/OBJ/glTF.
  `MeshData` carries UV0 + tangents (`aiProcess_CalcTangentSpace`) and a
  per-`SubMesh` `Material` (glTF 2.0 metallic-roughness, read via assimp's
  material abstraction). Each map (`TextureSource`) is an external file path OR
  inline bytes for embedded glTF/GLB `*N` images (still PNG/JPEG-encoded), keyed
  by content hash; `MeshRenderPass` decodes both, uploads color/emissive sRGB and
  data maps (metallic-roughness/normal/occlusion) linear, and caches per-context
  by key plus texture color space. Sources without PBR factors fall back to
  `MeshShadingParams`.
- **Rendering**: `MeshRenderPass` (metallic-roughness GGX + normal mapping +
  occlusion + emissive + **analytic image-based ambient** — diffuse irradiance
  plus a split-sum specular reflection of a procedural ground→sky environment
  (Karis `envBRDFApprox`, no HDRI cubemap) — lit by a **fixed world key/"sun"
  light** and a dimmer **camera-locked fill headlight**. Metals now reflect the
  sky/ground gradient instead of reading near-black; a true prefiltered-cube IBL
  from an HDRI is still future work) draws meshes by
  key and primitives from unit
  box/cylinder/sphere geometry (URDF cylinder is Z-aligned; sizes ride the
  model matrix). Unresolved meshes render a **magenta unit cube** —
  intentionally ugly, impossible to mistake for data. Visual meshes are split
  into opaque and best-effort translucent draws (layer opacity, override alpha,
  or glTF `BLEND`; no depth sort). Collision geometry with no `<material>` gets
  an **orange tint** (RViz convention) so hulls read as distinct from the visual
  meshes; it renders as a **translucent overlay** (blend on, depth-writes off, so
  the real robot shows through) below full opacity, and as a **solid, occluding**
  hull at opacity ≈ 1.0 (the opaque path: blend off, depth-writes on). Scene-wide
  opacity/visibility comes from `meshShadingParams()`. URDF draw calls are cached
  in camera-relative render space (after `FrameContext::lookup` subtracts the
  current render origin), so `RobotModelLayer` reuses them across pure
  view/projection changes but rebuilds when pan / zoom-to-cursor / follow changes
  the render origin.
- **Time contract**: `RobotModelLayer::timeRange()` returns the inverted
  sentinel `{Timepoint::max(), Timepoint::min()}` — a static decoration must
  neither widen the playback timeline (`{0, INT64_MAX}` would balloon it to
  2262) nor clamp the playhead to its latch timestamp. The dock skips
  inverted ranges and still delivers live tracker time.

## Streaming / live-data path

TF, pointclouds, and markers ingest live as well as from a file.
`TransformService::ingestFrameTransformsForDataset` reads every *new*
`FrameTransforms` message in the dataset into the core `TransformBuffer`
(cursor-based, so repeated calls only fold in what arrived since the last one)
and emits `datasetTransformsReady`. During a **progressive file load** the loader
calls it on every flush, so the buffer fills as the file streams in; the dock
connects `datasetTransformsReady` (in `setTransformService`) and re-renders at the
current playhead via `onTrackerTime`, so a restored 3D scene populates *as the file
loads* instead of only at completion. A non-progressive or already-finished load
runs the same call once at the end. Streaming has no loader to drive that call, so
the dock wires its own incremental path off `samplesIngested`:

- `Scene3DDockWidget::setSessionManager` shadows the base to also call
  `reconnectLiveSamples`, which connects `SessionManager::samplesIngested`
  (fired on the UI thread after each retention trim, `live == true` only while
  following a live stream; file load emits `live == false` and is served by the
  loader-driven `ingestFrameTransformsForDataset` + `datasetTransformsReady` path
  above instead).
- On each live tick, `TransformService::ingestNewTransforms` advances a
  per-topic store cursor and folds only the new `FrameTransforms` into the
  buffer — cheap, and a no-op when nothing new arrived. Without it the TF buffer
  stays empty and every sensor frame is orphaned (red).
- `driveVisibleLayersToLiveEdge` then queries each visible layer's `timeRange()`
  for the newest timestamp now in the `ObjectStore` and drives `setTrackerTime`
  on every visible layer (consulting the *layer's* range, not the store's per
  base-topic range, so a multi-topic layer like OccupancyGrid + its `_updates`
  sibling does not freeze at its last keyframe). Without this, object layers
  would stall while only the TF buffer advanced.

`TransformService` is a `widgets/`-level wrapper over the Qt/GL-free core
`TransformBuffer`. It owns no parser handle: each ingest resolves the topic's
binding through `SessionManager::parserBindingForObjectTopic` and decodes under
`parseLocked` (parse-locked per use — never a cached binding). Streaming uses a
finite `TransformBuffer` cache window (default 10 s) so a growing live stream
trims old samples and stays memory-bounded; the file path constructs the
buffer with eviction disabled, since the whole recording is retained (fed in
incrementally during a progressive load, or in one pass for a finished load).

### Object decode: parser-decoded vs canonical blob

Every object consumer (the layers above + `TransformService`) decodes a store
entry through one seam, `pj_scene3d_widgets/resolve_object.h::resolveObject`,
which picks the path by whether the topic has a bound `MessageParser`:

- **Parser bound** — file / streaming data sources whose objects arrive as
  parser-decoded messages (e.g. `parser_ros`, `parser_protobuf`): decode via the
  topic's parser under `parseLocked` (per-use binding, never cached — the
  reload-UAF rule).
- **No parser bound** — a data-source / toolbox that pushed an *already
  serialized canonical* object (e.g. the Mosaico cloud toolbox, which packages
  the server's `point_cloud2` / `pose` / `motion_state` ontologies into
  `sdk::PointCloud` / `PosesInFrame`): the bytes are a pj_base
  wire blob, so `resolveObject` deserializes them with the canonical codec
  selected by the topic's `builtin_object_type`. This mirrors pj_scene2D's
  `canonical = (binding.parser == nullptr)` discipline.

Either way the **host** performs the decode — the producing plugin never does
(the "canonical-objects-in" boundary; the canonical codec is the canonical
object's own serialization, not a transport/CDR wire format). A layer's
`attach()` therefore accepts a topic that has *either* a parser *or* a canonical
codec for its `builtin_object_type` (`hasCanonical3DCodec`); it rejects only a
topic that has neither.

### SceneEntities lifetime expiry & the decoded-batch cache

`SceneEntitiesLayer` replays every batch up to the tracker time into an id-keyed
entity map (replace-by-id, deletions, lifetime expiry). Two mechanisms support this
on the live path:

- **Dual-clock lifetime anchor.** A `SceneEntity` carries an embedded `timestamp`
  (sensor epoch) and an optional `lifetime_ns`. Under streaming the ObjectStore entry
  is *host*-stamped (tracker clock), a different epoch from the sensor timestamp.
  Expiry must therefore compare against the ingest timestamp, not `entity.timestamp` —
  otherwise every finite-lifetime entity expires the instant it is folded (the bug that
  hid the streamed car mesh). The layer keeps a parallel map `entity_expiry_anchor_ns_`
  (keyed identically to `entities_`) holding each entity's ingest timestamp;
  `expiredAt()` tests `anchor + lifetime < tracker` (overflow-safe; `lifetime == 0` ⇒
  never). The single erase choke-point `eraseEntity()` drops the entity and its anchor
  in lockstep, so the two maps never drift. Deletion gating is intentionally *not*
  re-anchored: deletions carry sensor-epoch timestamps, so
  `deletion.timestamp <= entity.timestamp` stays on the entity clock.
- **Decoded-batch snapshot cache.** Backward scrubs and jumps rebuild the entity map
  from scratch; re-parsing heavy embedded glTF every time hitched the scrub.
  `snapshot_cache_` (keyed by `SequentialUID`, byte-budgeted at `kSnapshotCacheMaxBytes`,
  oldest-UID eviction) holds decoded batches so a rebuild re-folds without re-parsing.
  Each cache entry records `store_ns` (the ingest timestamp) alongside the batch, so a
  cache-hit re-fold restores the *same* lifetime anchor a fresh parse would have produced.

## Asset resolution (`package://` for a non-ROS app)

`UrdfPackageResolver` — URI scheme dispatch first:
`file://` → absolute path; bare path → relative to the URDF's directory
(never enters the package chain); `http(s)://` → allowed only for URL-source
layers; `package://pkg/rel` → the chain below, stop at first hit:

0. **In-band embedded-asset lookup** — exact-name match against the embedded
   asset dictionary (`stepEmbeddedAsset`). The dataset source plugin (not the
   host) is what may carry such assets — e.g. files embedded in the container —
   surfaced format-neutrally via `setEmbeddedAssets`. Built and unit-tested, but
   **not yet connected to the app load path**: `MainWindow::onFileLoaded` does
   not yet surface the extracted asset map, so `setEmbeddedAssets` stays empty in
   production and this step is currently inert. (Steps 1+ — the per-source
   remembered map and auto-seeded search roots — *are* wired: `onFileLoaded` and
   `makeSceneDock` feed `setSourcePath` the loaded source path.)
1. **Remembered per-source mapping** — QSettings
   `pj_scene3d/urdf_per_source_packages`, keyed by the dataset's source path.
2. **Ancestor heuristic** — walk up from the URDF's dir (≤10 levels) looking
   for a directory whose basename == `pkg`, accepting only if
   `candidate/rel` exists (prevents wrong-folder false positives). URL
   sources use the URL-segment variant.
3. **Search roots** — ordered list auto-seeded from the URDF dir, the source
   file's dir, `$ROS_PACKAGE_PATH`, and `$AMENT_PREFIX_PATH`/`$COLCON_PREFIX_PATH`
   (+`/share`), all via `qEnvironmentVariable` — zero ROS dependency.
   Package identity is directory basename only; no `package.xml` check.
4. **Ask once** — unresolved packages surface on the layer's status text;
   a chosen root is stored per-source and globally, then pending meshes retry.

Failure semantics: never silent. Missing meshes → magenta cubes + status
counts; wrong folder picked → explicit "expected a subdirectory named <pkg>"
error; URDF latch not yet received → an indefinite 500 ms-throttled re-check
with a permanent "Waiting for …" status (no timeout escalation).

## Scene-controls panel (pj_app)

`Scene3DConfigPanel` (right side-panel) drives scene-wide state, persisted in
QSettings `pj_scene3d/scene_controls/*` and re-applied to every dock it binds:
grid style/size/divisions/visibility (`GridRenderPass` rebuilds geometry
lazily at render time; style toggles between a plain line grid and a full
two-tone **checkerboard** with the grid lines overlaid — all three colors
auto-derived from the active theme, no user color controls), TF-frame
size/opacity/visibility, mesh/collision
opacity/visibility, and the **Model/URDF** row — a File/Topic/URL source
combo plus an add button; each added robot gets a row with a remove button.
Robot layers are panel-managed: filtered out of the Topics list (the
per-layer config widget — frame prefix, color override, COLLADA up_axis,
resolution override — still exists on the layer but is not reachable from
the panel; resurrect behind an "advanced" disclosure if missed). The
phases-0B/D/B look knobs are runtime APIs with baked defaults — no app UI;
the `scene3d_mesh_viewer` demo exposes them for look-dev.
