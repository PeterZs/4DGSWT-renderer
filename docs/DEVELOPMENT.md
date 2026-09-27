# Developer reference

[README](../README.md) · [Usage](USAGE.md)

## Build and validate

Use the pinned Rust toolchain (nightly-2025-10-13 with rust-src and the WASM target)
and wasm-pack 0.13.1. Cargo defaults to WASM; native commands need an explicit target.
Serve with COOP/COEP headers; WebGPU requires HTTPS or loopback.

```powershell
wasm-pack build --target web
sfz -r --coi
cargo test --offline --release --lib --target x86_64-pc-windows-msvc -- --test-threads=1
cargo fmt --all -- --check
```

On Windows, use a compatible wasm-opt on the process PATH: Binaryen 132 works;
bundled 117 can abort during optimization. Rebuild pkg/ and reload after Rust/WGSL
changes. Run GPU tests serially and separately from benchmarks. For relocated
checkouts, set FOURDGSWT_WORKSPACE_ROOT to the umbrella root for motion fixtures.

## Asset contract

The loader accepts static six-LoD ZIPs and dynamic v1, qualified v2/v3 archives.
A manifest selects strict dynamic validation; malformed motion cannot fall back
to static loading. Limits: 1 GiB compressed, 2 GiB declared decompressed, 256 MiB
per member and 512 entries. Device memory can impose smaller practical limits.

Dynamic archives begin with manifest.json and motion/basis.bin (v1/v2) or the
translation/rotation/scale banks (v3), then paired tile{tile}_lod{lod}.ply and
motion/tile{tile}_lod{lod}.bin in LoD-major, tile-major order. See the umbrella
[schemas](../../../schemas/gswt_archive). PLY permutation, merged rows, coefficients,
placement transforms and channel masks must stay aligned. Motion uses 75 samples
and clamped, non-periodic Catmull–Rom interpolation. Nonfinite positions fall back
to canonical positions; covariance packing retries canonical scale/covariance.

## Module ownership

| Module | Responsibility |
|---|---|
| state.rs / worker.rs | Frame orchestration; coalesced camera work and configuration barriers |
| wangtile.rs | Placement, LoD transitions, selective merging and sorting |
| renderer.rs | Visibility, draw preparation and pass encoding |
| renderer/culling.rs | Conservative motion envelopes and mapped/group bounds cache |
| renderer/draw.rs | Streamed index residency and compact tile-uniform batches |
| renderer/pipelines.rs / shaders.rs | Pipeline/layout ownership and named WGSL assembly |
| renderer/motion.rs | Global/authored motion GPU preparation and dispatch |
| water.rs / water_hits.rs / underwater.rs | Shared frame state, intersections and scattering |
| gui/ | Configuration, scene, water and performance panels |

## Runtime contracts

- Keep sorted draw order. Reuse unchanged index streams; configuration invalidates
  residency. Cache keys include membership/revision, not only sizes or pointers.
- Build motion envelopes once from all Catmull–Rom control hulls, including
  between-sample overshoot and canonical fallback. Apply effective per-axis gains
  after placement rotation. Finite nonnegative amplified gains remain eligible;
  sphere/unsupported gains fall back. Gain edits do not rescan Gaussian rows.
- Early rejection uses existing vertex center planes with slack. Authored draws,
  including stale membership during edits/erase, retain the fallback. Preview and
  overlay do not disable culling of other draws. Camera movement reuses mapped
  group bounds; configuration invalidates them.
- PipelineSet owns eight base/authored pipelines. Select a pair once per frame,
  without lookup allocation or dynamic dispatch. Assemble WGSL only at startup;
  keep fast underwater vertex and fragment modules separate.
- Color-only underwater rendering requires active underwater, no proxy depth,
  and a camera strictly below the entire wave slab. Base and authored pipelines
  retain motion, preview and overlays. Waterline/proxy views use full cut/depth
  handling. Only provably unnecessary cuts are skipped.
- Dry shaders have no water work. Water and GS share intersections; submerged
  surfaces share scattering. Cache keys cover camera, coverage, geometry and
  relevant lighting. Caustic offsets do not invalidate these textures.
- Authored membership is revisioned: stale sorts cannot revive erased paint.
  Failed field uploads retain dirty regions. Brush strokes stay anchored in
  Wang-world coordinates as the tile window moves.
- Cubed sphere uses a 6N by N atlas and analytic CPU/GPU Jacobians. Transform
  sorting directions as depth covectors and retain reverse LoD streams. Plane
  dimensions, proxy ground and water do not apply to sphere mapping.

## Acceptance checks

Native tests cover culling bounds, WGSL validation, uploads, motion parity, water
caches and sphere mapping. In the browser, check close/far LoDs, amplified motion,
painting/erase, camera movement, reconfiguration, and water off/submerged/waterline/
proxy modes. Use identical assets, camera, resolution and GPU load; record frame
mean/p95, Gaussian/motion/water GPU time, CPU time, rows, draws and upload bytes in P.
Native pass timings do not establish browser FPS.

The no-depth path is an accepted visual approximation, not bit-exact to full
fragment depth. Refactoring must preserve its generated WGSL, pipeline state,
draw order and fallback decisions. Shared-motion/incremental brush experiments
remain deferred and are not part of the production runtime.
