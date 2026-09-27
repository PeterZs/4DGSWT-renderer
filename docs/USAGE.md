# Renderer usage

[README](../README.md) · [Developer reference](DEVELOPMENT.md)

## Load and navigate

Load a static six-LoD GSWT ZIP or supported dynamic archive, configure and click
**Confirm**. **Reconfig** changes configuration; reload selects another asset.

| Control | Action |
|---|---|
| WASD / IJKL | Move / look |
| Space | Sprint |
| M / P | Rendering / performance panel |
| B | Motion panel, when no text field has focus |

## Cubed Sphere

Select **Config → Surface mapping → Cubed Sphere**, then set **Tiles per face edge
(N)** and **Sphere radius**. Start with N=8, radius=20. N=1–128, including odd N,
gives 6 × N × N tiles. Confirm frames the complete sphere from outside at 95% of
the limiting viewport dimension; normal navigation remains available.

Wang labels/source directions match across cube faces for rotated-edge assets
(rotate_tile=true), including the sun assets. Separate horizontal/vertical edge
families are not guaranteed to match. Motion, Gaussian support and corner content
can still reveal seams. Plane size/scale, selective merging, proxy ground and water
are disabled. Asset density, N and LoD determine memory and FPS.

## Motion and painting

Dynamic archives expose **Scene default** for unpainted areas and **Local styles**
for named controllers. Play/pause, restart and scrubbing are in the shared header.
Motion amount, Speed and Playback variation affect the selected scope. Gentle,
Steady, Lively and Subtle are presets; Subtle is low motion, not freeze. Motion
details exposes channel gains, transitions and seed. Source & looping offers Hold,
Direct wrap, Appended transition and Blend ending into beginning for preview.

Under **Local styles → Paint this style**, use LMB on the world z=0 authoring plane.
Brush diameter is in world units and affects a vertical column. **Erase** restores
scene default; **Navigate** restores camera control. A red dashed cursor means no
valid intersection. Closing Motion ends the stroke but keeps paint. Paint stays
anchored as tiles stream; reconfiguration clears it.

More paint tools & settings includes opacity, spacing, falloff, quality, Motion
amount, Spatial variety, Grouping, Synchronize and Smooth. Amount blends local and
default results; variety/grouping choose alternate timelines and their patch size.
Synchronize sets zero variety/full grouping. Low/Default/High quality uses 8/16/32
texels per tile; the shared controller limit is 64. Duplicate copies settings with
a new identity/seed, without footprints. Show painted areas controls diagnostics;
Overlay Off is representative for playback performance.

**Save session…** writes a JSON sidecar of controllers, seeds, ranges and strokes.
Load with the original asset/tile width: checks are structural, not a content
fingerprint. Loading restarts playback, not the camera/current transition. Versions
1/2 load; limits are 16 MiB, 10,000 strokes and 200,000 path points. Stroke undo/redo
includes paint outside the cache. Removal undo restores the latest removed style
after newer strokes are undone; later style edits invalidate it. General parameter
undo is not supported.

## Water and underwater

In **M → Water**, enable water; it starts disabled and supports plane/height-map
scenes. For soft coral, raise **Water level** above terrain and enable the separate
**Underwater effect**. Nearby coral stays clear while distant geometry fades into
water color. Distances use final-world units.

| Control | Purpose |
|---|---|
| Water level / color | Mean Z / opaque upper-surface pigment |
| Amplitude / wavelength / variation | Wave geometry; zero amplitude is flat |
| Reflection / roughness | Skybox reflection strength and blur |
| Ripple strength / scale | Lighting detail, independent of geometry |
| Wave speed / Playing | Water clock, independent of Gaussian motion |
| Underwater color | Far-water tint, separate from surface pigment |
| Clear near distance | Preserve nearby colors; default 3 |
| Fog half-distance | Distance beyond clear zone for 50% contrast loss; default 15 |
| Underwater sunlight / sun direction | Scattering brightness and incoming direction |
| Light shafts / strength / width | Artificial world-space beams |
| Caustics / strength / scale | Artistic surface highlights, initially disabled |
| Caustic speed | Independent speed; default 0.25x, 0 freezes the pattern |

Amplitude is capped at 3.5% of wavelength. Water covers the committed tile window
plus one border tile. Level/phase remain fixed during streaming. Speed edits preserve
animation position; pause/global Freeze stops it, and Step advances 1/60 s when
playback is on. Enable a skybox for environment reflections.

The upper surface is opaque. The underside uses refracted sky and approximate
total internal reflection, with separate shading. Scene mirror reflections,
physical refraction through coral, foam, volumetric shadows and multiple scattering
are not implemented. Shafts/caustics are artistic and work with flat water. Caustics
light submerged GS/proxy ground before fog, never the background/interface. Increase
caustic scale for coarse LoDs; baked colors/center-depth sampling limit fine detail.

## Performance

Press **P** for mean/p95 frame, CPU, worker and GPU times, row/draw counts and upload
bytes. GPU timestamps depend on adapter support. Reset Timer clears history; disable
profiling when unneeded. Compare identical assets, camera, viewport and GPU load.

Unchanged draws reuse uploads; flat intersections can be reused while ripples,
shafts and caustics animate. Underwater work is skipped while inactive. Water
preparation covers intersections/scattering; water shading measures the interface.
Native GPU timing is not browser FPS. Canonical sorting can show alpha-sort artifacts
as geometry moves; point-cloud mode helps distinguish them from motion errors.
