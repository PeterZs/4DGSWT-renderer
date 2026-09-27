//! Conservative whole-draw rejection using animated center bounds.
//! Bounds follow the vertex shader's existing rejection planes, preserving splat support.
use crate::{
    camera::Camera,
    dynamic_archive::BasisBank,
    motion::{MOTION_SAMPLE_COUNT, MergedMotion, MergedMotionData},
    motion_behavior::MotionChannelGains,
    structure::*,
    utils::*,
};
use std::collections::HashMap;

#[derive(Clone, Copy, Debug)]
pub(crate) struct Bounds {
    lo: Vec3,
    hi: Vec3,
}
impl Bounds {
    fn unbounded() -> Self {
        Self {
            lo: vec3(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
            hi: vec3(f32::INFINITY, f32::INFINITY, f32::INFINITY),
        }
    }
    fn empty() -> Self {
        Self {
            lo: vec3(f32::INFINITY, f32::INFINITY, f32::INFINITY),
            hi: vec3(f32::NEG_INFINITY, f32::NEG_INFINITY, f32::NEG_INFINITY),
        }
    }
    fn include(&mut self, b: Self) {
        for c in 0..3 {
            self.lo[c] = self.lo[c].min(b.lo[c]);
            self.hi[c] = self.hi[c].max(b.hi[c]);
        }
    }
    fn visible(self, vp: Mat4) -> bool {
        if (0..3)
            .any(|i| !self.lo[i].is_finite() || !self.hi[i].is_finite() || self.lo[i] > self.hi[i])
        {
            return true;
        }
        let mut rejected = [true; 5];
        for x in [self.lo.x, self.hi.x] {
            for y in [self.lo.y, self.hi.y] {
                for z in [self.lo.z, self.hi.z] {
                    let p = vp * vec4(x, y, z, 1.0);
                    let guard = 0.001 * (1.0 + p.w.abs());
                    let d = [
                        p.x + 1.2 * p.w,
                        1.2 * p.w - p.x,
                        p.y + 1.2 * p.w,
                        1.2 * p.w - p.y,
                        0.5 * (p.z + p.w) + 1.2 * p.w,
                    ];
                    for i in 0..5 {
                        rejected[i] &= d[i] < -guard;
                    }
                }
            }
        }
        // Same five CENTER rejection planes as the existing VS, with slack.
        // No extra near/far/occlusion plane or small-splat quality cutoff.
        !rejected.into_iter().any(|v| v)
    }
}

fn basis_envelopes(bank: &BasisBank, dimensions: usize) -> Vec<[[f64; 2]; 3]> {
    (0..bank.basis_count)
        .map(|b| {
            std::array::from_fn(|c| {
                let mut lo = f64::INFINITY;
                let mut hi = f64::NEG_INFINITY;
                for s in 0..MOTION_SAMPLE_COUNT - 1 {
                    let at = |i: usize| {
                        bank.values[(b * MOTION_SAMPLE_COUNT + i) * dimensions + c] as f64
                    };
                    let (p0, p1, p2, p3) = (
                        at(s.saturating_sub(1)),
                        at(s),
                        at(s + 1),
                        at((s + 2).min(MOTION_SAMPLE_COUNT - 1)),
                    );
                    // Catmull-Rom -> cubic Bezier. The curve lies inside the control
                    // hull, including between-sample overshoot; not a sampled envelope.
                    for v in [p1, p1 + (p2 - p0) / 6.0, p2 - (p3 - p1) / 6.0, p2] {
                        lo = lo.min(v);
                        hi = hi.max(v);
                    }
                }
                [lo, hi]
            })
        })
        .collect()
}

#[derive(Clone, Copy, Debug)]
pub(crate) struct MotionBounds {
    unit: Bounds,
    // Signed displacement extrema after placement rotation, including zero.
    // Retained separately so gain edits never require scanning Gaussian rows.
    displacement: Bounds,
}

impl MotionBounds {
    fn expanded(self, gain: Vec3) -> Bounds {
        let mut b = self.unit;
        for c in 0..3 {
            if gain[c] > 1.0 {
                let extra = (gain[c] as f64) - 1.0;
                // p + g*d = (p + d) + (g - 1)*d. The unit envelope
                // contains p+d; displacement bounds contain d for every row.
                let lo = b.lo[c] as f64 + extra * self.displacement.lo[c] as f64;
                let hi = b.hi[c] as f64 + extra * self.displacement.hi[c] as f64;
                let guard = 1e-5 * (1.0 + lo.abs().max(hi.abs()));
                b.lo[c] = (lo - guard) as f32;
                b.hi[c] = (hi + guard) as f32;
            }
        }
        b
    }
}

pub(crate) fn motion_bounds(m: &MergedMotion) -> Vec<MotionBounds> {
    let (bank, k, dimensions) = match &m.data {
        MergedMotionData::Separate {
            basis_banks, top_k, ..
        } => (&basis_banks.translation, *top_k, 3),
        MergedMotionData::Legacy { basis, top_k, .. } => (basis, *top_k, 9),
    };
    let envelopes = basis_envelopes(bank, dimensions);
    let mut bounds = vec![
        MotionBounds {
            unit: Bounds::empty(),
            displacement: Bounds {
                lo: vec3(0., 0., 0.),
                hi: vec3(0., 0., 0.)
            },
        };
        m.summary.tile_count
    ];
    for lod in 0..m.summary.lod_count {
        for tile in 0..m.summary.tile_count {
            let start = m.member_offsets[lod][tile] as usize;
            let end = start + m.member_counts[lod][tile] as usize;
            for row in start..end {
                let mut delta = [[0.0f64; 2]; 3];
                let mut magnitude = [0.0f64; 3];
                if m.motion_channel_masks[row] & 1 != 0 {
                    for slot in 0..k {
                        let (id, w) = match &m.data {
                            MergedMotionData::Separate {
                                basis_ids, weights, ..
                            } => {
                                let i = row * k * 3 + slot;
                                (basis_ids[i] as usize, weights[i] as f64)
                            }
                            MergedMotionData::Legacy {
                                basis_ids, weights, ..
                            } => {
                                let i = row * k + slot;
                                (basis_ids[i] as usize, weights[i] as f64)
                            }
                        };
                        for c in 0..3 {
                            let a = w * envelopes[id][c][0];
                            let b = w * envelopes[id][c][1];
                            delta[c][0] += a.min(b);
                            delta[c][1] += a.max(b);
                            magnitude[c] += a.abs().max(b.abs());
                        }
                    }
                }
                for c in 0..3 {
                    let eps = 1e-5 * (1.0 + magnitude[c]);
                    delta[c][0] -= eps;
                    delta[c][1] += eps;
                }
                if m.transform_ids[row] == 1 {
                    let old = delta;
                    delta = [[-old[1][1], -old[1][0]], old[0], old[2]];
                } else {
                    if m.transform_ids[row] != 0 {
                        bounds[tile].unit = Bounds::unbounded();
                        continue;
                    }
                }
                let p = m.canonical[row].position;
                let lo: [f32; 3] =
                    std::array::from_fn(|c| (p[c] as f64 + delta[c][0].min(0.0) - 0.002) as f32);
                let hi: [f32; 3] =
                    std::array::from_fn(|c| (p[c] as f64 + delta[c][1].max(0.0) + 0.002) as f32);
                bounds[tile].unit.include(Bounds {
                    lo: Vec3::from(lo),
                    hi: Vec3::from(hi),
                });
                bounds[tile].displacement.include(Bounds {
                    lo: Vec3::from(std::array::from_fn::<_, 3, _>(|c| {
                        delta[c][0].min(0.0) as f32
                    })),
                    hi: Vec3::from(std::array::from_fn::<_, 3, _>(|c| {
                        delta[c][1].max(0.0) as f32
                    })),
                });
            }
        }
    }
    bounds
}

/// Invalid gains must retain all draws. Valid gains use the expanded envelope.
pub(super) fn supports_gains(gains: MotionChannelGains) -> bool {
    if !gains.master.is_finite() || gains.master < 0.0 {
        return false;
    }
    gains.translation.iter().all(|g| {
        let effective = *g * gains.master;
        g.is_finite() && *g >= 0.0 && effective.is_finite()
    })
}

#[derive(PartialEq)]
struct MappingKey {
    scene: u32,
    center: Vector2<i32>,
    scale: Vec3,
    height: f32,
    gain: Vec3,
}

pub(crate) struct GroupCulling {
    local: Vec<MotionBounds>,
    gain: Vec3,
    mapped: HashMap<(usize, usize, [u32; 3]), Bounds>,
    groups: Vec<Bounds>,
    mapping: Option<MappingKey>,
    revision: Option<u64>,
    slopes: Vec2,
    height_range: [f32; 2],
    // Immutable user terrain statistics survive camera movement and live scales.
    terrain_stats: Option<([f32; 2], Vec2)>,
}
impl GroupCulling {
    pub fn new(local: Vec<MotionBounds>) -> Self {
        Self {
            local,
            gain: vec3(1., 1., 1.),
            mapped: HashMap::new(),
            groups: vec![],
            mapping: None,
            revision: None,
            slopes: vec2(0.0, 0.0),
            height_range: [0.0; 2],
            terrain_stats: None,
        }
    }
    pub fn set_gains(&mut self, gains: MotionChannelGains) {
        debug_assert!(supports_gains(gains));
        // Sub-unit gains already fit the unit envelope, so they share its cache.
        self.gain = Vec3::from(gains.translation.map(|g| (g * gains.master).max(1.0)));
    }
    /// Called whenever configure replaces the immutable user/height-map data.
    pub fn invalidate(&mut self) {
        self.mapped.clear();
        self.groups.clear();
        self.mapping = None;
        self.revision = None;
        self.terrain_stats = None;
    }
    fn map_bounds(
        &self,
        u: &UserData,
        d: &RenderData,
        map: usize,
        tile: usize,
        offset: Vec3,
    ) -> Bounds {
        let scale = d.render_config.scene_scale;
        let Some(&b) = self.local.get(tile) else {
            return Bounds::unbounded();
        };
        let b = b.expanded(self.gain);
        if (0..3).any(|i| !b.lo[i].is_finite() || !b.hi[i].is_finite()) {
            return Bounds::unbounded();
        }
        let low = vec3(
            (b.lo.x + offset.x) * scale.x,
            (b.lo.y + offset.y) * scale.y,
            (b.lo.z + offset.z) * scale.z,
        );
        let high = vec3(
            (b.hi.x + offset.x) * scale.x,
            (b.hi.y + offset.y) * scale.y,
            (b.hi.z + offset.z) * scale.z,
        );
        if u.surface_type == SurfaceType::None {
            return Bounds { lo: low, hi: high };
        }
        let _ = map;
        let width = u.height_map_wh.x as i32;
        let height = u.height_map_wh.y as i32;
        let range_x = u.tile_map_wh.x as f32 * u.tile_width * u.height_map_scale.x;
        let range_y = u.tile_map_wh.y as f32 * u.tile_width * u.height_map_scale.y;
        let tx = |p: f32| {
            (p + u.tile_map_half_wh.x as f32 * u.tile_width) / range_x * width as f32 - 0.5
        };
        let ty = |p: f32| {
            (p + u.tile_map_half_wh.y as f32 * u.tile_width) / range_y * height as f32 - 0.5
        };
        let hz = u.height_map_scale.z * d.render_config.height_map_scale_v;
        let mut min_h = f32::INFINITY;
        let mut max_h = f32::NEG_INFINITY;
        let xs = [tx(low.x).floor() - 1.0, tx(high.x).ceil() + 1.0];
        let ys = [ty(low.y).floor() - 1.0, ty(high.y).ceil() + 1.0];
        // Extreme tiling scales must not cause unbounded CPU loops. Global
        // terrain bounds remain conservative when a footprint spans many texels.
        let small = xs
            .iter()
            .chain(ys.iter())
            .all(|v| v.is_finite() && v.abs() < 1e9)
            && (xs[1] - xs[0] + 1.0) * (ys[1] - ys[0] + 1.0) <= 4096.0;
        if !small {
            [min_h, max_h] = self.height_range;
        } else {
            // All bilinear contributors, plus a guard texel; repeating sampler.
            for y in ys[0] as i32..=ys[1] as i32 {
                for x in xs[0] as i32..=xs[1] as i32 {
                    let v = u.height_map
                        [(y.rem_euclid(height) * width + x.rem_euclid(width)) as usize]
                        * hz;
                    min_h = min_h.min(v);
                    max_h = max_h.max(v);
                }
            }
        }
        let reach = low.z.abs().max(high.z.abs());
        let min_nz = (1.0 + self.slopes.x * self.slopes.x + self.slopes.y * self.slopes.y)
            .sqrt()
            .recip();
        let z = [low.z, high.z, low.z * min_nz, high.z * min_nz];
        let margin = vec3(
            reach * self.slopes.x.min(1.0) + 0.005,
            reach * self.slopes.y.min(1.0) + 0.005,
            0.005,
        );
        Bounds {
            lo: vec3(
                low.x,
                low.y,
                min_h + z.into_iter().fold(f32::INFINITY, f32::min),
            ) - margin,
            hi: vec3(
                high.x,
                high.y,
                max_h + z.into_iter().fold(f32::NEG_INFINITY, f32::max),
            ) + margin,
        }
    }
    pub fn visibility(&mut self, camera: &Camera, u: &UserData, d: &RenderData) -> Vec<bool> {
        let scene = d.cur_scene_data.as_ref().unwrap();
        let sort = d.cur_sort_data.as_ref().unwrap();
        let scale = d.render_config.scene_scale;
        let height_scale = u.height_map_scale.z * d.render_config.height_map_scale_v;
        let terrain = u.surface_type == SurfaceType::HeightMap;
        if u.surface_type == SurfaceType::Sphere
            || (0..3).any(|i| !scale[i].is_finite() || scale[i] <= 0.0)
            || !height_scale.is_finite()
            || (terrain
                && (u.height_map_wh.x == 0
                    || u.height_map_wh.y == 0
                    || u.height_map_wh.x.checked_mul(u.height_map_wh.y)
                        != Some(u.height_map.len())
                    || u.tile_map_wh != u.tile_map_half_wh * 2 + vec2(1, 1)
                    || [u.tile_width, u.height_map_scale.x, u.height_map_scale.y]
                        .iter()
                        .any(|v| !v.is_finite() || *v <= 0.0)))
        {
            return vec![true; sort.render_data_vec.len()];
        }
        let key = MappingKey {
            scene: scene.scene_id,
            center: scene.center_coord,
            scale,
            height: height_scale,
            gain: self.gain,
        };
        if self.mapping.as_ref() != Some(&key) {
            self.mapped.clear();
            self.groups.clear();
            self.revision = None;
            self.mapping = Some(key);
            if terrain {
                let (raw_range, slope) = if let Some(stats) = self.terrain_stats {
                    stats
                } else {
                    let w = u.height_map_wh.x;
                    let h = u.height_map_wh.y;
                    let mut slope = vec2(0.0f32, 0.0f32);
                    let mut raw_range = [f32::INFINITY, f32::NEG_INFINITY];
                    for y in 0..h {
                        for x in 0..w {
                            let v = u.height_map[y * w + x];
                            if !v.is_finite() {
                                self.invalidate();
                                return vec![true; sort.render_data_vec.len()];
                            }
                            raw_range[0] = raw_range[0].min(v);
                            raw_range[1] = raw_range[1].max(v);
                            slope.x = slope.x.max((u.height_map[y * w + (x + 1) % w] - v).abs());
                            slope.y = slope.y.max((u.height_map[((y + 1) % h) * w + x] - v).abs());
                        }
                    }
                    let slope = vec2(
                        slope.x * w as f32
                            / (u.tile_map_wh.x as f32 * u.tile_width * u.height_map_scale.x),
                        slope.y * h as f32
                            / (u.tile_map_wh.y as f32 * u.tile_width * u.height_map_scale.y),
                    );
                    self.terrain_stats = Some((raw_range, slope));
                    (raw_range, slope)
                };
                let heights = [raw_range[0] * height_scale, raw_range[1] * height_scale];
                self.height_range = [heights[0].min(heights[1]), heights[0].max(heights[1])];
                self.slopes = slope * height_scale.abs() + vec2(0.0001, 0.0001);
            }
        }
        if self.revision != Some(d.sort_revision) {
            self.groups.clear();
            for (i, (key, value)) in sort.render_data_vec.iter().enumerate() {
                let mut bound = Bounds::empty();
                let mut add = |map: usize, tile: usize, offset: Vec3| {
                    let key = (
                        map,
                        tile,
                        [offset.x.to_bits(), offset.y.to_bits(), offset.z.to_bits()],
                    );
                    let b = if let Some(b) = self.mapped.get(&key) {
                        *b
                    } else {
                        let b = self.map_bounds(u, d, map, tile, offset);
                        self.mapped.insert(key, b);
                        b
                    };
                    bound.include(b);
                };
                if let Some(value) = value {
                    if key.tid.len() != value.merge_from_vec.len() || u.tile_map_wh.y == 0 {
                        self.groups.push(Bounds::unbounded());
                        continue;
                    }
                    for (&map, &(_, tile)) in value.merge_from_vec.iter().zip(&key.tid) {
                        let x = map / u.tile_map_wh.y;
                        let y = map % u.tile_map_wh.y;
                        let offset = vec3(
                            (x as i32 - u.tile_map_half_wh.x as i32 + scene.center_coord.x) as f32
                                * u.tile_width,
                            (y as i32 - u.tile_map_half_wh.y as i32 + scene.center_coord.y) as f32
                                * u.tile_width,
                            0.0,
                        );
                        add(map, tile, offset);
                    }
                } else {
                    let t = &sort.tile_instance_vec[i];
                    add(t.map_index, t.tid.1, t.tile_offset);
                }
                self.groups.push(bound);
            }
            self.revision = Some(d.sort_revision);
        }
        let vp = camera.view_proj();
        self.groups.iter().map(|b| b.visible(vp)).collect()
    }
}

#[cfg(test)]
mod tests {
    use super::*;
    fn single_row_motion() -> MergedMotion {
        let mut values = vec![0.0; MOTION_SAMPLE_COUNT * 9];
        for s in 0..MOTION_SAMPLE_COUNT {
            values[s * 9] = 20.0;
        }
        MergedMotion {
            data: MergedMotionData::Legacy {
                basis: BasisBank {
                    basis_count: 1,
                    values,
                },
                top_k: 1,
                basis_ids: vec![0],
                weights: vec![1.0],
            },
            canonical: vec![crate::motion::CanonicalGaussian {
                position: [0.0; 3],
                log_scale: [0.0; 3],
                rotation: [1.0, 0.0, 0.0, 0.0],
            }],
            transform_ids: vec![0],
            motion_channel_masks: vec![7],
            source_duration_seconds: 1.0,
            summary: crate::scene_archive::DynamicArchiveSummary {
                schema_version: 1,
                tile_count: 1,
                lod_count: 1,
                basis_count: 1,
                top_k: 1,
                total_rows: 1,
                backend: "bounds-test".into(),
            },
            member_offsets: vec![vec![0]],
            member_counts: vec![vec![1]],
        }
    }

    #[test]
    fn motion_bounds_retain_canonical_when_motion_is_disabled() {
        let m = single_row_motion();
        let b = motion_bounds(&m)[0].unit;
        assert!(
            b.lo.x <= 0.0 && b.hi.x >= 20.0,
            "must contain both canonical and animated position"
        );
    }

    #[test]
    fn amplified_bounds_contain_rotated_masked_and_blended_motion() {
        let mut m = single_row_motion();
        m.canonical[0].position = [4., -2., 7.];
        if let MergedMotionData::Legacy { basis, weights, .. } = &mut m.data {
            weights[0] = -1.7;
            for s in 0..MOTION_SAMPLE_COUNT {
                basis.values[s * 9] = (s as f32 * 0.7).sin() * 3.;
                basis.values[s * 9 + 1] = (s as f32 * 0.3).cos() - 1.;
                basis.values[s * 9 + 2] = (s as f32 * 0.2).sin();
            }
        }
        for transform in [0, 1] {
            m.transform_ids[0] = transform;
            for mask in [0, 1] {
                m.motion_channel_masks[0] = mask;
                let bound = motion_bounds(&m)[0];
                for gain in [vec3(1., 1., 1.), vec3(1.38, 1.38, 0.96), vec3(4., 0., 2.)] {
                    let expanded = bound.expanded(gain);
                    if gain == vec3(1., 1., 1.) {
                        assert_eq!(expanded.lo, bound.unit.lo);
                        assert_eq!(expanded.hi, bound.unit.hi);
                    }
                    let MergedMotionData::Legacy { basis, weights, .. } = &m.data else {
                        unreachable!()
                    };
                    let evaluate = |u| {
                        let f = crate::motion::sample_basis(basis, u);
                        let mut delta =
                            vec3(f.values[0][0], f.values[0][1], f.values[0][2]) * weights[0];
                        if mask == 0 {
                            delta = vec3(0., 0., 0.);
                        }
                        if transform == 1 {
                            delta = vec3(-delta.y, delta.x, delta.z);
                        }
                        Vec3::from(m.canonical[0].position)
                            + vec3(delta.x * gain.x, delta.y * gain.y, delta.z * gain.z)
                    };
                    for sample in 0..1001 {
                        let u = sample as f32 / 1000.;
                        let a = evaluate(u);
                        let b = evaluate(1. - u);
                        for p in [
                            a,
                            b,
                            a * 0.37 + b * 0.63,
                            Vec3::from(m.canonical[0].position),
                        ] {
                            assert!(
                                (0..3).all(|c| p[c] >= expanded.lo[c] && p[c] <= expanded.hi[c]),
                                "gain={gain:?} transform={transform} mask={mask} u={u}: {p:?} outside {expanded:?}"
                            );
                        }
                    }
                }
            }
        }
    }

    #[test]
    fn scene_scale_edit_recomputes_visibility_without_new_sort() {
        let camera = Camera::new_perspective(
            winit::dpi::PhysicalSize::new(1920, 1080),
            vec3(0., 0., 0.),
            vec3(0., 1., 0.),
            vec3(0., 0., 1.),
            degrees(45.),
            0.1,
            2400.,
        );
        let mut user = UserData::new();
        user.surface_type = SurfaceType::None;
        let mut d = RenderData::new(1);
        d.cur_scene_data = Some(SceneData::new());
        let mut tile = TileInstance::new();
        tile.tile_offset = vec3(100., 10., 0.);
        d.cur_sort_data = Some(SortData {
            scene_id: 0,
            tile_instance_vec: vec![tile],
            render_data_vec: vec![(
                RenderDataKey {
                    view_id: 0,
                    tid: vec![(0, 0)],
                    transition_status: vec![TileTransitionStatusHash::None],
                },
                None,
            )],
            authored: crate::motion_tagging::AuthoredSortMetadata::untagged(1),
        });
        let mut c = GroupCulling::new(vec![MotionBounds {
            unit: Bounds {
                lo: vec3(-1., -1., -1.),
                hi: vec3(1., 1., 1.),
            },
            displacement: Bounds {
                lo: vec3(-30., 0., 0.),
                hi: vec3(0., 0., 0.),
            },
        }]);
        assert_eq!(c.visibility(&camera, &user, &d), vec![false]);
        // Changing gains alone must invalidate mapped/group bounds, including
        // when reducing gains back to the original tightly bounded envelope.
        c.set_gains(MotionChannelGains {
            translation: [2., 1., 1.],
            master: 2.,
            ..Default::default()
        });
        assert_eq!(c.visibility(&camera, &user, &d), vec![true]);
        c.set_gains(MotionChannelGains::default());
        assert_eq!(c.visibility(&camera, &user, &d), vec![false]);
        d.render_config.scene_scale.x = 0.01;
        assert_eq!(c.visibility(&camera, &user, &d), vec![true]);

        // Reconfigure the same scene/sort with a high, constant terrain.
        user.surface_type = SurfaceType::HeightMap;
        user.tile_map_half_wh = vec2(0, 0);
        user.tile_map_wh = vec2(1, 1);
        user.height_map_wh = vec2(1, 1);
        user.height_map = vec![100.0];
        user.height_map_scale.z = 1.0;
        c.invalidate();
        assert_eq!(c.visibility(&camera, &user, &d), vec![false]);
        d.render_config.height_map_scale_v = 0.0;
        assert_eq!(c.visibility(&camera, &user, &d), vec![true]);

        // A new map center must not reuse bounds even with the same sort revision.
        d.render_config.scene_scale.x = 1.0;
        d.cur_scene_data.as_mut().unwrap().scene_id += 1;
        assert_eq!(c.visibility(&camera, &user, &d), vec![false]);

        // Unsupported mappings and invalid scales must retain all draws.
        user.surface_type = SurfaceType::Sphere;
        assert_eq!(c.visibility(&camera, &user, &d), vec![true]);
        user.surface_type = SurfaceType::None;
        d.render_config.scene_scale.x = -1.0;
        assert_eq!(c.visibility(&camera, &user, &d), vec![true]);
    }
    #[test]
    fn all_presets_and_amplified_translation_are_supported() {
        use crate::motion_behavior::{MotionBehavior, MotionBehaviorPreset};
        for preset in [
            MotionBehaviorPreset::GentleSway,
            MotionBehaviorPreset::SteadyWind,
            MotionBehaviorPreset::GustyWind,
            MotionBehaviorPreset::Calm,
        ] {
            assert!(supports_gains(MotionBehavior::from_preset(preset).gains));
        }
        let mut gains = MotionChannelGains::default();
        gains.translation[1] = 2.0;
        assert!(supports_gains(gains));
        gains.master = 0.25;
        assert!(supports_gains(gains));
        gains.translation[0] = f32::NAN;
        assert!(!supports_gains(gains));
        gains.translation[0] = -1.;
        assert!(!supports_gains(gains));
        gains.translation = [2.; 3];
        gains.master = 2.;
        assert!(supports_gains(gains));
        gains.master = f32::INFINITY;
        assert!(!supports_gains(gains));
    }
    #[test]
    fn cubic_envelope_includes_overshoot() {
        let mut values = vec![0.0; MOTION_SAMPLE_COUNT * 3];
        for i in 1..MOTION_SAMPLE_COUNT {
            values[i * 3] = 1.0;
        }
        let bank = BasisBank {
            basis_count: 1,
            values,
        };
        let bound = basis_envelopes(&bank, 3)[0][0];
        assert!(bound[1] > 1.0);
        for i in 0..1000 {
            let t = i as f64 / 999.0;
            let v = 1.0 + 0.5 * t - t * t + 0.5 * t * t * t;
            assert!(v >= bound[0] && v <= bound[1]);
        }
    }
    #[test]
    fn bounds_crossing_camera_are_retained() {
        let c = Camera::new_perspective(
            winit::dpi::PhysicalSize::new(1920, 1080),
            vec3(0., 0., 0.),
            vec3(0., 1., 0.),
            vec3(0., 0., 1.),
            degrees(45.),
            0.1,
            2400.,
        );
        assert!(
            Bounds {
                lo: vec3(-2., -2., -2.),
                hi: vec3(2., 2., 2.)
            }
            .visible(c.view_proj())
        );
        assert!(
            !Bounds {
                lo: vec3(100., 10., 0.),
                hi: vec3(101., 11., 1.)
            }
            .visible(c.view_proj())
        );
    }
}
