//! WGSL assembly performed once during pipeline creation.
//! Fast vertex and fragment modules deliberately remain separate.
#[derive(Clone, Copy, Debug)]
pub(super) enum GsShader {
    Dry,
    Water,
    Underwater,
    UnderwaterFastVertex,
    UnderwaterFastFragment,
}

pub(super) fn strip_motion_field_shader_blocks(source: &str) -> String {
    let mut output = String::with_capacity(source.len());
    let mut skipping = false;
    for line in source.lines() {
        let trimmed = line.trim();
        if trimmed.starts_with("// MOTION_FIELD_BEGIN:") {
            assert!(!skipping, "motion-field shader blocks must not be nested");
            skipping = true;
            continue;
        }
        if trimmed.starts_with("// MOTION_FIELD_END:") {
            assert!(skipping, "motion-field shader block end must have a begin");
            skipping = false;
            continue;
        }
        if !skipping {
            output.push_str(line);
            output.push('\n');
        }
    }
    assert!(!skipping, "motion-field shader block must be closed");
    output
}

pub(super) fn gs_shader_source(variant: GsShader) -> String {
    let (water, underwater, fast_cut, no_depth) = match variant {
        GsShader::Dry => (false, false, false, false),
        GsShader::Water => (true, false, false, false),
        GsShader::Underwater => (true, true, false, false),
        GsShader::UnderwaterFastVertex => (true, true, true, false),
        GsShader::UnderwaterFastFragment => (true, true, true, true),
    };
    let mut source = include_str!("../gswt.wgsl").to_owned();
    for (tag, keep) in [
        ("FAST_CUT", fast_cut),
        ("FULL_CUT", !fast_cut),
        ("WATER_DEPTH", !no_depth),
        ("WATER_COLOR", no_depth),
    ] {
        source = if keep {
            // Remove only variant markers, retaining the original fallback text.
            let begin = format!("// {tag}_BEGIN:");
            let end = format!("// {tag}_END:");
            source
                .lines()
                .filter(|line| {
                    let line = line.trim_start();
                    !line.starts_with(&begin) && !line.starts_with(&end)
                })
                .map(|line| format!("{line}\n"))
                .collect()
        } else {
            strip_tagged_shader_blocks(&source, tag)
        };
    }
    let mut body = if water {
        source
    } else {
        strip_tagged_shader_blocks(&source, "WATER")
    };
    if !underwater {
        body = strip_tagged_shader_blocks(&body, "UNDERWATER");
    }
    if water {
        [
            include_str!("../camera.wgsl"),
            include_str!("../cubed_sphere.wgsl"),
            &crate::water_hits::shader_source(3),
            &if underwater {
                crate::underwater::sampling_shader(3)
            } else {
                String::new()
            },
            &body,
        ]
        .concat()
    } else {
        [
            include_str!("../camera.wgsl"),
            include_str!("../cubed_sphere.wgsl"),
            &body,
            include_str!("../gswt_dry.wgsl"),
        ]
        .concat()
    }
}

fn strip_tagged_shader_blocks(source: &str, tag: &str) -> String {
    let mut body = String::new();
    let mut skipping = false;
    let begin = format!("// {tag}_BEGIN:");
    let end = format!("// {tag}_END:");
    for line in source.lines() {
        if line.trim_start().starts_with(&begin) {
            assert!(!skipping);
            skipping = true;
            continue;
        }
        if line.trim_start().starts_with(&end) {
            assert!(skipping);
            skipping = false;
            continue;
        }
        if !skipping {
            body.push_str(line);
            body.push('\n');
        }
    }
    assert!(!skipping, "{tag} shader block must be closed");
    body
}

#[cfg(test)]
mod tests {
    use super::*;
    #[cfg(not(target_arch = "wasm32"))]
    #[test]
    fn base_and_motion_field_shaders_pass_native_wgsl_validation() {
        let instance = wgpu::Instance::new(&wgpu::InstanceDescriptor::default());
        let adapter =
            match pollster::block_on(instance.request_adapter(&wgpu::RequestAdapterOptions {
                power_preference: wgpu::PowerPreference::LowPower,
                compatible_surface: None,
                force_fallback_adapter: false,
            })) {
                Ok(adapter) => adapter,
                Err(error) => {
                    eprintln!("motion field shader test skipped: no native adapter ({error})");
                    return;
                }
            };
        let (device, _) = pollster::block_on(adapter.request_device(&wgpu::DeviceDescriptor {
            label: Some("motion field shader validation device"),
            required_features: wgpu::Features::empty(),
            required_limits: wgpu::Limits::default(),
            experimental_features: wgpu::ExperimentalFeatures::disabled(),
            memory_hints: wgpu::MemoryHints::default(),
            trace: wgpu::Trace::Off,
        }))
        .unwrap();
        let authored = gs_shader_source(GsShader::Water);
        let base = strip_motion_field_shader_blocks(&authored);
        assert!(!base.contains("@group(2)"));
        assert!(!base.contains("u_motion_field"));
        for (label, source) in [
            ("base shader", base.as_str()),
            ("motion field shader", authored.as_str()),
        ] {
            device.push_error_scope(wgpu::ErrorFilter::Validation);
            let _shader = device.create_shader_module(wgpu::ShaderModuleDescriptor {
                label: Some(label),
                source: wgpu::ShaderSource::Wgsl(source.into()),
            });
            let error = pollster::block_on(device.pop_error_scope());
            assert!(error.is_none(), "{label} WGSL validation failed: {error:?}");
        }
    }
    #[test]
    fn dry_gs_shader_has_no_water_depth_or_intersection_work() {
        let source = strip_motion_field_shader_blocks(&gs_shader_source(GsShader::Dry));
        assert!(
            !source.contains("@builtin(frag_depth)"),
            "disabled water must use a color-only fragment shader"
        );
        assert!(
            !source.contains("water_surface_hit("),
            "disabled water cannot carry the iterative solver"
        );
        assert!(
            !source.contains("water_moments"),
            "disabled water cannot evaluate water cut moments"
        );
    }
    #[test]
    fn water_gs_shader_reads_shared_hits_without_solver() {
        let source = gs_shader_source(GsShader::Water);
        assert!(
            !source.contains("fn water_surface_hit("),
            "GS must not compile the iterative water solver"
        );
        assert!(
            source.contains("load_water_hit("),
            "GS must consume the current frame's shared hit texture"
        );
    }

    #[test]
    fn underwater_shader_is_a_separate_water_only_variant() {
        let dry = gs_shader_source(GsShader::Dry);
        let wet = gs_shader_source(GsShader::Water);
        let underwater = gs_shader_source(GsShader::Underwater);
        assert!(!dry.contains("underwater_transmission"));
        assert!(!wet.contains("underwater_transmission"));
        assert!(underwater.contains("underwater_transmission"));
        assert!(underwater.contains("load_water_hit("));
    }
}
