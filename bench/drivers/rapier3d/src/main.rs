// Rapier 3D driver: runs an engine-neutral 3D scene (docs/spec/scenes.md) through the library's public API.
use rapier3d::parry::query::ShapeCastOptions;
use rapier3d::prelude::*;
use std::io::Write;
use std::time::Instant;

struct Shape { kind: String, material: usize, density: f64, radius: f64, p: Vec<f64> }
struct Body { kind: u8, pos: [f64; 3], q: [f64; 4], v: [f64; 3], w: [f64; 3], bullet: bool, asleep: bool, shapes: Vec<Shape> }
struct Joint { a: usize, b: usize, pa: [f64; 3], pb: [f64; 3] }
struct Query { kind: u8, p: [f64; 8] }
struct Scene {
    name: String, dt: f64, steps: usize, gravity: [f64; 3], dump_every: usize, warmup: usize, query_reps: usize,
    materials: Vec<(f64, f64)>, bodies: Vec<Body>, joints: Vec<Joint>, queries: Vec<Query>, wakes: Vec<usize>,
}

fn load(path: &str) -> Scene {
    let text = std::fs::read_to_string(path).expect("cannot read scene");
    let mut s = Scene { name: String::new(), dt: 1.0 / 60.0, steps: 600, gravity: [0.0, -10.0, 0.0], dump_every: 30, warmup: 60, query_reps: 1,
        materials: vec![], bodies: vec![], joints: vec![], queries: vec![], wakes: vec![] };
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap();
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() { continue; }
        let f = |i: usize| t[i].parse::<f64>().unwrap();
        let u = |i: usize| t[i].parse::<usize>().unwrap();
        match t[0] {
            "name" => s.name = t[1].to_string(),
            "dim" => assert_eq!(t[1], "3", "dim 3 only"),
            "dt" => s.dt = f(1),
            "steps" => s.steps = u(1),
            "gravity" => s.gravity = [f(1), f(2), f(3)],
            "dump_every" => s.dump_every = u(1),
            "warmup" => s.warmup = u(1),
            "query_reps" => s.query_reps = u(1),
            "material" => s.materials.push((f(2), f(3))),
            "body" => s.bodies.push(Body { kind: match t[1] { "static" => 0, "kinematic" => 1, _ => 2 }, pos: [f(2), f(3), f(4)], q: [f(5), f(6), f(7), f(8)],
                v: [f(9), f(10), f(11)], w: [f(12), f(13), f(14)], bullet: t[15].contains("bullet"), asleep: t[15].contains("asleep"), shapes: vec![] }),
            "shape" => s.bodies.last_mut().unwrap().shapes.push(Shape { kind: t[1].to_string(), material: u(2), density: f(3), radius: f(4), p: t[5..].iter().map(|v| v.parse().unwrap()).collect() }),
            "joint" => s.joints.push(Joint { a: u(2), b: u(3), pa: [f(4), f(5), f(6)], pb: [f(7), f(8), f(9)] }), // point joints only
            "query" => { let mut p = [0.0; 8]; for (i, v) in t[2..].iter().enumerate().take(8) { p[i] = v.parse().unwrap(); }
                s.queries.push(Query { kind: match t[1] { "ray" => 0, "overlap" => 1, _ => 2 }, p }); }
            "event" => s.wakes.push(u(1)),
            _ => {}
        }
    }
    s
}

fn v3(p: &[f64]) -> Vector { Vector::new(p[0] as f32, p[1] as f32, p[2] as f32) }

fn colliders(sh: &Shape, s: &Scene) -> Vec<ColliderBuilder> {
    let p = &sh.p;
    let finish = |b: ColliderBuilder| b.density(sh.density as f32).friction(s.materials[sh.material].0 as f32).restitution(s.materials[sh.material].1 as f32);
    let b = match sh.kind.as_str() {
        "sphere" => ColliderBuilder::ball(sh.radius as f32).translation(v3(p)),
        "capsule" => ColliderBuilder::capsule_from_endpoints(v3(&p[0..3]), v3(&p[3..6]), sh.radius as f32),
        "box" => ColliderBuilder::cuboid(p[0] as f32, p[1] as f32, p[2] as f32).translation(v3(&p[3..6])), // local box rotations are identity in every scene
        "hull" => { let pts: Vec<Vector> = p.chunks(3).map(v3).collect(); ColliderBuilder::convex_hull(&pts).unwrap() }
        "mesh" | "heightfield" => {
            // Heightfields are given to Rapier as triangle meshes built from the same grid.
            let (verts, idx): (Vec<Vector>, Vec<[u32; 3]>) = if sh.kind == "mesh" {
                let nv = p[0] as usize; let ni = p[1] as usize;
                let verts = (0..nv).map(|i| v3(&p[2 + 3 * i..5 + 3 * i])).collect();
                let idx = (0..ni / 3).map(|t| { let o = 2 + 3 * nv + 3 * t; [p[o] as u32, p[o + 1] as u32, p[o + 2] as u32] }).collect();
                (verts, idx)
            } else {
                let nx = p[0] as usize; let nz = p[1] as usize; let cell = p[2];
                let mut verts = vec![]; for j in 0..nz { for i in 0..nx { verts.push(Vector::new((p[3] + i as f64 * cell) as f32, (p[4] + p[6 + j * nx + i]) as f32, (p[5] + j as f64 * cell) as f32)); } }
                let mut idx = vec![]; for j in 0..nz - 1 { for i in 0..nx - 1 { let a = (j * nx + i) as u32; let b = a + 1; let c = a + nx as u32; let d = c + 1; idx.push([a, c, b]); idx.push([b, c, d]); } }
                (verts, idx)
            };
            ColliderBuilder::trimesh(verts, idx).unwrap()
        }
        _ => panic!("unknown shape {}", sh.kind),
    };
    vec![finish(b)]
}

fn main() {
    let args: Vec<String> = std::env::args().collect();
    let (mut scene_path, mut dump_path, mut result_path, mut threads, mut timing) = (String::new(), None, String::new(), 1usize, false);
    let mut i = 1;
    while i + 1 < args.len() {
        match args[i].as_str() {
            "--scene" => scene_path = args[i + 1].clone(),
            "--dump" => dump_path = Some(args[i + 1].clone()),
            "--result" => result_path = args[i + 1].clone(),
            "--threads" => threads = args[i + 1].parse().unwrap(),
            "--mode" => timing = args[i + 1] == "timing",
            o => panic!("unknown argument {o}"),
        }
        i += 2;
    }
    let s = load(&scene_path);
    let pool = rayon::ThreadPoolBuilder::new().num_threads(threads).build().unwrap();

    let mut world = PhysicsWorld::new();
    world.gravity = v3(&s.gravity);
    world.integration_parameters.dt = s.dt as f32;

    let mut handles: Vec<RigidBodyHandle> = Vec::with_capacity(s.bodies.len());
    for b in &s.bodies {
        let pose = Pose::from_parts(v3(&b.pos), Rotation::from_xyzw(b.q[0] as f32, b.q[1] as f32, b.q[2] as f32, b.q[3] as f32));
        let mut rb = match b.kind { 0 => RigidBodyBuilder::fixed(), 1 => RigidBodyBuilder::kinematic_velocity_based(), _ => RigidBodyBuilder::dynamic() }
            .pose(pose).linvel(v3(&b.v)).angvel(v3(&b.w));
        if b.bullet { rb = rb.ccd_enabled(true); }
        let h = world.bodies.insert(rb);
        for sh in &b.shapes { for c in colliders(sh, &s) { world.insert_collider(c, Some(h)); } }
        handles.push(h);
    }
    for (b, h) in s.bodies.iter().zip(&handles) { if b.asleep { world.bodies[*h].sleep(); } }
    for j in &s.joints {
        if s.bodies[j.a].kind != 2 && s.bodies[j.b].kind != 2 { continue; }
        let jb = SphericalJointBuilder::new().local_anchor1(v3(&j.pa)).local_anchor2(v3(&j.pb)).contacts_enabled(false);
        world.insert_impulse_joint(handles[j.a], handles[j.b], jb);
    }

    let n = s.bodies.len();
    let mut d = vec![0f64; n * 14];
    let snapshot = |world: &PhysicsWorld, d: &mut Vec<f64>| {
        for (i, h) in handles.iter().enumerate() {
            let b = &world.bodies[*h];
            let t = b.translation(); let q = b.rotation(); let v = b.linvel(); let w = b.angvel();
            let o = i * 14;
            d[o] = t.x as f64; d[o + 1] = t.y as f64; d[o + 2] = t.z as f64;
            d[o + 3] = q.x as f64; d[o + 4] = q.y as f64; d[o + 5] = q.z as f64; d[o + 6] = q.w as f64;
            d[o + 7] = v.x as f64; d[o + 8] = v.y as f64; d[o + 9] = v.z as f64;
            d[o + 10] = w.x as f64; d[o + 11] = w.y as f64; d[o + 12] = w.z as f64;
            d[o + 13] = if b.is_sleeping() { 0.0 } else { 1.0 };
        }
    };
    let mut dump = if timing { None } else { dump_path.as_ref().map(|p| std::io::BufWriter::new(std::fs::File::create(p).unwrap())) };
    let write_frame = |w: &mut std::io::BufWriter<std::fs::File>, step: u32, d: &Vec<f64>| {
        w.write_all(&step.to_le_bytes()).unwrap(); w.write_all(&0u32.to_le_bytes()).unwrap();
        for v in d { w.write_all(&v.to_le_bytes()).unwrap(); }
    };
    if let Some(w) = dump.as_mut() {
        w.write_all(b"OPDUMP01").unwrap(); w.write_all(&(n as u32).to_le_bytes()).unwrap(); w.write_all(&3u32.to_le_bytes()).unwrap();
        snapshot(&world, &mut d); write_frame(w, 0, &d);
    }

    // Optional per-stage profile from the documented performance counters (they only measure in a build with the crate's `profiler` feature): the median of each timer over the measured steps, written when OP_PROFILE is set.
    let profiling = std::env::var_os("OP_PROFILE").is_some();
    if profiling { world.physics_pipeline.counters.enable(); }
    const PROF_NAMES: [&str; 11] = ["step", "collision_detection", "broad_phase", "narrow_phase", "solver", "assembly", "velocity_resolution", "velocity_update", "ccd", "island_construction", "update"];
    let mut prof: Vec<[f64; 11]> = Vec::new();
    let mut step_ns: Vec<u128> = Vec::with_capacity(s.steps);
    for step in 0..s.steps {
        if s.wakes.contains(&step) { for (b, h) in s.bodies.iter().zip(&handles) { if b.kind == 2 { world.bodies[*h].wake_up(true); } } }
        let t0 = Instant::now();
        pool.install(|| world.step());
        step_ns.push(t0.elapsed().as_nanos());
        if profiling && step >= s.warmup {
            let c = &world.physics_pipeline.counters;
            prof.push([c.step_time_ms(), c.collision_detection_time_ms(), c.broad_phase_time_ms(), c.narrow_phase_time_ms(), c.solver_time_ms(), c.assembly_time_ms(),
                c.velocity_resolution_time_ms(), c.velocity_update_time_ms(), c.ccd_time_ms(), c.island_construction_time_ms(), c.update_time_ms()]);
        }
        if let Some(w) = dump.as_mut() {
            if (step + 1) % s.dump_every == 0 || step + 1 == s.steps { snapshot(&world, &mut d); write_frame(w, (step + 1) as u32, &d); }
        }
    }
    drop(dump);

    let mut out = String::new();
    out += &format!("engine=rapier3d\nversion=0.36.1\nconfig=parallel feature, rayon pool of {} threads, default integration parameters, heightfields as triangle meshes\nscene={}\nthreads={}\nsteps={}\nwarmup={}\nbodies={}\n", threads, s.name, threads, s.steps, s.warmup, n);
    out += &format!("step_ns={}\n", step_ns.iter().map(|v| v.to_string()).collect::<Vec<_>>().join(","));
    if !s.queries.is_empty() {
        let names = ["ray", "overlap", "cast"];
        let mut ns = [0u128; 3]; let mut count = [0usize; 3]; let mut hits = [0usize; 3];
        let reps = s.query_reps;
        for rep in 0..reps {
            let measure = reps == 1 || rep > 0;
            for kind in 0..3u8 {
                let mut h = 0usize; let mut c = 0usize;
                let t0 = Instant::now();
                pool.install(|| {
                    let qp = world.query_pipeline();
                    for q in s.queries.iter().filter(|q| q.kind == kind) {
                        c += 1;
                        match kind {
                            0 => { let ray = Ray::new(v3(&q.p[0..3]), v3(&q.p[3..6])); if qp.cast_ray(&ray, q.p[6] as f32, true).is_some() { h += 1; } }
                            1 => { let ball = Ball::new(q.p[3] as f32); let pose = Pose::from_translation(v3(&q.p[0..3])); h += qp.intersect_shape(pose, &ball).count(); }
                            _ => { let ball = Ball::new(q.p[3] as f32); let pose = Pose::from_translation(v3(&q.p[0..3])); let vel = v3(&q.p[4..7]);
                                   let opt = ShapeCastOptions { max_time_of_impact: 1.0, target_distance: 0.0, stop_at_penetration: false, compute_impact_geometry_on_penetration: false };
                                   if qp.cast_shape(&pose, vel, &ball, opt).is_some() { h += 1; } }
                        }
                    }
                });
                if measure { ns[kind as usize] += t0.elapsed().as_nanos(); count[kind as usize] += c; hits[kind as usize] = h; }
            }
        }
        for k in 0..3 { out += &format!("query_{}_ns={}\nquery_{}_count={}\nquery_{}_hits={}\n", names[k], ns[k], names[k], count[k], names[k], hits[k]); }
    }
    if profiling && !prof.is_empty() {
        for (k, name) in PROF_NAMES.iter().enumerate() {
            let mut col: Vec<f64> = prof.iter().map(|row| row[k]).collect();
            col.sort_by(|a, b| a.partial_cmp(b).unwrap());
            out += &format!("prof_{}_ms={:.5}\n", name, col[col.len() / 2]);
        }
    }
    std::fs::write(&result_path, out).unwrap();
}
