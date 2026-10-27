// Rapier 2D driver: runs an engine-neutral scene (docs/scenes.md) through the library's public API.
use rapier2d::parry::query::ShapeCastOptions;
use rapier2d::prelude::*;
use std::io::Write;
use std::time::Instant;

struct Shape { kind: String, material: usize, density: f64, radius: f64, p: Vec<f64> }
struct Body { kind: u8, x: f64, y: f64, angle: f64, vx: f64, vy: f64, w: f64, bullet: bool, asleep: bool, shapes: Vec<Shape> }
struct Joint { a: usize, b: usize, ax: f64, ay: f64, bx: f64, by: f64, limit: Option<(f64, f64)>, motor: Option<(f64, f64)> }
struct Query { kind: u8, p: [f64; 5] }
struct Scene {
    name: String, dt: f64, steps: usize, gravity: (f64, f64), dump_every: usize, warmup: usize, query_reps: usize,
    materials: Vec<(f64, f64)>, bodies: Vec<Body>, joints: Vec<Joint>, queries: Vec<Query>, wakes: Vec<usize>,
}

fn load(path: &str) -> Scene {
    let text = std::fs::read_to_string(path).expect("cannot read scene");
    let mut s = Scene { name: String::new(), dt: 1.0 / 60.0, steps: 600, gravity: (0.0, -10.0), dump_every: 30, warmup: 60, query_reps: 1,
        materials: vec![], bodies: vec![], joints: vec![], queries: vec![], wakes: vec![] };
    for raw in text.lines() {
        let line = raw.split('#').next().unwrap();
        let t: Vec<&str> = line.split_whitespace().collect();
        if t.is_empty() { continue; }
        let f = |i: usize| t[i].parse::<f64>().unwrap();
        let u = |i: usize| t[i].parse::<usize>().unwrap();
        match t[0] {
            "name" => s.name = t[1].to_string(),
            "dim" => assert_eq!(t[1], "2", "dim 2 only"),
            "dt" => s.dt = f(1),
            "steps" => s.steps = u(1),
            "gravity" => s.gravity = (f(1), f(2)),
            "dump_every" => s.dump_every = u(1),
            "warmup" => s.warmup = u(1),
            "query_reps" => s.query_reps = u(1),
            "material" => s.materials.push((f(2), f(3))),
            "body" => s.bodies.push(Body { kind: match t[1] { "static" => 0, "kinematic" => 1, _ => 2 }, x: f(2), y: f(3), angle: f(4), vx: f(5), vy: f(6), w: f(7),
                bullet: t[8].contains("bullet"), asleep: t[8].contains("asleep"), shapes: vec![] }),
            "shape" => s.bodies.last_mut().unwrap().shapes.push(Shape { kind: t[1].to_string(), material: u(2), density: f(3), radius: f(4), p: t[5..].iter().map(|v| v.parse().unwrap()).collect() }),
            "joint" => {
                let mut j = Joint { a: u(2), b: u(3), ax: f(4), ay: f(5), bx: f(6), by: f(7), limit: None, motor: None };
                let mut k = 8;
                while k < t.len() {
                    match t[k] { "limit" => { j.limit = Some((f(k + 1), f(k + 2))); k += 3; } "motor" => { j.motor = Some((f(k + 1), f(k + 2))); k += 3; } o => panic!("joint option {o}") }
                }
                s.joints.push(j);
            }
            "query" => { let mut p = [0.0; 5]; for (i, v) in t[2..].iter().enumerate().take(5) { p[i] = v.parse().unwrap(); }
                s.queries.push(Query { kind: match t[1] { "ray" => 0, "overlap" => 1, _ => 2 }, p }); }
            "event" => s.wakes.push(u(1)),
            _ => {}
        }
    }
    s
}

fn colliders(sh: &Shape, s: &Scene) -> Vec<ColliderBuilder> {
    let p = &sh.p;
    let v = |x: f64, y: f64| Vector::new(x as f32, y as f32);
    let finish = |b: ColliderBuilder| b.density(sh.density as f32).friction(s.materials[sh.material].0 as f32).restitution(s.materials[sh.material].1 as f32);
    if sh.kind == "chain" { // a two-sided polyline: one plain segment per link
        return p.chunks(2).collect::<Vec<_>>().windows(2).map(|w| finish(ColliderBuilder::segment(v(w[0][0], w[0][1]), v(w[1][0], w[1][1])))).collect();
    }
    let b = match sh.kind.as_str() {
        "circle" => ColliderBuilder::ball(sh.radius as f32).position(Pose::new(v(p[0], p[1]), 0.0)),
        "capsule" => ColliderBuilder::capsule_from_endpoints(v(p[0], p[1]), v(p[2], p[3]), sh.radius as f32),
        "box" => if sh.radius > 0.0 {
                ColliderBuilder::round_cuboid(p[0] as f32, p[1] as f32, sh.radius as f32).position(Pose::new(v(p[2], p[3]), p[4] as f32))
            } else {
                ColliderBuilder::cuboid(p[0] as f32, p[1] as f32).position(Pose::new(v(p[2], p[3]), p[4] as f32))
            },
        "polygon" => {
            let pts: Vec<Vector> = p.chunks(2).map(|c| v(c[0], c[1])).collect();
            if sh.radius > 0.0 { ColliderBuilder::round_convex_hull(&pts, sh.radius as f32).unwrap() } else { ColliderBuilder::convex_hull(&pts).unwrap() }
        }
        "segment" => ColliderBuilder::segment(v(p[0], p[1]), v(p[2], p[3])),
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
    world.gravity = Vector::new(s.gravity.0 as f32, s.gravity.1 as f32);
    world.integration_parameters.dt = s.dt as f32;

    let mut handles: Vec<RigidBodyHandle> = Vec::with_capacity(s.bodies.len());
    for b in &s.bodies {
        let mut rb = match b.kind { 0 => RigidBodyBuilder::fixed(), 1 => RigidBodyBuilder::kinematic_velocity_based(), _ => RigidBodyBuilder::dynamic() }
            .translation(Vector::new(b.x as f32, b.y as f32)).rotation(b.angle as f32)
            .linvel(Vector::new(b.vx as f32, b.vy as f32)).angvel(b.w as f32);
        if b.bullet { rb = rb.ccd_enabled(true); }
        if b.asleep { rb = rb.sleeping(true); }
        let h = world.bodies.insert(rb);
        for sh in &b.shapes { for c in colliders(sh, &s) { world.insert_collider(c, Some(h)); } }
        handles.push(h);
    }
    for j in &s.joints {
        if s.bodies[j.a].kind != 2 && s.bodies[j.b].kind != 2 { continue; } // not simulated by any engine
        let mut jb = RevoluteJointBuilder::new()
            .local_anchor1(Vector::new(j.ax as f32, j.ay as f32)).local_anchor2(Vector::new(j.bx as f32, j.by as f32))
            .contacts_enabled(false);
        if let Some((lo, hi)) = j.limit { jb = jb.limits([lo as f32, hi as f32]); }
        if let Some((speed, max_torque)) = j.motor { jb = jb.motor_velocity(speed as f32, 1.0).motor_max_force(max_torque as f32); }
        world.insert_impulse_joint(handles[j.a], handles[j.b], jb);
    }

    let n = s.bodies.len();
    let mut d = vec![0f64; n * 7];
    let snapshot = |world: &PhysicsWorld, d: &mut Vec<f64>| {
        for (i, h) in handles.iter().enumerate() {
            let b = &world.bodies[*h];
            let t = b.translation(); let v = b.linvel();
            d[i * 7] = t.x as f64; d[i * 7 + 1] = t.y as f64; d[i * 7 + 2] = b.rotation().angle() as f64;
            d[i * 7 + 3] = v.x as f64; d[i * 7 + 4] = v.y as f64; d[i * 7 + 5] = b.angvel() as f64;
            d[i * 7 + 6] = if b.is_sleeping() { 0.0 } else { 1.0 };
        }
    };
    let mut dump = if timing { None } else { dump_path.as_ref().map(|p| std::io::BufWriter::new(std::fs::File::create(p).unwrap())) };
    let write_frame = |w: &mut std::io::BufWriter<std::fs::File>, step: u32, d: &Vec<f64>| {
        w.write_all(&step.to_le_bytes()).unwrap(); w.write_all(&0u32.to_le_bytes()).unwrap();
        for v in d { w.write_all(&v.to_le_bytes()).unwrap(); }
    };
    if let Some(w) = dump.as_mut() {
        w.write_all(b"OPDUMP01").unwrap(); w.write_all(&(n as u32).to_le_bytes()).unwrap(); w.write_all(&0u32.to_le_bytes()).unwrap();
        snapshot(&world, &mut d); write_frame(w, 0, &d);
    }

    let mut step_ns: Vec<u128> = Vec::with_capacity(s.steps);
    for step in 0..s.steps {
        if s.wakes.contains(&step) { for (b, h) in s.bodies.iter().zip(&handles) { if b.kind == 2 { world.bodies[*h].wake_up(true); } } }
        let t0 = Instant::now();
        pool.install(|| world.step());
        step_ns.push(t0.elapsed().as_nanos());
        if let Some(w) = dump.as_mut() {
            if (step + 1) % s.dump_every == 0 || step + 1 == s.steps { snapshot(&world, &mut d); write_frame(w, (step + 1) as u32, &d); }
        }
    }
    drop(dump);

    let mut out = String::new();
    out += &format!("engine=rapier2d\nversion=0.36.1\nconfig=parallel feature, rayon pool of {} threads, default integration parameters\nscene={}\nthreads={}\nsteps={}\nwarmup={}\nbodies={}\n", threads, s.name, threads, s.steps, s.warmup, n);
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
                            0 => { let ray = Ray::new(Vector::new(q.p[0] as f32, q.p[1] as f32), Vector::new(q.p[2] as f32, q.p[3] as f32));
                                   if qp.cast_ray(&ray, q.p[4] as f32, true).is_some() { h += 1; } }
                            1 => { let ball = Ball::new(q.p[2] as f32); let pose = Pose::new(Vector::new(q.p[0] as f32, q.p[1] as f32), 0.0);
                                   h += qp.intersect_shape(pose, &ball).count(); }
                            _ => { let ball = Ball::new(q.p[2] as f32); let pose = Pose::new(Vector::new(q.p[0] as f32, q.p[1] as f32), 0.0);
                                   let vel = Vector::new(q.p[3] as f32, q.p[4] as f32);
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
    std::fs::write(&result_path, out).unwrap();
}
